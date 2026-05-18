#include <nanoflann.hpp>
#include <queue>
#include <set>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "omp.h"

// 包装类：将 std::vector<Eigen::Vector3f> 适配给 nanoflann
struct PointCloudAdaptor {
    const std::vector<Eigen::Vector3f>& pts;

    PointCloudAdaptor(const std::vector<Eigen::Vector3f>& points): pts(points) {}

    // nanoflann 需要这些接口
    inline size_t kdtree_get_point_count() const {
        return pts.size();
    }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx][dim];
    }

    template<class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
        return false;
    }
};

// 修改KDTree定义，显式指定索引类型为size_t
using KDTree = nanoflann::
    KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3, size_t>;

std::vector<std::vector<Eigen::Vector3f>> clusterPointsKDTree(
    const std::vector<Eigen::Vector3f>& points,
    float cluster_tolerance,
    int min_cluster_size = 1,
    int max_cluster_size = 100000
) {
    std::vector<std::vector<Eigen::Vector3f>> clusters;
    std::vector<bool> processed(points.size(), false);

    PointCloudAdaptor pcAdaptor(points);
    KDTree tree(3, pcAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

#pragma omp parallel for schedule(dynamic, 30000)
    for (size_t i = 0; i < points.size(); ++i) {
        if (processed[i])
            continue;

        std::vector<size_t> cluster_indices;
        std::queue<size_t> search_queue;

        search_queue.push(i);
        processed[i] = true;

        while (!search_queue.empty()) {
            size_t idx = search_queue.front();
            search_queue.pop();

            cluster_indices.push_back(idx);

            std::vector<std::pair<size_t, float>> matches;
            nanoflann::SearchParams params;
            tree.radiusSearch(&points[idx][0], cluster_tolerance * cluster_tolerance, matches, params);

            for (auto& match: matches) {
                size_t j = match.first;
                if (!processed[j]) {
                    processed[j] = true;
                    search_queue.push(j);
                }
            }
        }

        if (static_cast<int>(cluster_indices.size()) >= min_cluster_size
            && static_cast<int>(cluster_indices.size()) <= max_cluster_size) {
            std::vector<Eigen::Vector3f> cluster;
            for (auto idx: cluster_indices) {
                cluster.push_back(points[idx]);
            }
#pragma omp critical
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

struct VoxelKeyHash {
    std::size_t operator()(const Eigen::Vector3i& k) const {
        return std::hash<int>()(k.x()) ^ std::hash<int>()(k.y() << 1) ^ std::hash<int>()(k.z() << 2);
    }
};

// 相等判定器
struct VoxelKeyEqual {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
        return a == b;
    }
};

// 聚类函数
std::vector<std::vector<Eigen::Vector3f>>
voxelClustering(const std::vector<Eigen::Vector3f>& points, float voxel_size = 0.05, int min_cluster_size = 10) {
    using Key      = Eigen::Vector3i;
    using VoxelMap = std::unordered_map<Key, std::vector<Eigen::Vector3f>, VoxelKeyHash, VoxelKeyEqual>;

    VoxelMap voxel_map;

// 第一步：构建 Voxel Map（并行）
#pragma omp parallel
    {
        VoxelMap local_map;

#pragma omp for nowait
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& p = points[i];
            Key key       = (p / voxel_size).array().floor().cast<int>();
            local_map[key].push_back(p);
        }

#pragma omp critical
        {
            for (auto& pair: local_map) {
                voxel_map[pair.first].insert(voxel_map[pair.first].end(), pair.second.begin(), pair.second.end());
            }
        }
    }

    // 第二步：聚类（BFS）每个 voxel 的邻接性
    std::unordered_set<Key, VoxelKeyHash, VoxelKeyEqual> visited;
    std::vector<std::vector<Eigen::Vector3f>> clusters;

    const std::vector<Key> neighbors = { { 1, 0, 0 },   { -1, 0, 0 }, { 0, 1, 0 },   { 0, -1, 0 },  { 0, 0, 1 },
                                         { 0, 0, -1 },  { 1, 1, 0 },  { 1, -1, 0 },  { -1, 1, 0 },  { -1, -1, 0 },
                                         { 1, 0, 1 },   { -1, 0, 1 }, { 1, 0, -1 },  { -1, 0, -1 }, { 0, 1, 1 },
                                         { 0, -1, 1 },  { 0, 1, -1 }, { 0, -1, -1 }, { 1, 1, 1 },   { 1, 1, -1 },
                                         { 1, -1, 1 },  { -1, 1, 1 }, { -1, -1, 1 }, { -1, 1, -1 }, { 1, -1, -1 },
                                         { -1, -1, -1 } };

    for (const auto& pair: voxel_map) {
        const Key& seed = pair.first;

        if (visited.count(seed))
            continue;

        std::queue<Key> q;
        q.push(seed);
        visited.insert(seed);

        std::vector<Eigen::Vector3f> cluster;

        while (!q.empty()) {
            Key current = q.front();
            q.pop();
            const auto& points_in_voxel = voxel_map[current];
            cluster.insert(cluster.end(), points_in_voxel.begin(), points_in_voxel.end());

            for (const auto& offset: neighbors) {
                Key neighbor = current + offset;
                if (voxel_map.count(neighbor) && !visited.count(neighbor)) {
                    visited.insert(neighbor);
                    q.push(neighbor);
                }
            }
        }

        if (static_cast<int>(cluster.size()) >= min_cluster_size) {
            clusters.push_back(std::move(cluster));
        }
    }

    return clusters;
}