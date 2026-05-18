#include "depth_handler/depth_processor_node.hpp"

depth_handler::depth_node::depth_node(const rclcpp::NodeOptions& options): Node("depth_processor_node", options) {
    // 参数声明与获取
    this->declare_parameters();
    this->update_parameters();

    // 初始化TF2相关组件
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建参数回调
    params_callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(&depth_node::parameters_callback, this, std::placeholders::_1));

    // 配置ROS2发布者
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
    bbox3d_pub_     = this->create_publisher<depth_handler::msg::Bbox3dArray>(bbox3d_topic_, 10);
    image_pub_      = this->create_publisher<sensor_msgs::msg::Image>(result_visulization_topic_, 10);

    if (enable_visualization_) {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic_, 10);
    }

    // 先订阅相机信息，以获取相机内参
    temp_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&depth_node::camera_info_callback, this, std::placeholders::_1)
    );

    // 使用消息过滤器同步检测结果和深度图像 - 修改为使用正确的QoS格式
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).get_rmw_qos_profile();
    bbox_sub_.subscribe(this, bbox2d_topic_, qos_profile);
    depth_sub_.subscribe(this, depth_topic_, qos_profile);

    sync_ = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<detector::msg::Bbox2dArray, sensor_msgs::msg::Image>>>(
        message_filters::sync_policies::ApproximateTime<detector::msg::Bbox2dArray, sensor_msgs::msg::Image>(10),
        bbox_sub_,
        depth_sub_
    );

    sync_->registerCallback(std::bind(&depth_node::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Depth Processor Node initialized");
}

depth_handler::depth_node::~depth_node() {
    RCLCPP_INFO(this->get_logger(), "depth_node destroyed");
}

void depth_handler::depth_node::callback(
    const std::shared_ptr<const detector::msg::Bbox2dArray>& detectmsg,
    const std::shared_ptr<const sensor_msgs::msg::Image>& depthmsg
) {
    // RCLCPP_INFO(this->get_logger(), "msg recved");
    if (!camera_info_received_) {
        RCLCPP_INFO(this->get_logger(), "CameraInfo not received yet, subscribing to camera info topic");
        return;
    }
    // 默认图像已经对齐
    // 计算深度图像的宽高
    int width  = depthmsg->width;
    int height = depthmsg->height;
    std::vector<std::vector<Eigen::Vector3f>> points;
    std::vector<int> cluster_ids;
    for (int i = 0; i < static_cast<int>(detectmsg->results.size()); ++i) {
        auto bbox2d = detectmsg->results[i];
        // 计算深度图像的 ROI
        cv::Rect roi(bbox2d.x, bbox2d.y, bbox2d.width, bbox2d.height);
        // roi = scale_bbox(roi, width, height, camera_info_.width, camera_info_.height);
        // 检查 ROI 是否在图像范围内
        if (roi.x < 0 || roi.y < 0 || roi.x + roi.width / 2 > width || roi.y + roi.height / 2 > height) {
            RCLCPP_WARN(this->get_logger(), "ROI is out of image bounds");
            continue;
        }

        // 将 ROS 图像转换为 OpenCV Mat
        cv::Mat depth_img;
        if (depthmsg->encoding == "16UC1") {
            depth_img = cv::Mat(height, width, CV_16UC1, const_cast<uint8_t*>(depthmsg->data.data()));
        } else if (depthmsg->encoding == "32FC1") {
            depth_img = cv::Mat(height, width, CV_32FC1, const_cast<uint8_t*>(depthmsg->data.data()));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", depthmsg->encoding.c_str());
            return;
        }
        // 计算深度图像的方向
        points.push_back(depthToPoints(depth_img, pixel_directions_, roi, 0.001f));
        cluster_ids.push_back(detectmsg->results[i].class_id);
    }
    std::vector<Eigen::Vector3f> all_points;
    // 计算每个聚类的bbox3d
    std::vector<depth_handler::msg::Bbox3d> bbox3d_array;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::Marker marker_point;
    int i = 0;
    for (const auto& cluster: points) {
        if (cluster.empty()) {
            continue;
        }

        // 创建点云的副本用于坐标变换
        std::vector<Eigen::Vector3f> transformed_cluster = cluster;

        // 将点云从相机坐标系转换到robotbase坐标系
        bool transform_success = transformPointCloud(transformed_cluster, target_frame_, source_frame_, this->now());

        if (!transform_success) {
            RCLCPP_WARN(this->get_logger(), "点云坐标变换失败，使用原始点云");
            transformed_cluster = cluster; // 如果转换失败，使用原始点云
        }

        Eigen::Vector3f min_point = transformed_cluster[0];
        Eigen::Vector3f max_point = transformed_cluster[0];
        for (const auto& point: transformed_cluster) {
            min_point = min_point.cwiseMin(point);
            max_point = max_point.cwiseMax(point);
            all_points.emplace_back(point);
        }
        // 地面消除，直接使用简单的z轴过滤
        transformed_cluster.erase(
            std::remove_if(
                transformed_cluster.begin(),
                transformed_cluster.end(),
                [min_point](const Eigen::Vector3f& point) { return (abs(point.z() - min_point.z()) < 0.01f || point.x() > min_point.x() + 0.10f); }
            ),
            transformed_cluster.end()
        );
        if (transformed_cluster.empty()) {
            RCLCPP_WARN(this->get_logger(), "经过地面消除后，点云为空");
            continue;
        }
        min_point = transformed_cluster[0];
        max_point = transformed_cluster[0];
        for (const auto& point: transformed_cluster) {
            min_point = min_point.cwiseMin(point);
            max_point = max_point.cwiseMax(point);
            all_points.emplace_back(point);
        }

        depth_handler::msg::Bbox3d bbox3d;
        bbox3d.x        = min_point.x();
        bbox3d.y        = min_point.y();
        bbox3d.z        = min_point.z();
        bbox3d.width    = max_point.x() - min_point.x();
        bbox3d.height   = max_point.y() - min_point.y();
        bbox3d.depth    = max_point.z() - min_point.z();
        bbox3d.class_id = cluster_ids[i];

        // 可视化 - 现在使用robotbase坐标系
        marker.header.frame_id    = target_frame_; // 使用目标坐标系
        marker.header.stamp       = depthmsg->header.stamp;
        marker.ns                 = "bbox3d";
        marker.id                 = i;
        marker.type               = visualization_msgs::msg::Marker::CUBE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.lifetime           = rclcpp::Duration::from_seconds(marker_lifetime_);
        marker.pose.position.x    = bbox3d.x + bbox3d.width / 2;
        marker.pose.position.y    = bbox3d.y + bbox3d.height / 2;
        marker.pose.position.z    = bbox3d.z + bbox3d.depth / 2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = bbox3d.width;
        marker.scale.y            = bbox3d.height;
        marker.scale.z            = bbox3d.depth;
        marker.color.r            = 0.0;
        marker.color.g            = 1.0;
        marker.color.b            = 0.0;
        marker.color.a            = 1.0;

        marker_array.markers.push_back(marker);

        marker_point.header.frame_id    = target_frame_; // 使用目标坐标系
        marker_point.header.stamp       = depthmsg->header.stamp;
        marker_point.ns                 = "center_point";
        marker_point.id                 = i;
        marker_point.type               = visualization_msgs::msg::Marker::SPHERE;
        marker_point.action             = visualization_msgs::msg::Marker::ADD;
        marker_point.lifetime           = rclcpp::Duration::from_seconds(marker_lifetime_);
        marker_point.pose.position.x    = bbox3d.x + bbox3d.width / 2;
        marker_point.pose.position.y    = bbox3d.y + bbox3d.height / 2;
        marker_point.pose.position.z    = bbox3d.z + bbox3d.depth / 2;
        marker_point.pose.orientation.x = 0.0;
        marker_point.pose.orientation.y = 0.0;
        marker_point.pose.orientation.z = 0.0;
        marker_point.pose.orientation.w = 1.0;
        marker_point.scale.x            = 0.01;
        marker_point.scale.y            = 0.01;
        marker_point.scale.z            = 0.01;
        marker_point.color.r            = 1.0;
        marker_point.color.g            = 0.0;
        marker_point.color.b            = 0.0;
        marker_point.color.a            = 1.0;
        marker_array.markers.push_back(marker_point);
        marker_point.header.frame_id    = target_frame_; // 使用目标坐标系
        marker_point.header.stamp       = depthmsg->header.stamp;
        marker_point.ns                 = "point";
        marker_point.id                 = i;
        marker_point.type               = visualization_msgs::msg::Marker::SPHERE;
        marker_point.action             = visualization_msgs::msg::Marker::ADD;
        marker_point.lifetime           = rclcpp::Duration::from_seconds(marker_lifetime_);
        marker_point.pose.position.x    = bbox3d.x;
        marker_point.pose.position.y    = bbox3d.y;
        marker_point.pose.position.z    = bbox3d.z;
        marker_point.pose.orientation.x = 0.0;
        marker_point.pose.orientation.y = 0.0;
        marker_point.pose.orientation.z = 0.0;
        marker_point.pose.orientation.w = 1.0;
        marker_point.scale.x            = 0.01;
        marker_point.scale.y            = 0.01;
        marker_point.scale.z            = 0.01;
        marker_point.color.r            = 0.0;
        marker_point.color.g            = 0.0;
        marker_point.color.b            = 1.0;
        marker_point.color.a            = 1.0;
        marker_array.markers.push_back(marker_point);
        // std::cout << "bbox3d in " << target_frame_ << ": " << bbox3d.x << " " << bbox3d.y << " " << bbox3d.z << " "
        //           << bbox3d.width << " " << bbox3d.height << " " << bbox3d.depth << " "
        //           << "class_id: " << bbox3d.class_id << std::endl;
        bbox3d_array.push_back(bbox3d);
        i++;
    }

    // 发布可视化标记
    if (enable_visualization_ && !marker_array.markers.empty()) {
        marker_pub_->publish(marker_array);
    }

    // 发布 bbox3d
    depth_handler::msg::Bbox3dArray bbox3d_msg;
    bbox3d_msg.header          = depthmsg->header;
    bbox3d_msg.header.frame_id = target_frame_; // 使用目标坐标系
    bbox3d_msg.header.stamp    = depthmsg->header.stamp;
    bbox3d_msg.results         = bbox3d_array;
    bbox3d_pub_->publish(bbox3d_msg);

    // 发布点云
    if (enable_pointcloud_ && !all_points.empty()) {
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        eigenToPointCloud2(all_points, pointcloud_msg);
        pointcloud_msg.header          = depthmsg->header;
        pointcloud_msg.header.frame_id = target_frame_; // 使用目标坐标系
        pointcloud_msg.header.stamp    = depthmsg->header.stamp;
        pointcloud_pub_->publish(pointcloud_msg);
        // RCLCPP_INFO(this->get_logger(), "发布点云到 %s 坐标系, 点数: %zu", target_frame_.c_str(), all_points.size());
    }
};

std::vector<Eigen::Vector3f> depth_handler::depth_node::depthToPoints(
    const cv::Mat& depth_img,
    const std::vector<Eigen::Vector3f>& pixel_directions,
    float depth_scale
) {
    std::vector<Eigen::Vector3f> points;
    int width  = depth_img.cols;
    int height = depth_img.rows;

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            float depth = depth_img.at<float>(v, u) * depth_scale;
            if (depth <= 0.0f || std::isnan(depth))
                continue;

            const Eigen::Vector3f& dir = pixel_directions[v * width + u];
            points.emplace_back(dir * depth);
        }
    }
    return points;
}

std::vector<Eigen::Vector3f> depth_handler::depth_node::depthToPoints(
    const cv::Mat& depth_img,
    const std::vector<Eigen::Vector3f>& pixel_directions,
    cv::Rect roi,
    float depth_scale
) {
    cv::Mat clean_depth_img;
    depth_img.copyTo(clean_depth_img); // 重新拷贝一份真正干净的深度图

    std::vector<Eigen::Vector3f> unclassed_points;
    std::vector<Eigen::Vector3f> points;
    int width  = clean_depth_img.cols;
    int height = clean_depth_img.rows;

    roi.x      = std::clamp(roi.x - roi.width / 2, 0, width - 1);
    roi.y      = std::clamp(roi.y - roi.height / 2, 0, height - 1);
    roi.width  = std::min(roi.width, width - roi.x);
    roi.height = std::min(roi.height, height - roi.y);

    for (int v = roi.y; v < roi.y + roi.height; ++v) {
        const uint16_t* row_ptr = clean_depth_img.ptr<uint16_t>(v);
        for (int u = roi.x; u < roi.x + roi.width; ++u) {
            uint16_t raw_depth = row_ptr[u];
            float depth        = static_cast<float>(raw_depth) * depth_scale;
            if (depth <= 0.0f || std::isnan(depth))
                continue;

            size_t idx = v * width + u;
            if (idx >= pixel_directions.size()) {
                std::cerr << "Pixel index out of range: idx=" << idx << " size=" << pixel_directions.size()
                          << std::endl;
                continue;
            }

            const Eigen::Vector3f& dir = pixel_directions[idx];
            unclassed_points.emplace_back(dir * depth);
        }
    }
    // downsample
    auto start = std::chrono::high_resolution_clock::now();
    if (unclassed_points.size() > 5000) {
        std::vector<Eigen::Vector3f> downsampled_points;
        downsampled_points.reserve(5000);
        for (size_t i = 0; i < unclassed_points.size(); i += 5) {
            downsampled_points.emplace_back(unclassed_points[i]);
        }
        unclassed_points = downsampled_points;
    }
    // classify the points
    // auto result                           = clusterPointsKDTree(unclassed_points, 0.01f, 3000, 100000);
    auto result                           = voxelClustering(unclassed_points, 0.01f, 3000);
    auto end                              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    auto end2                             = std::chrono::high_resolution_clock::now();
    // return largest cluster
    for (const auto& cluster: result) {
        if (cluster.size() < 3000 || cluster.size() > 100000)
            continue;
        for (auto& point: cluster) {
            points.emplace_back(point);
        }
    }

    std::chrono::duration<double> elapsed2 = end2 - end;

    return points;
}

// 实现点云坐标变换函数
bool depth_handler::depth_node::transformPointCloud(
    std::vector<Eigen::Vector3f>& points,
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time
) {
    if (points.empty()) {
        RCLCPP_WARN(this->get_logger(), "点云为空，无法进行变换");
        return false;
    }

    try {
        // 查找坐标变换
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(target_frame, source_frame, time, rclcpp::Duration::from_seconds(1.0));

        // 将ROS变换转换为Eigen变换矩阵
        Eigen::Isometry3d transform  = tf2::transformToEigen(transform_stamped);
        Eigen::Isometry3f transformf = transform.cast<float>();

// 应用变换到每个点
#pragma omp parallel for
        for (auto& point: points) {
            point = transformf * point;
        }
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "成功将点云从 %s 坐标系转换到 %s 坐标系",
        //     source_frame.c_str(),
        //     target_frame.c_str()
        // );
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "点云坐标变换失败: %s", ex.what());
        return false;
    }
}

// 实现单个点的坐标变换
bool depth_handler::depth_node::transformPoint(
    Eigen::Vector3f& point,
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time
) {
    try {
        // 查找坐标变换
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(target_frame, source_frame, time, rclcpp::Duration::from_seconds(1.0));

        // 将ROS变换转换为Eigen变换矩阵
        Eigen::Isometry3d transform  = tf2::transformToEigen(transform_stamped);
        Eigen::Isometry3f transformf = transform.cast<float>();

        // 应用变换到点
        point = transformf * point;

        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "点坐标变换失败: %s", ex.what());
        return false;
    }
}

// 实现边界框的坐标变换
bool depth_handler::depth_node::transformBoundingBox(
    depth_handler::msg::Bbox3d& bbox,
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time
) {
    try {
        // 获取边界框的8个顶点
        std::vector<Eigen::Vector3f> vertices;
        vertices.push_back(Eigen::Vector3f(bbox.x, bbox.y, bbox.z));
        vertices.push_back(Eigen::Vector3f(bbox.x + bbox.width, bbox.y, bbox.z));
        vertices.push_back(Eigen::Vector3f(bbox.x, bbox.y + bbox.height, bbox.z));
        vertices.push_back(Eigen::Vector3f(bbox.x + bbox.width, bbox.y + bbox.height, bbox.z));
        vertices.push_back(Eigen::Vector3f(bbox.x, bbox.y, bbox.z + bbox.depth));
        vertices.push_back(Eigen::Vector3f(bbox.x + bbox.width, bbox.y, bbox.z + bbox.depth));
        vertices.push_back(Eigen::Vector3f(bbox.x, bbox.y + bbox.height, bbox.z + bbox.depth));
        vertices.push_back(Eigen::Vector3f(bbox.x + bbox.width, bbox.y + bbox.height, bbox.z + bbox.depth));

        // 变换所有顶点
        if (!transformPointCloud(vertices, target_frame, source_frame, time)) {
            return false;
        }

        // 计算变换后的包围盒
        Eigen::Vector3f min_point = vertices[0];
        Eigen::Vector3f max_point = vertices[0];

        for (const auto& vertex: vertices) {
            min_point = min_point.cwiseMin(vertex);
            max_point = max_point.cwiseMax(vertex);
        }

        // 更新边界框
        bbox.x      = min_point.x();
        bbox.y      = min_point.y();
        bbox.z      = min_point.z();
        bbox.width  = max_point.x() - min_point.x();
        bbox.height = max_point.y() - min_point.y();
        bbox.depth  = max_point.z() - min_point.z();

        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "边界框坐标变换失败: %s", ex.what());
        return false;
    }
}

void depth_handler::depth_node::declare_parameters() {
    // 主题相关参数
    this->declare_parameter<std::string>("camera_info_topic", camera_info_topic_);
    this->declare_parameter<std::string>("depth_topic", depth_topic_);
    this->declare_parameter<std::string>("image_topic", image_topic_);
    this->declare_parameter<std::string>("bbox3d_topic", bbox3d_topic_);
    this->declare_parameter<std::string>("result_visualization_topic", result_visulization_topic_);
    this->declare_parameter<std::string>("bbox2d_topic", bbox2d_topic_);
    this->declare_parameter<std::string>("pointcloud_topic", pointcloud_topic_);
    this->declare_parameter<std::string>("visualization_topic", visualization_topic_);

    // 可视化相关参数
    this->declare_parameter<bool>("enable_visualization", enable_visualization_);
    this->declare_parameter<bool>("enable_pointcloud", enable_pointcloud_);
    this->declare_parameter<double>("marker_lifetime", marker_lifetime_);
    this->declare_parameter<double>("marker_scale", marker_scale_);

    // 深度处理相关参数
    this->declare_parameter<float>("depth_scale", depth_scale_);
    this->declare_parameter<int>("min_points", min_points_);
    this->declare_parameter<float>("outlier_threshold", outlier_threshold_);
}

void depth_handler::depth_node::update_parameters() {
    // 获取主题相关参数
    camera_info_topic_         = this->get_parameter("camera_info_topic").as_string();
    depth_topic_               = this->get_parameter("depth_topic").as_string();
    image_topic_               = this->get_parameter("image_topic").as_string();
    bbox3d_topic_              = this->get_parameter("bbox3d_topic").as_string();
    result_visulization_topic_ = this->get_parameter("result_visualization_topic").as_string();
    bbox2d_topic_              = this->get_parameter("bbox2d_topic").as_string();
    pointcloud_topic_          = this->get_parameter("pointcloud_topic").as_string();
    visualization_topic_       = this->get_parameter("visualization_topic").as_string();

    // 获取可视化相关参数
    enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
    enable_pointcloud_    = this->get_parameter("enable_pointcloud").as_bool();
    marker_lifetime_      = this->get_parameter("marker_lifetime").as_double();
    marker_scale_         = this->get_parameter("marker_scale").as_double();

    // 获取深度处理相关参数
    depth_scale_       = this->get_parameter("depth_scale").as_double();
    min_points_        = this->get_parameter("min_points").as_int();
    outlier_threshold_ = this->get_parameter("outlier_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "参数已更新:");
    RCLCPP_INFO(this->get_logger(), "  相机信息主题: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  深度图主题: %s", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  RGB图像主题: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  3D边界框主题: %s", bbox3d_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  可视化: %s", enable_visualization_ ? "开启" : "关闭");
    RCLCPP_INFO(this->get_logger(), "  点云发布: %s", enable_pointcloud_ ? "开启" : "关闭");
    RCLCPP_INFO(this->get_logger(), "  深度缩放因子: %.6f", depth_scale_);
}

// 参数回调函数实现
rcl_interfaces::msg::SetParametersResult depth_handler::depth_node::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters
) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason     = "成功";

    for (const auto& param: parameters) {
        if (param.get_name() == "enable_visualization") {
            enable_visualization_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "可视化已%s", enable_visualization_ ? "启用" : "禁用");
        } else if (param.get_name() == "enable_pointcloud") {
            enable_pointcloud_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "点云发布已%s", enable_pointcloud_ ? "启用" : "禁用");
        } else if (param.get_name() == "marker_lifetime") {
            marker_lifetime_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "标记存在时间更新为: %.2f 秒", marker_lifetime_);
        } else if (param.get_name() == "marker_scale") {
            marker_scale_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "标记缩放比例更新为: %.2f", marker_scale_);
        } else if (param.get_name() == "depth_scale") {
            depth_scale_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "深度缩放因子更新为: %.6f", depth_scale_);
        } else if (param.get_name() == "min_points") {
            min_points_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "最小点云数量更新为: %d", min_points_);
        } else if (param.get_name() == "outlier_threshold") {
            outlier_threshold_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "异常值阈值更新为: %.4f", outlier_threshold_);
        }
    }

    return result;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<depth_handler::depth_node>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}