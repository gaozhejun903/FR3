#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <iostream>
#include <cmath>

class planner {
public:
    int planning_count_ = 100; // 插值点数量
    void circular_planning_3d(
        double radius,                             // 圆弧半径
        const geometry_msgs::msg::Vector3& vector, // 圆弧原点处的切线向量
        double rotation_angle,                     // 旋转弧度 沿切向量方向
        bool x_points_to_center,                   // 是否让X轴指向圆心
        std::vector<geometry_msgs::msg::Pose>& trajectory
    );
};

void planner::circular_planning_3d(
    double radius,                             // 圆弧半径
    const geometry_msgs::msg::Vector3& vector, // 圆弧原点处的切线向量
    double rotation_angle,                     // 旋转弧度 沿切向量方向
    bool x_points_to_center,                   // 是否让X轴指向圆心
    std::vector<geometry_msgs::msg::Pose>& trajectory
) {
    // 将切向量转换为 Eigen 向量并归一化
    Eigen::Vector3d T0(vector.x, vector.y, vector.z);
    if (T0.norm() < 1e-6) {
        // 切向量为零，无法规划圆弧
        return;
    }
    T0.normalize();

    // 选择一个参考方向来确定圆弧平面
    // 这里选择与切向量最不平行的坐标轴
    Eigen::Vector3d ref_axis;
    if (std::abs(T0.x()) < 0.9) {
        ref_axis = Eigen::Vector3d(1, 0, 0);
    } else {
        ref_axis = Eigen::Vector3d(0, 1, 0);
    }

    // 计算圆弧平面的法向量
    Eigen::Vector3d N = T0.cross(ref_axis);
    N.normalize();

    // 计算径向方向（从原点指向圆心的方向）
    Eigen::Vector3d R = N.cross(T0);
    R.normalize();

    // 圆心位置：从原点沿径向方向偏移radius
    Eigen::Vector3d center = R * radius;

    // 构建正交坐标系
    Eigen::Vector3d e1 = R;  // 径向方向（从原点到圆心）
    Eigen::Vector3d e2 = T0; // 切向方向
    Eigen::Vector3d e3 = N;  // 法向方向

    // 起始位置
    Eigen::Vector3d start_position(0.0, 0.0, 0.0);

    // 插值数量
    size_t num_points = planning_count_;
    if (num_points < 2)
        num_points = 2;

    // 生成轨迹点
    for (size_t i = 0; i < num_points; ++i) {
        // 计算当前插值角度
        double phi = i * rotation_angle / (num_points - 1);

        // 计算圆弧上的位置（相对于圆心）
        Eigen::Vector3d pos_relative = radius * (cos(phi) * (-e1) + sin(phi) * e2);

        // 转换到全局坐标系
        Eigen::Vector3d position = center + pos_relative;

        // 计算姿态
        Eigen::Quaterniond q;
        if (x_points_to_center) {
            // X轴指向圆心
            Eigen::Vector3d x_axis = (center - position).normalized();
            Eigen::Vector3d z_axis = e3; // 保持法向量作为Z轴
            Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
            // 构建旋转矩阵
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix.col(0) = x_axis;
            rotation_matrix.col(1) = y_axis;
            rotation_matrix.col(2) = z_axis;

            // 仅保留z方向旋转
            Eigen::Matrix3d z_rot;
            z_rot           = Eigen::AngleAxisd(phi, z_axis);
            rotation_matrix = z_rot;

            q = Eigen::Quaterniond(rotation_matrix);
        } else {
            // 绕法向量旋转，保持相对姿态
            q = Eigen::Quaterniond(Eigen::AngleAxisd(phi, e3));
        }

        // 归一化四元数
        q.normalize();

        // 创建pose消息
        geometry_msgs::msg::Pose pose;
        pose.position.x    = position.x();
        pose.position.y    = position.y();
        pose.position.z    = position.z();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        trajectory.push_back(pose);
    }
}

// 简单测试函数
int main() {
    planner p;

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 0;
    start_pose.position.y = 0;
    start_pose.position.z = 0;
    // point to the center of the circle
    start_pose.orientation.w = 1.0;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;

    geometry_msgs::msg::Vector3 normal_vector;
    normal_vector.x = 0.0; // 假设圆弧平面
    normal_vector.y = 1.0; // 在Y轴上
    normal_vector.z = 0.0; // 平行于地面

    std::vector<geometry_msgs::msg::Pose> trajectory;
    p.circular_planning_3d(150, normal_vector, M_PI / 2, true, trajectory);

    std::cout << "Trajectory:\n";
    double last_pos[3] = { 0.0, 0.0, 0.0 };
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pose = trajectory[i];
        std::cout << "(" << start_pose.position.x + pose.position.x << ", " << start_pose.position.y + pose.position.y
                  << ", " << start_pose.position.z + pose.position.z << ",";
        std::cout << pose.orientation.w << ", " << pose.orientation.x << ", " << pose.orientation.y << ", "
                  << pose.orientation.z << "),\n";
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::cout << "Euler Angles: (" << roll * 180.0 / M_PI << ", " << pitch * 180.0 / M_PI << ", "
                  << yaw * 180.0 / M_PI << ")\n";
        std::cout << "angle raw differ" << roll * 180.0 / M_PI - last_pos[0] << ", "
                  << pitch * 180.0 / M_PI - last_pos[1] << ", " << yaw * 180.0 / M_PI - last_pos[2] << "\n";

        last_pos[0] = roll * 180.0 / M_PI;
        last_pos[1] = pitch * 180.0 / M_PI;
        last_pos[2] = yaw * 180.0 / M_PI;
    }
    // std::vector<geometry_msgs::msg::Pose> path_points = trajectory;
    // for (size_t i = 0; i < trajectory.size(); ++i) {
    //     if (i == 0)
    //         continue;
    //     trajectory[i].position.x -= path_points[i - 1].position.x;
    //     trajectory[i].position.y -= path_points[i - 1].position.y;
    //     trajectory[i].position.z -= path_points[i - 1].position.z;
    //     tf2::Quaternion q1, q2;
    //     tf2::fromMsg(path_points[i - 1].orientation, q1);
    //     tf2::fromMsg(path_points[i].orientation, q2);

    //     // 转换为欧拉角
    //     double roll1, pitch1, yaw1;
    //     double roll2, pitch2, yaw2;
    //     tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
    //     tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);

    //     // 计算欧拉角差值
    //     double roll_diff  = roll2 - roll1;
    //     double pitch_diff = pitch2 - pitch1;
    //     double yaw_diff   = yaw2 - yaw1;
    //     std::cout << "Position Difference: ("
    //               << trajectory[i].position.x << ", "
    //               << trajectory[i].position.y << ", "
    //               << trajectory[i].position.z << ","
    //                 << roll_diff * 180.0 / M_PI << ", "
    //                 << pitch_diff * 180.0 / M_PI << ", "
    //                 << yaw_diff * 180.0 / M_PI << ")\n";

    //     // 将差值重新转换为四元数
    //     tf2::Quaternion q_diff;
    //     q_diff.setRPY(roll_diff, pitch_diff, yaw_diff);
    //     trajectory[i].orientation = tf2::toMsg(q_diff);
    // }
    std::vector<geometry_msgs::msg::Pose> path_points1 = trajectory;
    for (size_t i = 0; i < trajectory.size(); ++i) {
        if (i == 0)
            continue;
        trajectory[i].position.x -= path_points1[i - 1].position.x;
        trajectory[i].position.y -= path_points1[i - 1].position.y;
        trajectory[i].position.z -= path_points1[i - 1].position.z;
        // 四元数取差
        tf2::Quaternion q1(
            path_points1[i - 1].orientation.x,
            path_points1[i - 1].orientation.y,
            path_points1[i - 1].orientation.z,
            path_points1[i - 1].orientation.w
        );
        tf2::Quaternion q2(
            trajectory[i].orientation.x,
            trajectory[i].orientation.y,
            trajectory[i].orientation.z,
            trajectory[i].orientation.w
        );
        tf2::Quaternion q_diff = q2 * q1.inverse();
        // 归一化四元数
        q_diff.normalize();
        trajectory[i].orientation.x = q_diff.x();
        trajectory[i].orientation.y = q_diff.y();
        trajectory[i].orientation.z = q_diff.z();
        trajectory[i].orientation.w = q_diff.w();
        std::cout << "Position Difference: (" << trajectory[i].position.x << ", " << trajectory[i].position.y << ", "
                  << trajectory[i].position.z << "," << trajectory[i].orientation.x << ", "
                  << trajectory[i].orientation.y << ", " << trajectory[i].orientation.z << ", "
                  << trajectory[i].orientation.w << ")\n";
    }
    return 0;
}
