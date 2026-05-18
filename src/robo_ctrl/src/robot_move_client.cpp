#include "rclcpp/rclcpp.hpp"
#include "robo_ctrl/srv/robot_move.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("robot_move_client");

    // 创建服务客户端
    auto client = node->create_client<robo_ctrl::srv::RobotMove>("robot_move");

    // 等待服务可用
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "等待服务时被中断");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "等待 'robot_move' 服务...");
    }

    // 创建请求
    auto request = std::make_shared<robo_ctrl::srv::RobotMove::Request>();

    // 关节位置移动示例
    request->move_type       = 0;                                 // 0表示关节移动
    request->joint_positions = { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 }; // 关节角度（度）
    request->velocity        = 50.0;                              // 速度百分比
    request->acceleration    = 30.0;                              // 加速度百分比

    RCLCPP_INFO(node->get_logger(), "发送关节移动请求...");

    // 发送请求
    auto result_future = client->async_send_request(request);

    // 等待响应
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get(); // 修改从 future->get() 到 future.get()
        if (response->success) {
            RCLCPP_INFO(node->get_logger(), "移动成功: %s", response->message.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "移动失败: %s", response->message.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "调用服务失败");
        return 1;
    }

    // 笛卡尔位置移动示例
    request                 = std::make_shared<robo_ctrl::srv::RobotMove::Request>();
    request->move_type      = 1;                                        // 1表示笛卡尔移动
    request->cartesian_pose = { 400.0, 0.0, 500.0, 180.0, 0.0, 180.0 }; // 笛卡尔位姿 [x, y, z, rx, ry, rz]
    request->velocity       = 30.0;                                     // 速度百分比
    request->acceleration   = 20.0;                                     // 加速度百分比

    RCLCPP_INFO(node->get_logger(), "发送笛卡尔移动请求...");

    // 发送请求
    result_future = client->async_send_request(request);

    // 等待响应
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get(); // 修改从 future->get() 到 future.get()
        if (response->success) {
            RCLCPP_INFO(node->get_logger(), "移动成功: %s", response->message.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "移动失败: %s", response->message.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "调用服务失败");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}