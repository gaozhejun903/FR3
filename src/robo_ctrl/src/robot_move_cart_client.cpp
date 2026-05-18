#include "rclcpp/rclcpp.hpp"
#include "robo_ctrl/srv/robot_move_cart.hpp"
#include "robo_ctrl/msg/tcp_pose.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("robot_move_cart_client");

    // 创建服务客户端
    auto client = node->create_client<robo_ctrl::srv::RobotMoveCart>("robot_move_cart");

    // 等待服务可用
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "等待服务时被中断");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "等待 'robot_move_cart' 服务...");
    }

    // 创建请求
    auto request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();

    // 设置TCP位姿
    request->tcp_pose.x  = 400.0; // X坐标，单位：毫米
    request->tcp_pose.y  = 0.0;   // Y坐标，单位：毫米
    request->tcp_pose.z  = 500.0; // Z坐标，单位：毫米
    request->tcp_pose.rx = 180.0; // Rx旋转角度，单位：度
    request->tcp_pose.ry = 0.0;   // Ry旋转角度，单位：度
    request->tcp_pose.rz = 180.0; // Rz旋转角度，单位：度

    request->tool         = 1;     // 工具坐标系号
    request->user         = 1;     // 工件坐标系号
    request->velocity     = 30.0;  // 速度百分比
    request->acceleration = 20.0;  // 加速度百分比
    request->ovl          = 100.0; // 速度缩放因子
    request->blend_time   = -1.0;  // 平滑时间，-1表示阻塞运动
    request->config       = -1;    // 关节空间构型，-1表示参考当前关节位置

    RCLCPP_INFO(node->get_logger(), "发送TCP移动请求...");
    RCLCPP_INFO(
        node->get_logger(),
        "TCP位姿: x=%.2f, y=%.2f, z=%.2f, rx=%.2f, ry=%.2f, rz=%.2f",
        request->tcp_pose.x,
        request->tcp_pose.y,
        request->tcp_pose.z,
        request->tcp_pose.rx,
        request->tcp_pose.ry,
        request->tcp_pose.rz
    );

    // 发送请求
    auto result_future = client->async_send_request(request);

    // 等待响应
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
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