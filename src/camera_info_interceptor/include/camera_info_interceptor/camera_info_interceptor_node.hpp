#ifndef CAMERA_INFO_INTERCEPTOR_NODE_HPP
#define CAMERA_INFO_INTERCEPTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

namespace camera_info_interceptor {

class CameraInfoInterceptorNode: public rclcpp::Node {
public:
    explicit CameraInfoInterceptorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CameraInfoInterceptorNode() = default;

private:
    // 处理接收的camera_info消息
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // 订阅器和发布器
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // 参数
    std::string source_frame_id_;
    std::string target_frame_id_;
    std::string input_topic_;
    std::string output_topic_;
};

} // namespace camera_info_interceptor

#endif // CAMERA_INFO_INTERCEPTOR_NODE_HPP
