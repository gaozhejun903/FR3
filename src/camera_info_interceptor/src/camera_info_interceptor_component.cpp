#include "camera_info_interceptor/camera_info_interceptor_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace camera_info_interceptor {

CameraInfoInterceptorNode::CameraInfoInterceptorNode(const rclcpp::NodeOptions& options):
    Node("camera_info_interceptor", options) {
    // 声明并获取参数
    this->declare_parameter("input_topic", "/camera/color/camera_info");
    this->declare_parameter("output_topic", "/camera/camera_info");
    this->declare_parameter("target_frame_id", "camera_depth_frame");

    input_topic_     = this->get_parameter("input_topic").as_string();
    output_topic_    = this->get_parameter("output_topic").as_string();
    target_frame_id_ = this->get_parameter("target_frame_id").as_string();

    // 创建订阅器和发布器
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        input_topic_,
        10,
        std::bind(&CameraInfoInterceptorNode::camera_info_callback, this, std::placeholders::_1)
    );

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "camera_info_interceptor已启动");
    RCLCPP_INFO(this->get_logger(), "监听: %s, 发布: %s", input_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "将frame_id更改为: %s", target_frame_id_.c_str());
}

void CameraInfoInterceptorNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // 创建消息的副本
    auto modified_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(*msg);

    // 修改frame_id
    modified_msg->header.frame_id = target_frame_id_;

    // 发布修改后的消息
    camera_info_pub_->publish(std::move(*modified_msg));

    RCLCPP_DEBUG(
        this->get_logger(),
        "拦截并转换消息，原始frame_id: %s, 新frame_id: %s",
        msg->header.frame_id.c_str(),
        target_frame_id_.c_str()
    );
}

} // namespace camera_info_interceptor

// 注册组件
RCLCPP_COMPONENTS_REGISTER_NODE(camera_info_interceptor::CameraInfoInterceptorNode)
