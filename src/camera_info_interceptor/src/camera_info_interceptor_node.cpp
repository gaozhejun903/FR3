#include <rclcpp/rclcpp.hpp>
#include "camera_info_interceptor/camera_info_interceptor_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<camera_info_interceptor::CameraInfoInterceptorNode>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
