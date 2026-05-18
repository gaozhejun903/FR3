#include <rclcpp/rclcpp.hpp>
#include "tools/dual_end_tf_collector.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<utils::DualEndTfCollector>(options);

    RCLCPP_INFO(node->get_logger(), "Starting Dual End TF Collector Node");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
