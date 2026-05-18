#include "rclcpp/rclcpp.hpp"
#include "robo_ctrl/robo_ctrl_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<robo_ctrl::RoboCtrlNode>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}