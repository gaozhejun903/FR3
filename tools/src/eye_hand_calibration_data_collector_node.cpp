#include "rclcpp/rclcpp.hpp"
#include "tools/eye_hand_calibration_data_collector.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<utils::EyeHandCalibrationDataCollector>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}