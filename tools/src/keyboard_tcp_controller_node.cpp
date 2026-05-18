#include "tools/keyboard_tcp_controller.hpp"
#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <opencv2/opencv.hpp>

// 声明你自己的清理函数
void cleanup() {
    std::cout << "\n[清理] 恢复终端状态并关闭窗口" << std::endl;
    cv::destroyAllWindows(); // 关闭所有 OpenCV 窗口
    // 如果用了 termios，也应恢复终端状态
    std::cout << "\033[0m" << std::flush; // 恢复颜色等终端状态
}

// 捕获 Ctrl+C (SIGINT)
void signal_handler(int signum) {
    cleanup();
    std::exit(signum);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);  // 捕获 Ctrl+C
    std::signal(SIGTERM, signal_handler); // 捕获 kill

    try {
        auto controller = std::make_shared<KeyboardTCPController>();
        controller->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "程序运行出错: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
