#ifndef KEYBOARD_TCP_CONTROLLER_HPP
#define KEYBOARD_TCP_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <robo_ctrl/srv/robot_servo_line.hpp>
#include <robo_ctrl/msg/robot_state.hpp>
#include <robo_ctrl/msg/tcp_pose.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardTCPController: public rclcpp::Node {
public:
    KeyboardTCPController();
    ~KeyboardTCPController();

    void run();

private:
    // ROS相关
    rclcpp::Client<robo_ctrl::srv::RobotServoLine>::SharedPtr servo_client_;
    rclcpp::Subscription<robo_ctrl::msg::RobotState>::SharedPtr robot_state_sub_;

    // 机器人状态
    robo_ctrl::msg::RobotState::SharedPtr robot_state_msg_;
    std::mutex robot_mutex_;

    // 当前TCP位姿 [x, y, z, rx, ry, rz]
    std::vector<double> current_pose_;
    std::mutex pose_mutex_;

    // 控制参数
    double linear_step_;  // mm
    double angular_step_; // degrees
    double velocity_;     // 速度百分比
    double acceleration_; // 加速度百分比
    double cmd_time_;     // 指令周期
    double filter_time_;  // 滤波时间

    // 控制状态
    std::atomic<bool> running_;
    std::atomic<bool> servo_started_;

    // OpenCV窗口
    cv::Mat display_image_;
    std::mutex image_mutex_;

    // 终端设置
    struct termios old_terminal_settings_;
    bool terminal_settings_saved_;

    // 方法
    void robot_state_callback(const robo_ctrl::msg::RobotState::SharedPtr msg);
    void setup_terminal();
    void restore_terminal();
    char get_key();
    void process_key(char key);
    void start_servo_mode();
    void stop_servo_mode();
    void send_incremental_pose(const std::vector<double>& delta_pose);
    void reset_pose();
    void update_display();
    void create_display_window();
    void handle_opencv_events();
    void print_help();
};

#endif // KEYBOARD_TCP_CONTROLLER_HPP
