#include "tools/keyboard_tcp_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>

KeyboardTCPController::KeyboardTCPController():
    Node("keyboard_tcp_controller"),
    current_pose_(6, 0.0),
    linear_step_(1.0),
    angular_step_(1.0),
    velocity_(20.0),
    acceleration_(20.0),
    cmd_time_(0.08),
    filter_time_(0.0),
    running_(true),
    servo_started_(false),
    terminal_settings_saved_(false) {
    // 声明参数
    this->declare_parameter<std::string>("robot_name", "/R");
    std::string robot_name;
    this->get_parameter("robot_name", robot_name);

    // 创建服务客户端
    servo_client_ = this->create_client<robo_ctrl::srv::RobotServoLine>(robot_name + "/robot_servo_line");

    // 创建机器人状态订阅器
    robot_state_sub_ = this->create_subscription<robo_ctrl::msg::RobotState>(
        robot_name + "/robot_state",
        10,
        std::bind(&KeyboardTCPController::robot_state_callback, this, std::placeholders::_1)
    );

    // 等待服务可用
    while (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "等待机器人服务可用...");
        if (!rclcpp::ok()) {
            return;
        }
    }

    // 设置终端
    setup_terminal();

    // 尝试创建显示窗口
    try {
        create_display_window();
        RCLCPP_INFO(this->get_logger(), "图形界面模式已启用");
    } catch (const cv::Exception& e) {
        RCLCPP_WARN(this->get_logger(), "无法创建图形界面，将使用终端模式: %s", e.what());
        // 在无GUI环境下，仅使用终端控制
        display_image_ = cv::Mat(); // 设置为空，表示无GUI模式
    }

    RCLCPP_INFO(this->get_logger(), "键盘TCP控制器已启动");
    print_help();
}

KeyboardTCPController::~KeyboardTCPController() {
    running_ = false;
    stop_servo_mode();
    restore_terminal();
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "键盘TCP控制器已关闭");
}

void KeyboardTCPController::setup_terminal() {
    // 保存当前终端设置
    if (tcgetattr(STDIN_FILENO, &old_terminal_settings_) == 0) {
        terminal_settings_saved_ = true;

        // 设置非阻塞输入
        struct termios new_settings = old_terminal_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VMIN]  = 0;
        new_settings.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

        // 设置非阻塞模式
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

void KeyboardTCPController::restore_terminal() {
    if (terminal_settings_saved_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);

        // 恢复阻塞模式
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
}

char KeyboardTCPController::get_key() {
    char key = 0;
    if (read(STDIN_FILENO, &key, 1) > 0) {
        return key;
    }
    return 0;
}

void KeyboardTCPController::robot_state_callback(const robo_ctrl::msg::RobotState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    robot_state_msg_ = msg;

    // 更新当前位姿显示
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    // 注意：这里假设RobotState消息中包含TCP位姿信息
    // 实际实现时需要根据具体的消息结构调整
}

void KeyboardTCPController::create_display_window() {
    RCLCPP_INFO(this->get_logger(), "正在创建OpenCV显示窗口...");

    // 创建图像
    display_image_ = cv::Mat::zeros(600, 800, CV_8UC3);

    // 创建窗口
    cv::namedWindow("机器人TCP位姿控制", cv::WINDOW_AUTOSIZE);

    // 初始显示
    update_display();

    // 强制显示窗口
    cv::waitKey(1);

    RCLCPP_INFO(this->get_logger(), "OpenCV窗口创建完成");
}

void KeyboardTCPController::update_display() {
    // 检查是否在GUI模式
    if (display_image_.empty()) {
        return; // 无GUI模式，不更新显示
    }

    std::lock_guard<std::mutex> lock(image_mutex_);

    // 清空图像
    display_image_ = cv::Scalar(50, 50, 50); // 深灰色背景

    // 设置字体
    int font          = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness     = 2;
    cv::Scalar text_color(255, 255, 255); // 白色文字
    cv::Scalar value_color(0, 255, 0);    // 绿色数值
    cv::Scalar help_color(200, 200, 200); // 浅灰色帮助文字

    // 标题
    cv::putText(display_image_, "TCP Pose Controller", cv::Point(250, 40), font, 1.2, cv::Scalar(0, 255, 255), 3);

    // 获取当前位姿
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);

    // 显示当前TCP位姿
    int y_offset    = 100;
    int line_height = 35;

    cv::putText(display_image_, "Current TCP Pose:", cv::Point(50, y_offset), font, font_scale, text_color, thickness);

    y_offset += line_height + 10;

    // 位置信息
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss.str("");
    ss << "X: " << current_pose_[0] << " mm";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, font_scale, value_color, thickness);
    y_offset += line_height;

    ss.str("");
    ss << "Y: " << current_pose_[1] << " mm";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, font_scale, value_color, thickness);
    y_offset += line_height;

    ss.str("");
    ss << "Z: " << current_pose_[2] << " mm";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, font_scale, value_color, thickness);
    y_offset += line_height + 10;

    // 姿态信息
    ss.str("");
    ss << "Rx: " << current_pose_[3] << " deg";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, font_scale, value_color, thickness);
    y_offset += line_height;

    ss.str("");
    ss << "Ry: " << current_pose_[4] << " deg";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, font_scale, value_color, thickness);
    y_offset += line_height;

    ss.str("");
    ss << "Rz: " << current_pose_[5] << " deg";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, font_scale, value_color, thickness);
    y_offset += line_height + 20;

    // 控制参数
    cv::putText(display_image_, "Control Settings:", cv::Point(50, y_offset), font, font_scale, text_color, thickness);
    y_offset += line_height;

    ss.str("");
    ss << "Linear Step: " << linear_step_ << " mm";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, 0.6, help_color, 1);
    y_offset += 25;

    ss.str("");
    ss << "Angular Step: " << angular_step_ << " deg";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, 0.6, help_color, 1);
    y_offset += 25;

    ss.str("");
    ss << "Velocity: " << velocity_ << "%";
    cv::putText(display_image_, ss.str(), cv::Point(70, y_offset), font, 0.6, help_color, 1);
    y_offset += 25;

    // 控制说明
    y_offset += 20;
    cv::putText(display_image_, "Controls:", cv::Point(400, 100), font, font_scale, text_color, thickness);

    std::vector<std::string> controls = { "w/s: Forward/Backward (X)", "a/d: Left/Right (Y)",
                                          "Space/z: Up/Down (Z)",      "q/e: Rotate Z+/-",
                                          "Arrow Keys: Rotate X/Y",    "1-9: Set step size",
                                          "r: Reset to zero",          "ESC: Exit" };

    int ctrl_y = 130;
    for (const auto& ctrl: controls) {
        cv::putText(display_image_, ctrl, cv::Point(420, ctrl_y), font, 0.5, help_color, 1);
        ctrl_y += 25;
    }

    // 状态指示
    cv::Scalar status_color = servo_started_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    std::string status_text = servo_started_ ? "SERVO ACTIVE" : "SERVO INACTIVE";
    cv::putText(display_image_, status_text, cv::Point(400, 400), font, 0.8, status_color, 2);

    // 显示图像
    cv::imshow("机器人TCP位姿控制", display_image_);

    // 强制刷新显示
    cv::waitKey(1);
}

void KeyboardTCPController::handle_opencv_events() {
    int key = cv::waitKey(1) & 0xFF;

    if (key != 255) {
        RCLCPP_INFO(this->get_logger(), "接收到键码: %d", key); // ✅ 输出按键值

        std::vector<double> delta_pose(6, 0.0);
        bool has_movement = false;

        switch (key) {
            case 82:                           // Up
                delta_pose[4] = angular_step_; // +Ry
                has_movement  = true;
                break;
            case 84:                            // Down
                delta_pose[4] = -angular_step_; // -Ry
                has_movement  = true;
                break;
            case 83:                           // Right
                delta_pose[3] = angular_step_; // +Rx
                has_movement  = true;
                break;
            case 81:                            // Left
                delta_pose[3] = -angular_step_; // -Rx
                has_movement  = true;
                break;
            default:
                // 不是方向键，转发给字符处理函数
                process_key(static_cast<char>(key));
                return;
        }

        if (has_movement) {
            send_incremental_pose(delta_pose);

            std::lock_guard<std::mutex> lock(pose_mutex_);
            for (size_t i = 0; i < 6; ++i) {
                current_pose_[i] += delta_pose[i];
            }

            RCLCPP_INFO(
                this->get_logger(),
                "当前位姿: X=%.1f, Y=%.1f, Z=%.1f, Rx=%.1f, Ry=%.1f, Rz=%.1f",
                current_pose_[0],
                current_pose_[1],
                current_pose_[2],
                current_pose_[3],
                current_pose_[4],
                current_pose_[5]
            );
        }
    }
}

void KeyboardTCPController::process_key(char key) {
    std::vector<double> delta_pose(6, 0.0);

    switch (key) {
        // 平移控制
        case 'w':
        case 'W':
            delta_pose[0] = linear_step_; // +X
            break;
        case 's':
        case 'S':
            delta_pose[0] = -linear_step_; // -X
            break;
        case 'a':
        case 'A':
            delta_pose[1] = linear_step_; // +Y
            break;
        case 'd':
        case 'D':
            delta_pose[1] = -linear_step_; // -Y
            break;
        case ' ':                         // Space
            delta_pose[2] = linear_step_; // +Z
            break;
        case 'z':
        case 'Z':
            delta_pose[2] = -linear_step_; // -Z
            break;

        // 旋转控制
        case 'q':
        case 'Q':
            delta_pose[5] = angular_step_; // +Rz
            break;
        case 'e':
        case 'E':
            delta_pose[5] = -angular_step_; // -Rz
            break;

        // 数字键设置步进值
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9': {
            double step_value = static_cast<double>(key - '0');
            linear_step_      = step_value;
            angular_step_     = step_value;
            RCLCPP_INFO(this->get_logger(), "步进值设置为: %.1f mm / %.1f 度", linear_step_, angular_step_);
        } break;
        case '0':
            linear_step_  = 10.0;
            angular_step_ = 10.0;
            RCLCPP_INFO(this->get_logger(), "步进值设置为: %.1f mm / %.1f 度", linear_step_, angular_step_);
            break;

        // 特殊功能键
        case 'r':
        case 'R':
            reset_pose();
            break;
        case 27: // ESC
            running_ = false;
            break;
    }

    // 处理方向键（通过OpenCV窗口事件处理）
    // 这里不处理方向键，因为它们会通过handle_opencv_events处理

    // 发送运动指令
    bool has_movement = false;
    for (double val: delta_pose) {
        if (std::abs(val) > 0.001) {
            has_movement = true;
            break;
        }
    }

    if (has_movement) {
        send_incremental_pose(delta_pose);

        // 更新当前位姿
        std::lock_guard<std::mutex> lock(pose_mutex_);
        for (size_t i = 0; i < 6; ++i) {
            current_pose_[i] += delta_pose[i];
        }

        RCLCPP_INFO(
            this->get_logger(),
            "当前位姿: X=%.1f, Y=%.1f, Z=%.1f, Rx=%.1f, Ry=%.1f, Rz=%.1f",
            current_pose_[0],
            current_pose_[1],
            current_pose_[2],
            current_pose_[3],
            current_pose_[4],
            current_pose_[5]
        );
    }
}

void KeyboardTCPController::start_servo_mode() {
    if (!servo_started_) {
        auto request             = std::make_shared<robo_ctrl::srv::RobotServoLine::Request>();
        request->command_type    = 0; // ServoMoveStart
        request->acc             = acceleration_;
        request->vel             = velocity_;
        request->cmd_time        = cmd_time_;
        request->filter_time     = filter_time_;
        request->gain            = 0.0;
        request->point_count     = 0;
        request->use_incremental = true; // 使用增量模式

        // 创建空的位姿数组
        request->cartesian_pose = geometry_msgs::msg::PoseArray();

        auto future = servo_client_->async_send_request(request);

        // 等待响应
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                servo_started_ = true;
                RCLCPP_INFO(this->get_logger(), "伺服模式已启动");
            } else {
                RCLCPP_ERROR(this->get_logger(), "启动伺服模式失败: %s", response->message.c_str());
            }
        }
    }
}

void KeyboardTCPController::stop_servo_mode() {
    if (servo_started_) {
        auto request          = std::make_shared<robo_ctrl::srv::RobotServoLine::Request>();
        request->command_type = 1; // ServoMoveEnd

        auto future = servo_client_->async_send_request(request);

        // 等待响应
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                servo_started_ = false;
                RCLCPP_INFO(this->get_logger(), "伺服模式已停止");
            } else {
                RCLCPP_ERROR(this->get_logger(), "停止伺服模式失败: %s", response->message.c_str());
            }
        }
    }
}

void KeyboardTCPController::send_incremental_pose(const std::vector<double>& delta_pose) {
    if (!servo_started_) {
        start_servo_mode();
    }

    // 创建增量位姿
    geometry_msgs::msg::Pose pose;
    pose.position.x = delta_pose[0]; // mm转m
    pose.position.y = delta_pose[1];
    pose.position.z = delta_pose[2];

    // 角度转弧度
    tf2::Quaternion q;
    q.setRPY(
        delta_pose[3] * M_PI / 180.0, // Rx
        delta_pose[4] * M_PI / 180.0, // Ry
        delta_pose[5] * M_PI / 180.0  // Rz
    );
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // 创建请求
    auto request                  = std::make_shared<robo_ctrl::srv::RobotServoLine::Request>();
    request->command_type         = 0; // ServoMoveStart
    request->cartesian_pose.poses = { pose };
    request->acc                  = acceleration_;
    request->vel                  = velocity_;
    request->cmd_time             = cmd_time_;
    request->filter_time          = filter_time_;
    request->gain                 = 0.0;
    request->point_count          = 1;
    request->use_incremental      = true;

    // 异步发送请求
    auto future = servo_client_->async_send_request(request);
}

void KeyboardTCPController::reset_pose() {
    RCLCPP_INFO(this->get_logger(), "重置到零位...");
    std::vector<double> reset_delta(6, 0.0);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    for (size_t i = 0; i < 6; ++i) {
        reset_delta[i]   = -current_pose_[i];
        current_pose_[i] = 0.0;
    }

    send_incremental_pose(reset_delta);
}

void KeyboardTCPController::print_help() {
    std::cout << "\n=== 键盘TCP位姿控制器 (C++/OpenCV版) ===\n";
    std::cout << "控制键说明：\n";
    std::cout << "  w/s     : 前进/后退 (X轴)\n";
    std::cout << "  a/d     : 左移/右移 (Y轴)\n";
    std::cout << "  space/z : 上升/下降 (Z轴)\n";
    std::cout << "  q/e     : 绕Z轴旋转 (Rz)\n";
    std::cout << "  ↑/↓     : 绕Y轴旋转 (Ry)\n";
    std::cout << "  ←/→     : 绕X轴旋转 (Rx)\n";
    std::cout << "设置键：\n";
    std::cout << "  1-9     : 设置步进值 (1-9 mm/度)\n";
    std::cout << "  0       : 设置步进值为 10 mm/度\n";
    std::cout << "  r       : 重置到零位\n";
    std::cout << "  ESC     : 退出程序\n";
    std::cout << "=============================\n\n";
}

void KeyboardTCPController::run() {
    while (running_ && rclcpp::ok()) {
        // 处理键盘输入
        char key = get_key();
        if (key != 0) {
            process_key(key);
        }

        // 处理OpenCV事件
        handle_opencv_events();

        // 更新显示
        update_display();

        // 处理ROS事件
        rclcpp::spin_some(this->shared_from_this());

        // 短暂休眠
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
