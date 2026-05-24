#include "libfairino/robot.h"
#include "robo_ctrl/robo_ctrl_node.hpp"
#include "libfairino/robot_error.h"
#include <thread>
#include <chrono>
#include <Eigen/Dense>
namespace robo_ctrl {

// 修改构造函数，启动状态读取线程
RoboCtrlNode::RoboCtrlNode(const rclcpp::NodeOptions& options):
    Node("robo_ctrl_node", options),
    is_connected_(false),
    thread_running_(false) {
    // 从参数获取机器人连接信息
    this->declare_parameter<std::string>("robot_ip", "192.168.58.2");
    this->declare_parameter<int>("robot_port", 8080);
    this->declare_parameter<double>("state_query_interval", 0.01); // 状态查询间隔，默认10ms
    this->declare_parameter<std::string>("robot_name", "FRRobot");

    robot_ip_             = this->get_parameter("robot_ip").as_string();
    robot_port_           = this->get_parameter("robot_port").as_int();
    state_query_interval_ = this->get_parameter("state_query_interval").as_double();
    robot_name_           = this->get_parameter("robot_name").as_string();

    // 初始化机器人连接
    if (!init_robot_connection()) {
        RCLCPP_ERROR(this->get_logger(), "无法连接到机器人，IP: %s, 端口: %d", robot_ip_.c_str(), robot_port_);
    }

    // 创建服务
    robot_move_service_ = this->create_service<robo_ctrl::srv::RobotMove>(
        robot_name_ + "/robot_move",
        std::bind(&RoboCtrlNode::handle_robot_move, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建MoveCart服务
    robot_move_cart_service_ = this->create_service<robo_ctrl::srv::RobotMoveCart>(
        robot_name_ + "/robot_move_cart",
        std::bind(&RoboCtrlNode::handle_robot_move_cart, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建Servo服务
    robot_servo_service_ = this->create_service<robo_ctrl::srv::RobotServo>(
        robot_name_ + "/robot_servo",
        std::bind(&RoboCtrlNode::handle_robot_servo, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建设置速度服务
    robot_set_speed_service_ = this->create_service<robo_ctrl::srv::RobotSetSpeed>(
        robot_name_ + "/robot_set_speed",
        std::bind(&RoboCtrlNode::handle_robot_set_speed, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建ServoLine服务
    robot_servo_line_service_ = this->create_service<robo_ctrl::srv::RobotServoLine>(
        robot_name_ + "/robot_servo_line",
        std::bind(&RoboCtrlNode::handle_robot_servo_line, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建ServoJoint服务
    robot_servo_joint_service_ = this->create_service<robo_ctrl::srv::RobotServoJoint>(
        robot_name_ + "/robot_servo_joint",
        std::bind(&RoboCtrlNode::handle_robot_servo_joint, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建机器人状态发布器
    robot_state_pub_ = this->create_publisher<robo_ctrl::msg::RobotState>(robot_name_ + "/robot_state", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(robot_name_ + "/joint_states", 10);

    // 初始化TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 如果连接成功，启动状态读取线程
    if (is_connected_) {
        // 启动状态读取线程
        thread_running_ = true;
        state_thread_   = std::thread(&RoboCtrlNode::robot_state_thread_func, this);
        RCLCPP_INFO(this->get_logger(), "机器人状态监控线程已启动，查询间隔: %.2f秒", state_query_interval_);
    }

    RCLCPP_INFO(this->get_logger(), "机器人控制节点已启动");
}

RoboCtrlNode::~RoboCtrlNode() {
    // 先停止状态读取线程
    if (thread_running_) {
        RCLCPP_INFO(this->get_logger(), "正在停止机器人状态监控线程...");
        thread_running_ = false;
        if (state_thread_.joinable()) {
            state_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "机器人状态监控线程已停止");
    }

    // 断开机器人连接 - 使用CloseRPC代替DisConnect
    if (robot_ && is_connected_) {
        robot_->CloseRPC();
        RCLCPP_INFO(this->get_logger(), "已断开机器人连接");
    }
}

bool RoboCtrlNode::init_robot_connection() {
    try {
        // 创建机器人对象
        robot_ = std::make_unique<FRRobot>();

        // 连接到机器人 - RPC方法只接受IP地址参数
        RCLCPP_INFO(this->get_logger(), "正在连接机器人，IP: %s", robot_ip_.c_str());

        int ret = robot_->RPC(robot_ip_.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "连接机器人失败，错误码: %d", ret);
            return false;
        }

        is_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "成功连接到机器人");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "连接机器人时发生异常: %s", e.what());
        return false;
    }
}

void RoboCtrlNode::handle_robot_move(
    const std::shared_ptr<robo_ctrl::srv::RobotMove::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotMove::Response> response
) {
    if (!robot_ || !is_connected_) {
        response->success = false;
        response->message = "机器人未连接";
        return;
    }

    try {
        int ret = 0;

        // 根据移动类型选择相应的移动命令
        if (request->move_type == 0) {
            // 关节移动
            if (request->joint_positions.size() < 6) {
                response->success = false;
                response->message = "关节位置数组长度不足";
                return;
            }

            // 创建关节位置结构体
            JointPos joint_pos;
            for (size_t i = 0; i < 6; i++) {
                joint_pos.jPos[i] = static_cast<float>(request->joint_positions[i]);
            }

            float blendT       = -1.0f; // 平滑时间，-1表示阻塞运动
            uint8_t offsetflag = 0;     // 偏移标志，0表示无偏移
            // 不需要的设为memset(0)
            DescPose offset_pos;
            memset(&offset_pos, 0, sizeof(DescPose));
            ExaxisPos exaxis_pos;
            memset(&exaxis_pos, 0, sizeof(ExaxisPos));

            // 调用MoveJ函数，根据头文件定义提供所有必需的参数
            ret = robot_->MoveJ(
                &joint_pos,                                // 关节位置
                &offset_pos,                               // 笛卡尔位置
                0,                                         // 工具号
                0,                                         // 工件号
                static_cast<float>(request->velocity),     // 速度
                static_cast<float>(request->acceleration), // 加速度
                100.0f,                                    // 速度缩放因子，设为100%
                &exaxis_pos,                               // 外部轴位置，不需要，memset 0
                blendT,                                    // 平滑时间，-1表示阻塞运动
                offsetflag,                                // 偏移标志，0表示无偏移
                &offset_pos                                // 偏移位置，不需要，memset 0
            );
        } else if (request->move_type == 1) {
            // 笛卡尔移动
            if (request->cartesian_pose.size() < 6) {
                response->success = false;
                response->message = "笛卡尔位姿数组长度不足";
                return;
            }

            // 创建笛卡尔位置结构体
            DescPose desc_pos;
            desc_pos.tran.x = static_cast<float>(request->cartesian_pose[0]);
            desc_pos.tran.y = static_cast<float>(request->cartesian_pose[1]);
            desc_pos.tran.z = static_cast<float>(request->cartesian_pose[2]);
            desc_pos.rpy.rx = static_cast<float>(request->cartesian_pose[3]);
            desc_pos.rpy.ry = static_cast<float>(request->cartesian_pose[4]);
            desc_pos.rpy.rz = static_cast<float>(request->cartesian_pose[5]);

            // 调用MoveL函数，根据头文件定义提供所有必需的参数
            ret = robot_->MoveL(
                nullptr,                                   // 关节位置，不需要，设为nullptr
                &desc_pos,                                 // 笛卡尔位置
                0,                                         // 工具号
                0,                                         // 工件号
                static_cast<float>(request->velocity),     // 速度
                static_cast<float>(request->acceleration), // 加速度
                100.0f,                                    // 速度缩放因子，设为100%
                -1.0f,                                     // 平滑半径，-1表示阻塞运动
                nullptr,                                   // 外部轴位置，不需要，设为nullptr
                0,                                         // 寻线标志，0表示不寻线
                0,                                         // 偏移标志，0表示无偏移
                nullptr                                    // 偏移位置，不需要，设为nullptr
            );
        } else {
            response->success = false;
            response->message = "未知的移动类型";
            return;
        }

        if (ret != 0) {
            response->success = false;
            response->message = "执行移动命令失败，错误码: " + std::to_string(ret);
            return;
        }

        // 等待机器人运动完成
        uint8_t motion_done = 0;
        while (motion_done == 0) {
            ret = robot_->GetRobotMotionDone(&motion_done);
            if (ret != 0) {
                response->success = false;
                response->message = "获取机器人运动状态失败，错误码: " + std::to_string(ret);
                return;
            }

            // 短暂休眠，避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        response->success = true;
        response->message = "移动命令执行成功";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("执行移动命令时发生异常: ") + e.what();
    }
}

void RoboCtrlNode::handle_robot_move_cart(
    const std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Response> response
) {
    if (!robot_ || !is_connected_) {
        response->success = false;
        response->message = "机器人未连接";
        return;
    }

    try {
        // 保存目标位置用于TF发布和错误处理
        last_move_cart_target_ = request->tcp_pose;
        has_move_cart_target_  = true;

        // 创建笛卡尔位置结构体
        DescPose desc_pos;

        // 如果是增量运动，先获取当前TCP位置，然后加上增量
        if (request->use_increment) {
            // 获取当前TCP位姿
            DescPose current_tcp_pose;
            int get_ret = robot_->GetActualTCPPose(0, &current_tcp_pose);
            if (get_ret != 0) {
                response->success = false;
                response->message = "获取当前TCP位姿失败，无法执行增量运动，错误码: " + std::to_string(get_ret);
                return;
            }

            // 计算增量运动后的目标位置
            desc_pos.tran.x = current_tcp_pose.tran.x + static_cast<float>(request->tcp_pose.x);
            desc_pos.tran.y = current_tcp_pose.tran.y + static_cast<float>(request->tcp_pose.y);
            desc_pos.tran.z = current_tcp_pose.tran.z + static_cast<float>(request->tcp_pose.z);
            desc_pos.rpy.rx = current_tcp_pose.rpy.rx + static_cast<float>(request->tcp_pose.rx);
            desc_pos.rpy.ry = current_tcp_pose.rpy.ry + static_cast<float>(request->tcp_pose.ry);
            desc_pos.rpy.rz = current_tcp_pose.rpy.rz + static_cast<float>(request->tcp_pose.rz);

            // 修正：更新last_move_cart_target_为增量后的实际目标位置
            last_move_cart_target_.x  = static_cast<double>(desc_pos.tran.x);
            last_move_cart_target_.y  = static_cast<double>(desc_pos.tran.y);
            last_move_cart_target_.z  = static_cast<double>(desc_pos.tran.z);
            last_move_cart_target_.rx = static_cast<double>(desc_pos.rpy.rx);
            last_move_cart_target_.ry = static_cast<double>(desc_pos.rpy.ry);
            last_move_cart_target_.rz = static_cast<double>(desc_pos.rpy.rz);

            RCLCPP_INFO(
                this->get_logger(),
                "执行增量运动 - 当前位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], 增量: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], 目标: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                current_tcp_pose.tran.x,
                current_tcp_pose.tran.y,
                current_tcp_pose.tran.z,
                current_tcp_pose.rpy.rx,
                current_tcp_pose.rpy.ry,
                current_tcp_pose.rpy.rz,
                static_cast<float>(request->tcp_pose.x),
                static_cast<float>(request->tcp_pose.y),
                static_cast<float>(request->tcp_pose.z),
                static_cast<float>(request->tcp_pose.rx),
                static_cast<float>(request->tcp_pose.ry),
                static_cast<float>(request->tcp_pose.rz),
                desc_pos.tran.x,
                desc_pos.tran.y,
                desc_pos.tran.z,
                desc_pos.rpy.rx,
                desc_pos.rpy.ry,
                desc_pos.rpy.rz
            );
        } else {
            // 绝对运动，直接使用请求中的位置
            desc_pos.tran.x = static_cast<float>(request->tcp_pose.x);  // X坐标
            desc_pos.tran.y = static_cast<float>(request->tcp_pose.y);  // Y坐标
            desc_pos.tran.z = static_cast<float>(request->tcp_pose.z);  // Z坐标
            desc_pos.rpy.rx = static_cast<float>(request->tcp_pose.rx); // Rx旋转角度
            desc_pos.rpy.ry = static_cast<float>(request->tcp_pose.ry); // Ry旋转角度
            desc_pos.rpy.rz = static_cast<float>(request->tcp_pose.rz); // Rz旋转角度
        }

        // 条件限制
        if (desc_pos.tran.x < -1000.0f || desc_pos.tran.x > 1000.0f || desc_pos.tran.y < -1000.0f
            || desc_pos.tran.y > 1000.0f || desc_pos.tran.z < 0.0f || desc_pos.tran.z > 2000.0f) {
            response->success = false;
            response->message = "笛卡尔位置超出范围";
            return;
        }

        // 调用MoveCart函数，根据头文件定义提供所有必需的参数
        int ret = robot_->MoveCart(
            &desc_pos,                                 // 笛卡尔位置
            request->tool < 0 ? 0 : request->tool,     // 工具坐标系号
            request->user < 0 ? 0 : request->user,     // 工件坐标系号
            static_cast<float>(request->velocity),     // 速度百分比
            static_cast<float>(request->acceleration), // 加速度百分比
            static_cast<float>(request->ovl),          // 速度缩放因子
            static_cast<float>(request->blend_time),   // 平滑时间
            request->config                            // 关节空间构型
        );
        RCLCPP_INFO(this->get_logger(), "运动发送完成");

        if (ret != 0) {
            //     // 尝试处理错误
            //     if (handle_move_cart_error(ret)) {
            //         response->success = true;
            //         response->message = "MoveCart命令执行成功（自动消除错误后重试）";
            //         return;
            //     } else {
            response->success = false;
            response->message = "执行MoveCart命令失败，错误码: " + std::to_string(ret);
            return;
        }
        // }

        // 等待机器人运动完成，带超时机制
        uint8_t motion_done         = 0;
        auto start_time             = std::chrono::steady_clock::now();
        const auto timeout_duration = std::chrono::seconds(5); // 5秒超时

        while (motion_done == 0) {
            ret = robot_->GetRobotMotionDone(&motion_done);
            // std::cout << (motion_done == 0) << std::endl;
            if (ret != 0) {
                response->success = false;
                response->message = "获取机器人运动状态失败，错误码: " + std::to_string(ret);
                return;
            }

            // 检查超时
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time > timeout_duration) {
                RCLCPP_WARN(this->get_logger(), "等待机器人运动完成超时，默认返回成功");
                break; // 超时后跳出循环，默认认为运动完成
            }

            // 短暂休眠，避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        response->success = true;
        response->message = "MoveCart命令执行成功";
        RCLCPP_INFO(this->get_logger(), "运动完成");
        RCLCPP_DEBUG(this->get_logger(), "MoveCart命令执行成功");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("执行MoveCart命令时发生异常: ") + e.what();
    }
}

void RoboCtrlNode::handle_robot_servo(
    const std::shared_ptr<robo_ctrl::srv::RobotServo::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotServo::Response> response
) {
    if (!robot_ || !is_connected_) {
        response->success = false;
        response->message = "机器人未连接";
        return;
    }

    try {
        int ret = 0;

        // 根据命令类型执行相应的伺服控制
        switch (request->command_type) {
            case 0: // ServoMoveStart
                RCLCPP_INFO(this->get_logger(), "执行ServoMoveStart命令");
                ret = robot_->ServoMoveStart();
                if (ret != 0) {
                    response->success = false;
                    response->message = "执行ServoMoveStart命令失败，错误码: " + std::to_string(ret);
                    RCLCPP_ERROR(this->get_logger(), "执行ServoMoveStart命令失败，错误码: %d", ret);
                    return;
                }
                response->success = true;
                response->message = "ServoMoveStart命令执行成功";
                break;

            case 1: // ServoMoveEnd
                RCLCPP_INFO(this->get_logger(), "执行ServoMoveEnd命令");
                ret = robot_->ServoMoveEnd();
                if (ret != 0) {
                    response->success = false;
                    response->message = "执行ServoMoveEnd命令失败，错误码: " + std::to_string(ret);
                    RCLCPP_ERROR(this->get_logger(), "执行ServoMoveEnd命令失败，错误码: %d", ret);
                    return;
                }
                response->success = true;
                response->message = "ServoMoveEnd命令执行成功";
                break;

            case 2: // ServoJ
                if (request->joint_positions.size() < 6) {
                    response->success = false;
                    response->message = "关节位置数组长度不足，需要6个关节位置";
                    return;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "执行ServoJ命令，关节位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]，周期时间: %.4f秒",
                    request->joint_positions[0],
                    request->joint_positions[1],
                    request->joint_positions[2],
                    request->joint_positions[3],
                    request->joint_positions[4],
                    request->joint_positions[5],
                    request->cmd_time
                );

                // 创建关节位置结构体
                JointPos joint_pos;
                for (size_t i = 0; i < 6; i++) {
                    joint_pos.jPos[i] = static_cast<double>(request->joint_positions[i]);
                }

                // 执行ServoJ命令
                ret = robot_->ServoJ(
                    &joint_pos,                               // 关节位置
                    static_cast<float>(request->acc),         // 加速度百分比
                    static_cast<float>(request->vel),         // 速度百分比
                    static_cast<float>(request->cmd_time),    // 指令周期时间
                    static_cast<float>(request->filter_time), // 滤波时间
                    static_cast<float>(request->gain)         // 目标位置增益
                );

                if (ret != 0) {
                    response->success = false;
                    response->message = "执行ServoJ命令失败，错误码: " + std::to_string(ret);
                    RCLCPP_ERROR(this->get_logger(), "执行ServoJ命令失败，错误码: %d", ret);
                    return;
                }
                response->success = true;
                response->message = "ServoJ命令执行成功";
                break;

            case 3: // ServoCart
                if (request->cartesian_pose.size() < 6) {
                    response->success = false;
                    response->message = "笛卡尔位姿数组长度不足，需要6个参数 [x, y, z, rx, ry, rz]";
                    return;
                }

                if (request->pos_gain.size() < 6) {
                    response->success = false;
                    response->message = "位姿增量比例系数数组长度不足，需要6个参数对应 [x, y, z, rx, ry, rz]";
                    return;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "执行ServoCart命令，模式: %d，位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]，周期时间: %.4f秒",
                    request->mode,
                    request->cartesian_pose[0],
                    request->cartesian_pose[1],
                    request->cartesian_pose[2],
                    request->cartesian_pose[3],
                    request->cartesian_pose[4],
                    request->cartesian_pose[5],
                    request->cmd_time
                );

                // 创建笛卡尔位姿结构体
                DescPose desc_pose;
                desc_pose.tran.x = static_cast<double>(request->cartesian_pose[0]);
                desc_pose.tran.y = static_cast<double>(request->cartesian_pose[1]);
                desc_pose.tran.z = static_cast<double>(request->cartesian_pose[2]);
                desc_pose.rpy.rx = static_cast<double>(request->cartesian_pose[3]);
                desc_pose.rpy.ry = static_cast<double>(request->cartesian_pose[4]);
                desc_pose.rpy.rz = static_cast<double>(request->cartesian_pose[5]);

                // 创建位姿增量比例系数数组
                float pos_gain[6];
                for (size_t i = 0; i < 6; i++) {
                    pos_gain[i] = static_cast<float>(request->pos_gain[i]);
                }

                // 执行ServoCart命令
                ret = robot_->ServoCart(
                    request->mode,                            // 运动模式
                    &desc_pose,                               // 笛卡尔位姿
                    pos_gain,                                 // 位姿增量比例系数
                    static_cast<float>(request->acc),         // 加速度百分比
                    static_cast<float>(request->vel),         // 速度百分比
                    static_cast<float>(request->cmd_time),    // 指令周期时间
                    static_cast<float>(request->filter_time), // 滤波时间
                    static_cast<float>(request->gain)         // 目标位置增益
                );

                if (ret != 0) {
                    response->success = false;
                    response->message = "执行ServoCart命令失败，错误码: " + std::to_string(ret);
                    RCLCPP_ERROR(this->get_logger(), "执行ServoCart命令失败，错误码: %d", ret);
                    return;
                }
                response->success = true;
                response->message = "ServoCart命令执行成功";
                break;

            default:
                response->success = false;
                response->message = "未知的命令类型: " + std::to_string(request->command_type);
                RCLCPP_ERROR(this->get_logger(), "未知的命令类型: %d", request->command_type);
                break;
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("执行Servo命令时发生异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "执行Servo命令时发生异常: %s", e.what());
    }
}

void RoboCtrlNode::handle_robot_servo_line(
    const std::shared_ptr<robo_ctrl::srv::RobotServoLine::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotServoLine::Response> response
) {
    if (!request) {
        response->success = false;
        response->message = "请求信息无效";
        return;
    }
    if (!robot_ || !is_connected_) {
        response->success = false;
        response->message = "机器人未连接";
        return;
    }
    try {
        switch (request->command_type) {
            case 0: { // ServoLineStart
                if (is_servo_running_.load()) {
                    response->success = false;
                    response->message = "运动已在进行中";
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "开始执行ServoLine路径跟踪...");
                is_servo_running_ = true;
                if (servo_thread_.joinable()) {
                    servo_thread_.join();
                }
                // 启动运动线程
                servo_thread_ = std::thread([this, request]() {
                    int point_num     = request->cartesian_pose.poses.size();
                    float pos_gain[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
                    int mode          = request->use_incremental ? 1 : 0;
                    float acc         = request->acc;
                    float vel         = request->vel;
                    float cmdT        = request->cmd_time;
                    float filterT     = request->filter_time;
                    float gain        = request->gain;

                    for (int i = 0; i < point_num && is_servo_running_; ++i) {
                        const auto& pose = request->cartesian_pose.poses[i];
                        DescPose target_pose;
                        target_pose.tran.x = pose.position.x;
                        target_pose.tran.y = pose.position.y;
                        target_pose.tran.z = pose.position.z;
                        tf2::Quaternion
                            q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                        tf2::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        target_pose.rpy.rx = roll * 180.0 / M_PI;  // 转换为度
                        target_pose.rpy.ry = pitch * 180.0 / M_PI; // 转换为度
                        target_pose.rpy.rz = yaw * 180.0 / M_PI;   // 转换为度
                        RCLCPP_INFO(
                            this->get_logger(),
                            "ServoLine路径点[%d]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                            i,
                            target_pose.tran.x,
                            target_pose.tran.y,
                            target_pose.tran.z,
                            target_pose.rpy.rx,
                            target_pose.rpy.ry,
                            target_pose.rpy.rz
                        );
                        auto ret = robot_->ServoCart(
                            mode,         // 运动模式
                            &target_pose, // 笛卡尔位姿
                            pos_gain,     // 位姿增量比例系数
                            acc,          // 加速度百分比
                            vel,          // 速度百分比
                            cmdT,         // 指令周期时间
                            filterT,      // 滤波时间
                            gain          // 目标位置增益
                        );
                        if (ret != 0) {
                            RCLCPP_ERROR(this->get_logger(), "ServoLine路径跟踪失败，错误码: %d", ret);
                            is_servo_running_.store(false);
                            return;
                        }
                        robot_->WaitMs(cmdT * 1000);
                    }

                    is_servo_running_.store(false);
                    RCLCPP_INFO(this->get_logger(), "ServoLine路径执行完毕或被中断");
                });

                response->success = true;
                response->message = "ServoLine开始执行";
                break;
            }

            case 1: { // ServoLineEnd
                if (!is_servo_running_.load()) {
                    response->success = false;
                    response->message = "当前没有正在执行的ServoLine任务";
                    return;
                }

                is_servo_running_.store(false);

                if (servo_thread_.joinable()) {
                    servo_thread_.join();
                }

                response->success = true;
                response->message = "ServoLine任务已中断";
                RCLCPP_DEBUG(this->get_logger(), "收到ServoLineEnd命令，运动中断");
                break;
            }

            default:
                response->success = false;
                response->message = "未知的命令类型";
                return;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void RoboCtrlNode::handle_robot_servo_joint(
    const std::shared_ptr<robo_ctrl::srv::RobotServoJoint::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotServoJoint::Response> response
) {
    if (!request) {
        response->success = false;
        response->message = "请求信息无效";
        return;
    }
    if (!robot_ || !is_connected_) {
        response->success = false;
        response->message = "机器人未连接";
        return;
    }
    try {
        switch (request->command_type) {
            case 0: { // ServoJStart
                if (is_servo_running_.load()) {
                    response->success = false;
                    response->message = "运动已在进行中";
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "开始执行ServoJ路径跟踪...");
                is_servo_running_.store(true);
                std::lock_guard<std::mutex> lock(servo_thread_mutex_);
                if (servo_thread_.joinable()) {
                    servo_thread_.join();
                }
                // 提前生成所有关节角度序列
                int point_num = request->joint_positions.size();
                std::vector<JointPos> target_poses;
                target_poses.reserve(point_num);

                RCLCPP_INFO(this->get_logger(), "ServoJ路径点数量: %d", point_num);

                // 如果是增量模式，获取当前关节位置作为基准
                JointPos current_pose;
                if (request->use_incremental) {
                    int ret = robot_->GetActualJointPosDegree(0, &current_pose);
                    if (ret != 0) {
                        response->success = false;
                        response->message = "获取当前关节位置失败，错误码: " + std::to_string(ret);
                        is_servo_running_.store(false);
                        return;
                    }
                    RCLCPP_INFO(
                        this->get_logger(),
                        "当前关节位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        current_pose.jPos[0],
                        current_pose.jPos[1],
                        current_pose.jPos[2],
                        current_pose.jPos[3],
                        current_pose.jPos[4],
                        current_pose.jPos[5]
                    );
                }

                // 预计算所有目标位置
                for (int i = 0; i < point_num; ++i) {
                    const auto& pose = request->joint_positions[i];
                    JointPos target_pose;
                    memset(&target_pose, 0, sizeof(JointPos));

                    if (!request->use_incremental) {
                        // 绝对位置模式
                        target_pose.jPos[0] = static_cast<float>(pose.position.at(0));
                        target_pose.jPos[1] = static_cast<float>(pose.position.at(1));
                        target_pose.jPos[2] = static_cast<float>(pose.position.at(2));
                        target_pose.jPos[3] = static_cast<float>(pose.position.at(3));
                        target_pose.jPos[4] = static_cast<float>(pose.position.at(4));
                        target_pose.jPos[5] = static_cast<float>(pose.position.at(5));
                    } else {
                        // 增量模式：基于第一个点的位置计算累积增量
                        if (i == 0) {
                            // 第一个点：当前位置 + 第一个增量
                            target_pose.jPos[0] = current_pose.jPos[0] + static_cast<float>(pose.position.at(0));
                            target_pose.jPos[1] = current_pose.jPos[1] + static_cast<float>(pose.position.at(1));
                            target_pose.jPos[2] = current_pose.jPos[2] + static_cast<float>(pose.position.at(2));
                            target_pose.jPos[3] = current_pose.jPos[3] + static_cast<float>(pose.position.at(3));
                            target_pose.jPos[4] = current_pose.jPos[4] + static_cast<float>(pose.position.at(4));
                            target_pose.jPos[5] = current_pose.jPos[5] + static_cast<float>(pose.position.at(5));
                        } else {
                            // 后续点：基于前一个目标位置累加增量
                            const auto& prev_pose = target_poses[i - 1];
                            target_pose.jPos[0]   = prev_pose.jPos[0] + static_cast<float>(pose.position.at(0));
                            target_pose.jPos[1]   = prev_pose.jPos[1] + static_cast<float>(pose.position.at(1));
                            target_pose.jPos[2]   = prev_pose.jPos[2] + static_cast<float>(pose.position.at(2));
                            target_pose.jPos[3]   = prev_pose.jPos[3] + static_cast<float>(pose.position.at(3));
                            target_pose.jPos[4]   = prev_pose.jPos[4] + static_cast<float>(pose.position.at(4));
                            target_pose.jPos[5]   = prev_pose.jPos[5] + static_cast<float>(pose.position.at(5));
                        }
                    }

                    target_poses.push_back(target_pose);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "预计算ServoJ路径点[%d]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        i,
                        target_pose.jPos[0],
                        target_pose.jPos[1],
                        target_pose.jPos[2],
                        target_pose.jPos[3],
                        target_pose.jPos[4],
                        target_pose.jPos[5]
                    );
                }

                // 启动运动线程，只负责执行预计算的序列
                servo_thread_ = std::thread([this, target_poses, request]() {
                    auto cleanup = [&]() {
                        is_servo_running_.store(false);
                        RCLCPP_DEBUG(this->get_logger(), "ServoJ路径执行完毕或被中断");
                    };

                    float acc     = request->acc;
                    float vel     = request->vel;
                    float cmdT    = request->cmd_time;
                    float filterT = request->filter_time;
                    float gain    = request->gain;

                    // 执行预计算的关节角度序列
                    for (size_t i = 0; i < target_poses.size() && is_servo_running_; ++i) {
                        const auto& target_pose = target_poses[i];

                        RCLCPP_DEBUG(
                            this->get_logger(),
                            "执行ServoJ路径点[%zu]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                            i,
                            target_pose.jPos[0],
                            target_pose.jPos[1],
                            target_pose.jPos[2],
                            target_pose.jPos[3],
                            target_pose.jPos[4],
                            target_pose.jPos[5]
                        );

                        auto ret = robot_->ServoJ(const_cast<JointPos*>(&target_pose), acc, vel, cmdT, filterT, gain);
                        if (ret != 0) {
                            RCLCPP_ERROR(this->get_logger(), "ServoJ路径跟踪失败，错误码: %d", ret);
                            is_servo_running_.store(false);
                            return;
                        }
                        robot_->WaitMs(cmdT * 1100);
                    }

                    is_servo_running_.store(false);
                    RCLCPP_DEBUG(this->get_logger(), "ServoJ路径执行完毕或被中断");
                });

                response->success = true;
                response->message = "ServoJ开始执行";
                break;
            }

            case 1: { // ServoJEnd
                if (!is_servo_running_.load()) {
                    response->success = false;
                    response->message = "当前没有正在执行的ServoJ任务";
                    return;
                }

                is_servo_running_.store(false);

                if (servo_thread_.joinable()) {
                    servo_thread_.join();
                }

                response->success = true;
                response->message = "ServoJ任务已中断";
                RCLCPP_DEBUG(this->get_logger(), "收到ServoJEnd命令，运动中断");
                break;
            }

            default:
                response->success = false;
                response->message = "未知的命令类型";
                return;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void RoboCtrlNode::robot_state_timer_callback() {
    if (!robot_ || !is_connected_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "机器人未连接，无法获取状态");
        return;
    }

    try {
        // 创建机器人状态消息
        auto robot_state_msg = std::make_unique<robo_ctrl::msg::RobotState>();
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();

        bool has_data = false;

        // 1. 获取关节位置
        JointPos joint_pos;
        int ret = robot_->GetActualJointPosDegree(0, &joint_pos); // 0表示阻塞调用
        if (ret == 0) {
            // 直接赋值到单个对象，而不是数组
            robot_state_msg->joint_position.j1 = static_cast<double>(joint_pos.jPos[0]);
            robot_state_msg->joint_position.j2 = static_cast<double>(joint_pos.jPos[1]);
            robot_state_msg->joint_position.j3 = static_cast<double>(joint_pos.jPos[2]);
            robot_state_msg->joint_position.j4 = static_cast<double>(joint_pos.jPos[3]);
            robot_state_msg->joint_position.j5 = static_cast<double>(joint_pos.jPos[4]);
            robot_state_msg->joint_position.j6 = static_cast<double>(joint_pos.jPos[5]);
            has_data                           = true;
            joint_state_msg->header.stamp      = this->now();
            joint_state_msg->position.resize(6);
            joint_state_msg->position[0] = static_cast<double>(joint_pos.jPos[0]) * M_PI / 180.0;
            joint_state_msg->position[1] = static_cast<double>(joint_pos.jPos[1]) * M_PI / 180.0;
            joint_state_msg->position[2] = static_cast<double>(joint_pos.jPos[2]) * M_PI / 180.0;
            joint_state_msg->position[3] = static_cast<double>(joint_pos.jPos[3]) * M_PI / 180.0;
            joint_state_msg->position[4] = static_cast<double>(joint_pos.jPos[4]) * M_PI / 180.0;
            joint_state_msg->position[5] = static_cast<double>(joint_pos.jPos[5]) * M_PI / 180.0;
            joint_state_msg->name.resize(6);
            joint_state_msg->name[0] = "j1";
            joint_state_msg->name[1] = "j2";
            joint_state_msg->name[2] = "j3";
            joint_state_msg->name[3] = "j4";
            joint_state_msg->name[4] = "j5";
            joint_state_msg->name[5] = "j6";
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "获取关节位置失败，错误码: %d", ret);
        }

        // 2. 获取TCP位姿
        DescPose tcp_pose;
        ret = robot_->GetActualTCPPose(1, &tcp_pose); // 0表示阻塞调用
        std::cout << "tcp_pose: " << tcp_pose.tran.x << ", " << tcp_pose.tran.y << ", " << tcp_pose.tran.z << std::endl;
        std::cout << "tcp_pose: " << tcp_pose.rpy.rx << ", " << tcp_pose.rpy.ry << ", " << tcp_pose.rpy.rz << std::endl;

        if (ret == 0) {
            robot_state_msg->tcp_pose.x  = static_cast<double>(tcp_pose.tran.x);
            robot_state_msg->tcp_pose.y  = static_cast<double>(tcp_pose.tran.y);
            robot_state_msg->tcp_pose.z  = static_cast<double>(tcp_pose.tran.z);
            robot_state_msg->tcp_pose.rx = static_cast<double>(tcp_pose.rpy.rx);
            robot_state_msg->tcp_pose.ry = static_cast<double>(tcp_pose.rpy.ry);
            robot_state_msg->tcp_pose.rz = static_cast<double>(tcp_pose.rpy.rz);
            has_data                     = true;

            // 发布基坐标和TCP位置TF
            publish_tf_transforms(robot_state_msg->tcp_pose);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "获取TCP位姿失败，错误码: %d", ret);
        }

        // 3. 获取机器人运动完成状态
        uint8_t motion_done = 0;
        ret                 = robot_->GetRobotMotionDone(&motion_done);
        if (ret == 0) {
            robot_state_msg->motion_done = (motion_done != 0);
            has_data                     = true;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "获取运动状态失败，错误码: %d", ret);
        }

        // 4. 获取机器人错误代码
        int main_code = 0, sub_code = 0;
        ret = robot_->GetRobotErrorCode(&main_code, &sub_code);
        if (ret == 0) {
            robot_state_msg->error_code = main_code;
            has_data                    = true;

            if (main_code != 0) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "机器人报告错误，主错误码: %d，子错误码: %d",
                    main_code,
                    sub_code
                );
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "获取错误代码失败，错误码: %d", ret);
        }
        // 发布机器人状态
        if (has_data) {
            rclcpp::Time now = this->now();
            std::cout << now.nanoseconds() << std::endl;
            robot_state_msg->header.stamp = now;
            robot_state_pub_->publish(std::move(robot_state_msg));
            joint_state_pub_->publish(std::move(joint_state_msg));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "获取机器人状态时发生异常: %s", e.what());
    }
}

// 发布TF变换
void RoboCtrlNode::publish_tf_transforms(const robo_ctrl::msg::TCPPose& tcp_pose) {
    rclcpp::Time now = this->now();

    // 发布世界坐标系到机器人基坐标系的变换 (0,0,0)
    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp            = now;
    base_transform.header.frame_id         = "world";
    base_transform.child_frame_id          = this->robot_name_ + "robot_base";
    if (robot_name_ == "R") {
        base_transform.transform.translation.x = 0.067;
        base_transform.transform.translation.y = -0.799;
        base_transform.transform.translation.z = 0.0;
    } else {
    base_transform.transform.translation.x = 0.0;
    base_transform.transform.translation.y = 0.0;
    base_transform.transform.translation.z = 0.0;
    }
    tf2::Quaternion q_base;
    q_base.setRPY(0, 0, robot_name_=="R"? 128.0 * M_PI / 180.0 : 0.0);
    base_transform.transform.rotation.x = q_base.x();
    base_transform.transform.rotation.y = q_base.y();
    base_transform.transform.rotation.z = q_base.z();
    base_transform.transform.rotation.w = q_base.w();

    // 发布基坐标系到TCP的变换
    geometry_msgs::msg::TransformStamped tcp_transform;
    tcp_transform.header.stamp    = now;
    tcp_transform.header.frame_id = this->robot_name_ + "robot_base";
    tcp_transform.child_frame_id  = this->robot_name_ + "tcp";
    // 转换毫米到米
    tcp_transform.transform.translation.x = tcp_pose.x / 1000.0;
    tcp_transform.transform.translation.y = tcp_pose.y / 1000.0;
    tcp_transform.transform.translation.z = tcp_pose.z / 1000.0;
    tf2::Quaternion q_tcp;
    // RCLCPP_INFO(this->get_logger(), "TCP Pose: rx=%.2f, ry=%.2f, rz=%.2f",
    // tcp_pose.rx, tcp_pose.ry, tcp_pose.rz
    // );
    // 使用setRPY设置欧拉角，注意顺序是roll, pitch, yaw
    // 如果使用setEuler会导致顺序不正确，可能会出现奇异性问题
    // 这里的顺序是：roll (rx), pitch (ry), yaw (rz)
    // 注意：setRPY的参数顺序是 roll, pitch, yaw
    // 如果需要使用setEuler，请确保顺序正确
    // 例如：setEuler(yaw, pitch, roll) 或 setRPY(roll, pitch, yaw)
    //    q_tcp.setEuler(
    // q_tcp.setEuler(
    //     tcp_pose.rz * M_PI / 180.0, // Z轴旋转 (rz)
    //     tcp_pose.ry * M_PI / 180.0, // Y轴旋转 (ry)
    //     tcp_pose.rx * M_PI / 180.0  // X轴旋转 (rx)
    // );
    // 转换度到弧度
    q_tcp.setRPY(tcp_pose.rx * M_PI / 180.0, tcp_pose.ry * M_PI / 180.0, tcp_pose.rz * M_PI / 180.0);
    tcp_transform.transform.rotation.x = q_tcp.x();
    tcp_transform.transform.rotation.y = q_tcp.y();
    tcp_transform.transform.rotation.z = q_tcp.z();
    tcp_transform.transform.rotation.w = q_tcp.w();

    // 发布变换
    tf_broadcaster_->sendTransform(base_transform);
    tf_broadcaster_->sendTransform(tcp_transform);

    // 如果有MoveCart目标位置，也发布它
    if (has_move_cart_target_) {
        publish_move_cart_target_tf(last_move_cart_target_);
    }
}

// 发布MoveCart目标位置TF
void RoboCtrlNode::publish_move_cart_target_tf(const robo_ctrl::msg::TCPPose& target_pose) {
    rclcpp::Time now = this->now();

    // 发布目标位置TF
    geometry_msgs::msg::TransformStamped target_transform;
    target_transform.header.stamp    = now;
    target_transform.header.frame_id = this->robot_name_ + "robot_base";
    target_transform.child_frame_id  = this->robot_name_ + "target_pose";
    // 转换毫米到米
    target_transform.transform.translation.x = target_pose.x / 1000.0;
    target_transform.transform.translation.y = target_pose.y / 1000.0;
    target_transform.transform.translation.z = target_pose.z / 1000.0;
    tf2::Quaternion q_target;
    // 转换度到弧度
    q_target.setRPY(target_pose.rx * M_PI / 180.0, target_pose.ry * M_PI / 180.0, target_pose.rz * M_PI / 180.0);
    target_transform.transform.rotation.x = q_target.x();
    target_transform.transform.rotation.y = q_target.y();
    target_transform.transform.rotation.z = q_target.z();
    target_transform.transform.rotation.w = q_target.w();

    // 发布变换
    tf_broadcaster_->sendTransform(target_transform);
}

// 处理MoveCart错误并返回原位置
bool RoboCtrlNode::handle_move_cart_error(int error_code) {
    int ret;
    // 错误特殊处理
    switch (error_code) {
        case ERR_MOVEC_MIDDLE_POINT:
        case ERR_MOVEC_TARGET_POINT:
        case ERR_STRANGE_POSE:
        case ERR_TARGET_POSE_CANNOT_REACHED:
            // 重置机器人错误
            RCLCPP_INFO(this->get_logger(), "检测到MoveCart错误，错误码: %d，尝试重置错误并返回原位置", error_code);
            ret = robot_->ResetAllError();
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "消除错误失败，错误码: %d", ret);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "成功消除错误，尝试返回原位置");

            // 短暂延时
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 使用已保存的上一次成功的TCP位置
            DescPose last_tcp_pose;
            if (has_move_cart_target_) {
                RCLCPP_INFO(this->get_logger(), "使用记录的上一次MoveCart目标位置");
                // 使用自己记录的上一次MoveCart位置
                last_tcp_pose.tran.x = static_cast<float>(last_move_cart_target_.x);
                last_tcp_pose.tran.y = static_cast<float>(last_move_cart_target_.y);
                last_tcp_pose.tran.z = static_cast<float>(last_move_cart_target_.z);
                last_tcp_pose.rpy.rx = static_cast<float>(last_move_cart_target_.rx);
                last_tcp_pose.rpy.ry = static_cast<float>(last_move_cart_target_.ry);
                last_tcp_pose.rpy.rz = static_cast<float>(last_move_cart_target_.rz);
            } else {
                // 如果没有记录的位置，获取当前TCP位置
                RCLCPP_INFO(this->get_logger(), "没有记录的MoveCart位置，使用当前TCP位置");
                ret = robot_->GetActualTCPPose(0, &last_tcp_pose);
                if (ret != 0) {
                    RCLCPP_ERROR(this->get_logger(), "获取当前TCP位姿失败，错误码: %d", ret);
                    return false;
                }
            }

            // 执行MoveCart移动到上一次位置的命令
            RCLCPP_INFO(
                this->get_logger(),
                "尝试使用MoveCart返回到原位置 X:%.2f Y:%.2f Z:%.2f",
                last_tcp_pose.tran.x,
                last_tcp_pose.tran.y,
                last_tcp_pose.tran.z
            );

            // 使用MoveCart命令返回原位置，使用安全参数
            ret = robot_->MoveCart(
                &last_tcp_pose, // 上一次的TCP位置
                0,              // 默认工具号
                0,              // 默认工件号
                20.0f,          // 低速度 (20%)
                20.0f,          // 低加速度 (20%)
                100.0f,         // 正常速度缩放因子
                -1.0f,          // 阻塞运动
                -1              // 使用当前关节构型
            );

            if (ret == 0) {
                // 等待机器人运动完成
                uint8_t motion_done = 0;
                while (motion_done == 0) {
                    ret = robot_->GetRobotMotionDone(&motion_done);
                    if (ret != 0) {
                        RCLCPP_ERROR(this->get_logger(), "获取机器人运动状态失败，错误码: %d", ret);
                        return false;
                    }
                    // 短暂休眠，避免CPU占用过高
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                RCLCPP_INFO(this->get_logger(), "成功使用MoveCart返回到原位置");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "使用MoveCart返回原位置失败，错误码: %d", ret);

                // 如果MoveCart失败，尝试使用MoveL作为备选方案
                // RCLCPP_INFO(this->get_logger(), "尝试使用MoveL作为备选方案返回原位置");
                // ret = robot_->MoveL(
                //     nullptr,        // 关节位置，不需要
                //     &last_tcp_pose, // 上一次的TCP位置
                //     0,              // 默认工具号
                //     0,              // 默认工件号
                //     20.0f,          // 低速度 (20%)
                //     20.0f,          // 低加速度 (20%)
                //     100.0f,         // 正常速度缩放因子
                //     -1.0f,          // 阻塞运动
                //     nullptr,        // 外部轴位置，不需要
                //     0,              // 不寻线
                //     0,              // 不偏移
                //     nullptr         // 偏移位置，不需要
                // );

                // if (ret == 0) {
                //     RCLCPP_INFO(this->get_logger(), "成功使用MoveL返回到原位置");
                //     return true;
                // } else {
                //     RCLCPP_ERROR(this->get_logger(), "返回原位置失败，错误码: %d", ret);
                // }
            }
            break;

        default:
            RCLCPP_INFO(this->get_logger(), "未处理的MoveCart错误，错误码: %d", error_code);
            break;
    }

    return false;
}

// 机器人状态线程函数 - 在独立线程中运行
void RoboCtrlNode::robot_state_thread_func() {
    // 创建一个线程本地的时钟对象，而不是使用节点的共享时钟
    rclcpp::Clock thread_clock(RCL_SYSTEM_TIME);

    // 上次警告日志的时间，用于模拟throttle功能
    auto last_warn_time  = thread_clock.now();
    auto last_error_time = thread_clock.now();

    // 线程启动日志 - 使用节点的主线程发送
    RCLCPP_INFO(this->get_logger(), "机器人状态监控线程已启动");

    // 线程循环，当thread_running_为false时退出
    while (thread_running_) {
        if (!robot_ || !is_connected_) {
            // 模拟throttle功能，每5秒打印一次警告
            auto current_time = thread_clock.now();
            if ((current_time - last_warn_time).seconds() > 5.0) {
                // 使用互斥锁保护logger访问
                {
                    std::lock_guard<std::mutex> lock(robot_mutex_);
                    RCLCPP_WARN(this->get_logger(), "机器人未连接，无法获取状态");
                }
                last_warn_time = current_time;
            }

            // 如果未连接，稍微等待一段时间后再尝试
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(state_query_interval_ * 1000)));
            continue;
        }

        try {
            // 创建机器人状态消息
            auto robot_state_msg = std::make_unique<robo_ctrl::msg::RobotState>();
            auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
            bool has_data        = false;
            robo_ctrl::msg::TCPPose tcp_pose_msg;
            bool has_valid_pose = false;

            {
                // 使用互斥锁保护对机器人对象的访问
                std::lock_guard<std::mutex> lock(robot_mutex_);

                // 1. 获取关节位置
                JointPos joint_pos;
                joint_state_msg->header.stamp = this->now();
                joint_state_msg->name.resize(6);
                joint_state_msg->name[0] = "j1";
                joint_state_msg->name[1] = "j2";
                joint_state_msg->name[2] = "j3";
                joint_state_msg->name[3] = "j4";
                joint_state_msg->name[4] = "j5";
                joint_state_msg->name[5] = "j6";

                int ret = robot_->GetActualJointPosDegree(0, &joint_pos); // 0表示阻塞调用
                if (ret == 0) {
                    // 直接赋值到单个对象，而不是数组
                    robot_state_msg->joint_position.j1 = static_cast<double>(joint_pos.jPos[0]);
                    robot_state_msg->joint_position.j2 = static_cast<double>(joint_pos.jPos[1]);
                    robot_state_msg->joint_position.j3 = static_cast<double>(joint_pos.jPos[2]);
                    robot_state_msg->joint_position.j4 = static_cast<double>(joint_pos.jPos[3]);
                    robot_state_msg->joint_position.j5 = static_cast<double>(joint_pos.jPos[4]);
                    robot_state_msg->joint_position.j6 = static_cast<double>(joint_pos.jPos[5]);
                    joint_state_msg->position.resize(6);
                    joint_state_msg->position[0] = static_cast<double>(joint_pos.jPos[0]) * M_PI / 180.0;
                    joint_state_msg->position[1] = static_cast<double>(joint_pos.jPos[1]) * M_PI / 180.0;
                    joint_state_msg->position[2] = static_cast<double>(joint_pos.jPos[2]) * M_PI / 180.0;
                    joint_state_msg->position[3] = static_cast<double>(joint_pos.jPos[3]) * M_PI / 180.0;
                    joint_state_msg->position[4] = static_cast<double>(joint_pos.jPos[4]) * M_PI / 180.0;
                    joint_state_msg->position[5] = static_cast<double>(joint_pos.jPos[5]) * M_PI / 180.0;
                    has_data                     = true;
                } else {
                    auto current_time = thread_clock.now();
                    if ((current_time - last_warn_time).seconds() > 5.0) {
                        RCLCPP_WARN(this->get_logger(), "获取关节位置失败，错误码: %d", ret);
                        last_warn_time = current_time;
                    }

                    // 检测错误码-2，这通常表示通信断开或者超时
                    if (ret == -2) {
                        RCLCPP_ERROR(this->get_logger(), "检测到通信断开(错误码: -2)，尝试重新连接机器人...");

                        // 先关闭当前连接
                        if (is_connected_) {
                            robot_->CloseRPC();
                            is_connected_ = false;
                            RCLCPP_INFO(this->get_logger(), "已断开当前连接");
                        }

                        // 稍微延时，给机器人一些恢复时间
                        std::this_thread::sleep_for(std::chrono::seconds(1));

                        // 尝试重新连接
                        try {
                            int connect_ret = robot_->RPC(robot_ip_.c_str());
                            if (connect_ret == 0) {
                                is_connected_ = true;
                                RCLCPP_INFO(this->get_logger(), "成功重新连接到机器人");
                            } else {
                                RCLCPP_ERROR(this->get_logger(), "重新连接机器人失败，错误码: %d", connect_ret);
                            }
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(this->get_logger(), "重新连接机器人时发生异常: %s", e.what());
                        }

                        // 无论重连是否成功，都跳过本次循环的其余部分
                        continue;
                    }
                }

                // 2. 获取TCP位姿
                DescPose tcp_pose;
                ret = robot_->GetActualTCPPose(0, &tcp_pose); // 0表示阻塞调用
                if (ret == 0) {
                    robot_state_msg->tcp_pose.x  = static_cast<double>(tcp_pose.tran.x);
                    robot_state_msg->tcp_pose.y  = static_cast<double>(tcp_pose.tran.y);
                    robot_state_msg->tcp_pose.z  = static_cast<double>(tcp_pose.tran.z);
                    robot_state_msg->tcp_pose.rx = static_cast<double>(tcp_pose.rpy.rx);
                    robot_state_msg->tcp_pose.ry = static_cast<double>(tcp_pose.rpy.ry);
                    robot_state_msg->tcp_pose.rz = static_cast<double>(tcp_pose.rpy.rz);
                    has_data                     = true;
                    has_valid_pose               = true;

                    // 保存TCP位置用于后续发布
                    tcp_pose_msg = robot_state_msg->tcp_pose;
                } else {
                    auto current_time = thread_clock.now();
                    if ((current_time - last_warn_time).seconds() > 5.0) {
                        RCLCPP_WARN(this->get_logger(), "获取TCP位姿失败，错误码: %d", ret);
                        last_warn_time = current_time;
                    }
                }

                // 3. 获取机器人运动完成状态
                uint8_t motion_done = 0;
                ret                 = robot_->GetRobotMotionDone(&motion_done);
                if (ret == 0) {
                    robot_state_msg->motion_done = (motion_done != 0);
                    has_data                     = true;
                } else {
                    auto current_time = thread_clock.now();
                    if ((current_time - last_warn_time).seconds() > 5.0) {
                        RCLCPP_WARN(this->get_logger(), "获取运动状态失败，错误码: %d", ret);
                        last_warn_time = current_time;
                    }
                }

                // 4. 获取机器人错误代码
                int main_code = 0, sub_code = 0;
                ret = robot_->GetRobotErrorCode(&main_code, &sub_code);
                if (ret == 0) {
                    robot_state_msg->error_code = main_code;
                    has_data                    = true;

                    if (main_code != 0) {
                        auto current_time = thread_clock.now();
                        if ((current_time - last_warn_time).seconds() > 5.0) {
                            RCLCPP_WARN(
                                this->get_logger(),
                                "机器人报告错误，主错误码: %d，子错误码: %d",
                                main_code,
                                sub_code
                            );
                            last_warn_time = current_time;
                        }
                    }
                } else {
                    auto current_time = thread_clock.now();
                    if ((current_time - last_warn_time).seconds() > 5.0) {
                        RCLCPP_WARN(this->get_logger(), "获取错误代码失败，错误码: %d", ret);
                        last_warn_time = current_time;
                    }
                }

                // 发布机器人状态
                if (has_data) {
                    rclcpp::Time now              = this->now();
                    robot_state_msg->header.stamp = now;
                    robot_state_pub_->publish(std::move(robot_state_msg));
                    joint_state_pub_->publish(std::move(joint_state_msg));
                }
            } // 互斥锁在这里释放

            // 在锁外发布TF，减少持锁时间
            if (has_valid_pose) {
                try {
                    if (tcp_pose_msg.x != 0 || tcp_pose_msg.y != 0 || tcp_pose_msg.z != 0) {
                        // 通过共享互斥锁调用
                        std::lock_guard<std::mutex> lock(robot_mutex_);
                        publish_tf_transforms(tcp_pose_msg);
                    }
                } catch (const std::exception& e) {
                    auto current_time = thread_clock.now();
                    if ((current_time - last_error_time).seconds() > 5.0) {
                        std::lock_guard<std::mutex> lock(robot_mutex_);
                        RCLCPP_ERROR(this->get_logger(), "发布TF变换时发生异常: %s", e.what());
                        last_error_time = current_time;
                    }
                }
            }
        } catch (const std::exception& e) {
            auto current_time = thread_clock.now();
            if ((current_time - last_error_time).seconds() > 5.0) {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                RCLCPP_ERROR(this->get_logger(), "获取机器人状态时发生异常: %s", e.what());
                last_error_time = current_time;
            }
        }

        // 按照设定的状态查询间隔休眠
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(state_query_interval_ * 1000)));
    }

    // 线程结束日志 - 使用节点的主线程发送
    RCLCPP_INFO(this->get_logger(), "机器人状态监控线程已退出");
}

void RoboCtrlNode::handle_robot_set_speed(
    const std::shared_ptr<robo_ctrl::srv::RobotSetSpeed::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotSetSpeed::Response> response
) {
    if (!robot_ || !is_connected_) {
        response->success = false;
        response->message = "机器人未连接";
        return;
    }

    try {
        // 设置速度和加速度
        robot_->SetSpeed(request->speed);
        response->success = true;
        response->message = "设置速度和加速度成功";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("设置速度时发生异常: ") + e.what();
    }
}

} // namespace robo_ctrl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robo_ctrl::RoboCtrlNode)
