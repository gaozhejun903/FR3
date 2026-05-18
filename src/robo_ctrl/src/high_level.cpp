#include "robo_ctrl/high_level_ctrl.hpp"

using namespace std::placeholders;

// ========================= plan_and_execute_node 实现 =========================
plan_and_execute_node::plan_and_execute_node(const rclcpp::NodeOptions& options):
    Node("plan_and_execute_node", options) {
    this->declare_parameter<std::string>("robot_name", "/L");
    this->get_parameter("robot_name", robot_name);

    // 初始化TF2 buffer和listener
    tf_buffer_         = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_       = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    last_cache_update_ = this->now();

    // 创建服务，处理机器人动作请求
    robot_move_service_ = this->create_service<robo_ctrl::srv::RobotAct>(
        robot_name + "/robot_act",
        std::bind(&plan_and_execute_node::handle_robot_act, this, _1, _2)
    );

    // 创建RobotActJ服务，处理关节角度动作请求
    robot_act_j_service_ = this->create_service<robo_ctrl::srv::RobotActJ>(
        robot_name + "/robot_act_j",
        std::bind(&plan_and_execute_node::handle_robot_act_j, this, _1, _2)
    );

    // 创建客户端，用于发送伺服线性运动命令
    robot_servo_line_client_ = this->create_client<robo_ctrl::srv::RobotServoLine>(robot_name + "/robot_servo_line");

    // 创建客户端，用于发送伺服关节运动命令
    robot_servo_joint_client_ = this->create_client<robo_ctrl::srv::RobotServoJoint>(robot_name + "/robot_servo_joint");

    // 创建轨迹可视化发布器
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_visualization", 10);

    // 创建机器人状态订阅器
    robot_state_sub_ = this->create_subscription<robo_ctrl::msg::RobotState>(
        robot_name + "/robot_state",
        10,
        std::bind(&plan_and_execute_node::robot_state_callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "计划与执行节点已启动");
}

plan_and_execute_node::~plan_and_execute_node() {
    RCLCPP_INFO(this->get_logger(), "计划与执行节点已关闭");
}

void plan_and_execute_node::handle_robot_act(
    const std::shared_ptr<robo_ctrl::srv::RobotAct::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotAct::Response> response
) {
    RCLCPP_INFO(this->get_logger(), "收到机器人动作请求，命令类型: %d", request->command_type);

    // 检查客户端是否可用
    if (!robot_servo_line_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "robot_servo_line服务不可用");
        response->success = false;
        response->message = "robot_servo_line服务不可用";
        return;
    }

    // 创建规划器对象
    planner trajectory_planner;
    geometry_msgs::msg::Pose current_pose;

    if (!request->use_incremental) {
        // 绝对请求
        // 获取当前机器人位姿
        if (!robot_state_msg_) {
            response->success = false;
            response->message = "机器人状态未初始化";
            RCLCPP_ERROR(this->get_logger(), "机器人状态未初始化");
            return;
        }
        // use mutex to safely access the robot state
        std::lock_guard<std::mutex> lock(robot_mutex_);
        robo_ctrl::msg::TCPPose tcp_pose = robot_state_msg_->tcp_pose;
        current_pose.position.x          = tcp_pose.x; // 毫米转米
        current_pose.position.y          = tcp_pose.y;
        current_pose.position.z          = tcp_pose.z;
        tf2::Quaternion q;
        q.setRPY(
            tcp_pose.rx * M_PI / 180.0, // 度转弧度
            tcp_pose.ry * M_PI / 180.0,
            tcp_pose.rz * M_PI / 180.0
        );
        current_pose.orientation = tf2::toMsg(q);
    } else {
        // 增量请求
        current_pose.position.x    = 0.0;
        current_pose.position.y    = 0.0;
        current_pose.position.z    = 0.0;
        current_pose.orientation.w = 1.0; // 默认四元数
        current_pose.orientation.x = 0.0;
        current_pose.orientation.y = 0.0;
        current_pose.orientation.z = 0.0;
    }

    // 将TCPPose转换为geometry_msgs::Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->tcp_pose.x; // 毫米转米
    target_pose.position.y = request->tcp_pose.y;
    target_pose.position.z = request->tcp_pose.z;

    double roll  = request->tcp_pose.rx * M_PI / 180.0; // 度转弧度
    double pitch = request->tcp_pose.ry * M_PI / 180.0;
    double yaw   = request->tcp_pose.rz * M_PI / 180.0;

    // 使用tf2进行欧拉角到四元数转换
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target_pose.orientation = tf2::toMsg(q);

    // 设置起点和终点
    trajectory_planner.set_start_pose(current_pose);
    trajectory_planner.set_target_pose(target_pose);

    // 计划轨迹
    std::vector<geometry_msgs::msg::Pose> trajectory;
    // 默认值为 100
    trajectory_planner.set_planning_count(request->point_count == 0 ? 100 : request->point_count);

    // 设置圆弧规划参数
    if (request->plan_type != 0) { // 圆弧规划
        trajectory_planner.set_circle_center(request->circle_center);
        trajectory_planner.radius_ = request->circle_center.x; // 圆弧半径
        trajectory_planner.set_face_center(request->face_center);
        trajectory_planner.vector_ = geometry_msgs::msg::Vector3(request->initial_orientation); // 设置切线向量
        trajectory_planner.rotation_angle_     = request->radian * M_PI / 180.0; // 将角度转换为弧度
        trajectory_planner.x_points_to_center_ = request->face_center;           // 是否让X轴指向圆心
    }

    trajectory_planner.plan_trajectory(request->plan_type == 0 ? planner::LINEAR : planner::CIRCULAR, trajectory);
    trajectory_planner.get_trajectory(trajectory);

    RCLCPP_INFO(this->get_logger(), "规划完成，轨迹点数量: %zu", trajectory.size());

    // 在圆弧规划后进行坐标变换
    if (request->plan_type != 0) { // 圆弧规划
        std::vector<geometry_msgs::msg::Pose> transformed_trajectory;
        if (transformPosesFromLfakeGripper(trajectory, transformed_trajectory, robot_name + "robot_base")) {
            trajectory = std::move(transformed_trajectory);
            RCLCPP_INFO(
                this->get_logger(),
                "圆弧轨迹已从Lfake_gripper_frame变换到robot_base，共%zu个点",
                trajectory.size()
            );
        } else {
            RCLCPP_WARN(this->get_logger(), "圆弧轨迹坐标变换失败，使用原始轨迹");
        }
    }

    if (request->use_incremental || request->plan_type != 0) {
        std::vector<geometry_msgs::msg::Pose> path_points = trajectory;
        RCLCPP_INFO(this->get_logger(), "增量运动模式");
        for (size_t i = 0; i < trajectory.size(); ++i) {
            if (i == 0)
                continue;
            trajectory[i].position.x -= path_points[i - 1].position.x;
            trajectory[i].position.y -= path_points[i - 1].position.y;
            trajectory[i].position.z -= path_points[i - 1].position.z;
            // 四元数取差
            tf2::Quaternion q1(
                path_points[i - 1].orientation.x,
                path_points[i - 1].orientation.y,
                path_points[i - 1].orientation.z,
                path_points[i - 1].orientation.w
            );
            tf2::Quaternion q2(
                trajectory[i].orientation.x,
                trajectory[i].orientation.y,
                trajectory[i].orientation.z,
                trajectory[i].orientation.w
            );
            tf2::Quaternion q_diff = q2 * q1.inverse();
            // 归一化四元数
            q_diff.normalize();
            trajectory[i].orientation.x = q_diff.x();
            trajectory[i].orientation.y = q_diff.y();
            trajectory[i].orientation.z = q_diff.z();
            trajectory[i].orientation.w = q_diff.w();

            RCLCPP_INFO(
                this->get_logger(),
                "ServoLine路径点[%ld]: [%f, %f, %f, %f, %f, %f,%f]",
                i,
                trajectory[i].position.x,
                trajectory[i].position.y,
                trajectory[i].position.z,
                trajectory[i].orientation.x,
                trajectory[i].orientation.y,
                trajectory[i].orientation.z,
                trajectory[i].orientation.w
            );
        }
        if (request->plan_type) {
            trajectory[0].position.x    = 0;
            trajectory[0].position.y    = 0;
            trajectory[0].position.z    = 0;
            trajectory[0].orientation.x = 0;
            trajectory[0].orientation.y = 0;
            trajectory[0].orientation.z = 0;
            trajectory[0].orientation.w = 1.0; // 保持初始姿
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "绝对运动模式");
    }

    // 可视化轨迹
    visualize_trajectory(trajectory);

    // 发送轨迹到机器人控制服务
    auto servo_request          = std::make_shared<robo_ctrl::srv::RobotServoLine::Request>();
    servo_request->command_type = request->command_type;

    // 将轨迹点转换为PoseArray
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.poses              = trajectory;
    servo_request->cartesian_pose = pose_array;

    // 设置其他参数
    servo_request->acc             = 50.0;                      // 默认加速度百分比
    servo_request->vel             = 30.0;                      // 默认速度百分比
    servo_request->cmd_time        = request->message_time;     // 指令周期时间
    servo_request->filter_time     = request->message_time / 3; // 默认滤波时间
    servo_request->gain            = 0.0;                       // 默认增益
    servo_request->use_incremental = request->use_incremental;  // 是否使用增量运动

    // 发送请求
    auto servo_future = robot_servo_line_client_->async_send_request(
        servo_request,
        [this](rclcpp::Client<robo_ctrl::srv::RobotServoLine>::SharedFuture future) {
            auto servo_response = future.get();

            RCLCPP_INFO(
                this->get_logger(),
                "机器人轨迹执行%s: %s",
                servo_response->success ? "成功" : "失败",
                servo_response->message.c_str()
            );

            // 如果你需要把这个结果传给某个成员变量或别的系统处理，可以在这里做
            // 比如：
            // this->last_result_ = servo_response->success;
        }
    );
    response->success = true;
    response->message = "执行成功";
}

void plan_and_execute_node::handle_robot_act_j(
    const std::shared_ptr<robo_ctrl::srv::RobotActJ::Request> request,
    std::shared_ptr<robo_ctrl::srv::RobotActJ::Response> response
) {
    RCLCPP_INFO(this->get_logger(), "收到关节动作请求，命令类型: %d", request->command_type);

    // 检查客户端是否可用
    if (!robot_servo_joint_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "robot_servo_joint服务不可用");
        response->success = false;
        response->message = "robot_servo_joint服务不可用";
        return;
    }

    // 验证关节角度限制
    const std::vector<double> joint_limits_min = { -175.0, -265.0, -150.0, -265.0, -175.0, -175.0 };
    const std::vector<double> joint_limits_max = { 175.0, 85.0, 150.0, 85.0, 175.0, 175.0 };

    if (request->target_joints.size() != 6) {
        response->success = false;
        response->message = "关节角度数组长度必须为6";
        RCLCPP_ERROR(this->get_logger(), "关节角度数组长度错误: %zu", request->target_joints.size());
        return;
    }

    // 检查关节角度是否在限制范围内
    for (size_t i = 0; i < request->target_joints.size(); ++i) {
        if (request->target_joints[i] < joint_limits_min[i] || request->target_joints[i] > joint_limits_max[i]) {
            response->success = false;
            response->message = "关节" + std::to_string(i + 1) + "角度超出限制范围["
                + std::to_string(joint_limits_min[i]) + ", " + std::to_string(joint_limits_max[i]) + "]";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
    }

    // 根据命令类型处理
    if (request->command_type == 0) { // ServoJStart
        RCLCPP_INFO(
            this->get_logger(),
            "开始ServoJ运动，目标关节角度: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            request->target_joints[0],
            request->target_joints[1],
            request->target_joints[2],
            request->target_joints[3],
            request->target_joints[4],
            request->target_joints[5]
        );

        // 获取当前关节角度
        std::vector<double> current_joints(6, 0.0);
        if (!request->use_incremental) {
            // 绝对运动模式，需要获取当前关节角度
            if (!robot_state_msg_) {
                response->success = false;
                response->message = "机器人状态未初始化";
                RCLCPP_ERROR(this->get_logger(), "机器人状态未初始化");
                return;
            }

            std::lock_guard<std::mutex> lock(robot_mutex_);
            // 从JointPosition消息中获取关节角度（已经是度）
            current_joints[0] = robot_state_msg_->joint_position.j1;
            current_joints[1] = robot_state_msg_->joint_position.j2;
            current_joints[2] = robot_state_msg_->joint_position.j3;
            current_joints[3] = robot_state_msg_->joint_position.j4;
            current_joints[4] = robot_state_msg_->joint_position.j5;
            current_joints[5] = robot_state_msg_->joint_position.j6;
        } else {
            // 增量运动模式，当前关节角度默认为0
            current_joints = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            RCLCPP_INFO(this->get_logger(), "增量运动模式，当前关节角度默认为[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]");
        }

        // 生成轨迹点
        std::vector<sensor_msgs::msg::JointState> joint_trajectory;
        int point_count = request->point_count > 0 ? request->point_count : 100;

        for (int i = 0; i < point_count; ++i) {
            sensor_msgs::msg::JointState joint_state;
            joint_state.position.resize(6);

            for (size_t j = 0; j < 6; ++j) {
                if (request->use_incremental) {
                    // 增量运动模式，计算增量
                    joint_state.position[j] = (request->target_joints[j] / point_count);
                } else {
                    // 绝对运动模式，直接使用目标角度
                    joint_state.position[j] =
                        current_joints[j] + (i * (request->target_joints[j] - current_joints[j]) / point_count);
                }
            }
            RCLCPP_INFO(
                this->get_logger(),
                "ServoJ轨迹点[%d]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                i,
                joint_state.position[0],
                joint_state.position[1],
                joint_state.position[2],
                joint_state.position[3],
                joint_state.position[4],
                joint_state.position[5]
            );

            joint_trajectory.push_back(joint_state);
        }

        // 创建ServoJoint请求
        auto servo_request             = std::make_shared<robo_ctrl::srv::RobotServoJoint::Request>();
        servo_request->command_type    = 0; // ServoMoveStart
        servo_request->joint_positions = joint_trajectory;
        servo_request->acc             = 100; // 默认加速度百分比
        servo_request->vel             = 100; // 默认速度百分比
        servo_request->cmd_time        = request->message_time;
        servo_request->filter_time     = -1; // 默认滤波时间
        servo_request->gain            = 0.0;
        servo_request->use_incremental = request->use_incremental;

        // 发送请求
        auto servo_future = robot_servo_joint_client_->async_send_request(
            servo_request,
            [this](rclcpp::Client<robo_ctrl::srv::RobotServoJoint>::SharedFuture future) {
                auto servo_response = future.get();
                RCLCPP_INFO(
                    this->get_logger(),
                    "ServoJ执行%s: %s",
                    servo_response->success ? "成功" : "失败",
                    servo_response->message.c_str()
                );
            }
        );

        response->success = true;
        response->message = "ServoJ开始执行";

    } else if (request->command_type == 1) { // ServoJEnd
        RCLCPP_INFO(this->get_logger(), "结束ServoJ运动");

        // 创建ServoJoint结束请求
        auto servo_request             = std::make_shared<robo_ctrl::srv::RobotServoJoint::Request>();
        servo_request->command_type    = 1; // ServoMoveEnd
        servo_request->use_incremental = request->use_incremental;

        // 发送请求
        auto servo_future = robot_servo_joint_client_->async_send_request(
            servo_request,
            [this](rclcpp::Client<robo_ctrl::srv::RobotServoJoint>::SharedFuture future) {
                auto servo_response = future.get();
                RCLCPP_INFO(
                    this->get_logger(),
                    "ServoJ结束%s: %s",
                    servo_response->success ? "成功" : "失败",
                    servo_response->message.c_str()
                );
            }
        );

        response->success = true;
        response->message = "ServoJ结束执行";

    } else {
        response->success = false;
        response->message = "不支持的命令类型: " + std::to_string(request->command_type);
        RCLCPP_ERROR(this->get_logger(), "不支持的命令类型: %d", request->command_type);
    }
}

void plan_and_execute_node::visualize_trajectory(const std::vector<geometry_msgs::msg::Pose>& trajectory) {
    if (trajectory.empty()) {
        RCLCPP_WARN(this->get_logger(), "轨迹为空，无法可视化");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // 创建线条标记
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "robot_base"; // 假设基座标系
    line_marker.header.stamp    = this->now();
    line_marker.ns              = "trajectory_line";
    line_marker.id              = 0;
    line_marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action          = visualization_msgs::msg::Marker::ADD;

    // 设置线条属性
    line_marker.scale.x = 0.005; // 线宽 5mm
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    // 将轨迹点添加到线条中
    for (const auto& pose: trajectory) {
        geometry_msgs::msg::Point point;
        point.x = pose.position.x;
        point.y = pose.position.y;
        point.z = pose.position.z;
        line_marker.points.push_back(point);
    }

    marker_array.markers.push_back(line_marker);

    // 创建轨迹点标记
    for (size_t i = 0; i < trajectory.size(); i++) {
        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = "robot_base";
        point_marker.header.stamp    = this->now();
        point_marker.ns              = "trajectory_points";
        point_marker.id              = i + 1;
        point_marker.type            = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action          = visualization_msgs::msg::Marker::ADD;

        point_marker.pose.position = trajectory[i].position;
        point_marker.scale.x       = 0.01; // 球体直径 1cm
        point_marker.scale.y       = 0.01;
        point_marker.scale.z       = 0.01;

        // 起点绿色，终点蓝色，中间点黄色
        if (i == 0) {
            point_marker.color.r = 0.0;
            point_marker.color.g = 1.0;
            point_marker.color.b = 0.0;
        } else if (i == trajectory.size() - 1) {
            point_marker.color.r = 0.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 1.0;
        } else {
            point_marker.color.r = 1.0;
            point_marker.color.g = 1.0;
            point_marker.color.b = 0.0;
        }
        point_marker.color.a = 1.0;

        marker_array.markers.push_back(point_marker);
    }

    // 发布标记
    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "已发布轨迹可视化，共%ld个点", trajectory.size());
}

// ========================= planner 实现 =========================
planner::planner() {
    // 构造函数，初始化成员变量
}

planner::~planner() {
    // 析构函数
}

void planner::set_start_pose(const geometry_msgs::msg::Pose& start_pose) {
    start_pose_ = start_pose;
}

void planner::set_target_pose(const geometry_msgs::msg::Pose& target_pose) {
    target_pose_ = target_pose;
}

void planner::plan_trajectory(planner::PLAN_TYPE plan_type, std::vector<robo_ctrl::msg::TCPPose> waypoints) {
    // 清空现有轨迹
    trajectory_.clear();

    // 根据规划类型生成轨迹
    if (plan_type == LINEAR) { // LINEAR
        linear_planning(start_pose_, target_pose_, trajectory_);
    } else { // CIRCULAR
        circular_planning(radius_, vector_, rotation_angle_, x_points_to_center_, trajectory_);
    }
    // change pose to TCPPose
    for (const auto& waypoint: trajectory_) {
        robo_ctrl::msg::TCPPose tcp_pose;
        tcp_pose.x = waypoint.position.x * 1000.0; // 米转毫米
        tcp_pose.y = waypoint.position.y * 1000.0;
        tcp_pose.z = waypoint.position.z * 1000.0;

        // 四元数转欧拉角（度）
        tf2::Quaternion q;
        tf2::fromMsg(waypoint.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        tcp_pose.rx = roll * 180.0 / M_PI; // 弧度转度
        tcp_pose.ry = pitch * 180.0 / M_PI;
        tcp_pose.rz = yaw * 180.0 / M_PI;

        waypoints.push_back(tcp_pose);
    }
}

void planner::plan_trajectory(planner::PLAN_TYPE plan_type, std::vector<geometry_msgs::msg::Pose> waypoints) {
    // 清空现有轨迹
    trajectory_.clear();

    // 根据规划类型生成轨迹
    if (plan_type == LINEAR) { // LINEAR
        linear_planning(start_pose_, target_pose_, trajectory_);
    } else { // CIRCULAR
        circular_planning(radius_, vector_, rotation_angle_, x_points_to_center_, trajectory_);
    }

    waypoints.clear();
    waypoints = trajectory_;
}

void planner::linear_planning(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& target_pose,
    std::vector<geometry_msgs::msg::Pose>& trajectory
) {
    // 清空轨迹
    trajectory.clear();

    // 添加起始点
    trajectory.push_back(start_pose);

    // 设置轨迹点数量（可根据距离动态调整）
    const int num_points = planning_count_; // 默认50个点

    // 计算增量
    double dx = (target_pose.position.x - start_pose.position.x) / (num_points - 1);
    double dy = (target_pose.position.y - start_pose.position.y) / (num_points - 1);
    double dz = (target_pose.position.z - start_pose.position.z) / (num_points - 1);

    // 生成中间点
    for (int i = 1; i < num_points - 1; ++i) {
        geometry_msgs::msg::Pose intermediate_pose;

        // 位置插值
        intermediate_pose.position.x = start_pose.position.x + i * dx;
        intermediate_pose.position.y = start_pose.position.y + i * dy;
        intermediate_pose.position.z = start_pose.position.z + i * dz;
        RCLCPP_INFO(
            rclcpp::get_logger("planner"),
            "插值点 %d: (%f, %f, %f)",
            i,
            intermediate_pose.position.x,
            intermediate_pose.position.y,
            intermediate_pose.position.z
        );

        // 姿态插值 - 球面线性插值(SLERP)
        // 简化版，实际应用中应使用完整的四元数SLERP
        double t = static_cast<double>(i) / (num_points - 1);

        // SLERP简化公式
        double cos_half_theta = start_pose.orientation.w * target_pose.orientation.w
            + start_pose.orientation.x * target_pose.orientation.x
            + start_pose.orientation.y * target_pose.orientation.y
            + start_pose.orientation.z * target_pose.orientation.z;

        // 确保走最短路径
        double factor1, factor2;

        if (cos_half_theta < 0.0) {
            cos_half_theta = -cos_half_theta;
            factor1        = sin((1.0 - t) * M_PI / 2) / sin(M_PI / 2);
            factor2        = -sin(t * M_PI / 2) / sin(M_PI / 2);
        } else {
            double half_theta     = acos(cos_half_theta);
            double sin_half_theta = sin(half_theta);

            if (fabs(sin_half_theta) < 1e-10) {
                // 四元数接近，线性插值
                factor1 = 1.0 - t;
                factor2 = t;
            } else {
                factor1 = sin((1.0 - t) * half_theta) / sin_half_theta;
                factor2 = sin(t * half_theta) / sin_half_theta;
            }
        }

        // 计算插值后的四元数
        intermediate_pose.orientation.w = factor1 * start_pose.orientation.w + factor2 * target_pose.orientation.w;
        intermediate_pose.orientation.x = factor1 * start_pose.orientation.x + factor2 * target_pose.orientation.x;
        intermediate_pose.orientation.y = factor1 * start_pose.orientation.y + factor2 * target_pose.orientation.y;
        intermediate_pose.orientation.z = factor1 * start_pose.orientation.z + factor2 * target_pose.orientation.z;

        // 归一化四元数
        double norm = sqrt(
            intermediate_pose.orientation.w * intermediate_pose.orientation.w
            + intermediate_pose.orientation.x * intermediate_pose.orientation.x
            + intermediate_pose.orientation.y * intermediate_pose.orientation.y
            + intermediate_pose.orientation.z * intermediate_pose.orientation.z
        );

        intermediate_pose.orientation.w /= norm;
        intermediate_pose.orientation.x /= norm;
        intermediate_pose.orientation.y /= norm;
        intermediate_pose.orientation.z /= norm;

        trajectory.push_back(intermediate_pose);
    }
    // transform points to base frame

    // 添加终点
    trajectory.push_back(target_pose);
}

void planner::circular_planning(
    double radius,                       // 圆弧半径
    geometry_msgs::msg::Vector3& vector, // 圆弧原点处的切线向量
    double rotation_angle,               // 旋转弧度 沿切向量方向
    bool x_points_to_center,             // 是否让X轴指向圆心
    std::vector<geometry_msgs::msg::Pose>& trajectory
) {
    // 将切向量转换为 Eigen 向量并归一化
    Eigen::Vector3d T0(vector.x, vector.y, vector.z);
    if (T0.norm() < 1e-6) {
        // 切向量为零，无法规划圆弧
        return;
    }
    T0.normalize();

    // 选择一个参考方向来确定圆弧平面
    // 这里选择与切向量最不平行的坐标轴
    Eigen::Vector3d ref_axis;
    if (std::abs(T0.x()) < 0.9) {
        ref_axis = Eigen::Vector3d(1, 0, 0);
    } else {
        ref_axis = Eigen::Vector3d(0, 1, 0);
    }

    // 计算圆弧平面的法向量
    Eigen::Vector3d N = T0.cross(ref_axis);
    N.normalize();

    // 计算径向方向（从原点指向圆心的方向）
    Eigen::Vector3d R = N.cross(T0);
    R.normalize();

    // 圆心位置：从原点沿径向方向偏移radius
    Eigen::Vector3d center = R * radius;

    // 构建正交坐标系
    Eigen::Vector3d e1 = R;  // 径向方向（从原点到圆心）
    Eigen::Vector3d e2 = T0; // 切向方向
    Eigen::Vector3d e3 = N;  // 法向方向

    // 起始位置
    Eigen::Vector3d start_position(0.0, 0.0, 0.0);

    // 插值数量
    size_t num_points = planning_count_;
    if (num_points < 2)
        num_points = 2;

    // 生成轨迹点
    for (size_t i = 0; i < num_points; ++i) {
        // 计算当前插值角度
        double phi = i * rotation_angle / (num_points - 1);

        // 计算圆弧上的位置（相对于圆心）
        Eigen::Vector3d pos_relative = radius * (cos(phi) * (-e1) + sin(phi) * e2);

        // 转换到全局坐标系
        Eigen::Vector3d position = center + pos_relative;

        // 计算姿态
        Eigen::Quaterniond q;
        if (x_points_to_center) {
            // X轴指向圆心
            Eigen::Vector3d x_axis = (center - position).normalized();
            Eigen::Vector3d z_axis = e3; // 保持法向量作为Z轴
            Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

            // 构建旋转矩阵
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix.col(0) = x_axis;
            rotation_matrix.col(1) = y_axis;
            rotation_matrix.col(2) = z_axis;

            Eigen::Matrix3d z_rot;
            z_rot           = Eigen::AngleAxisd(phi, z_axis);
            rotation_matrix = z_rot;

            q = Eigen::Quaterniond(rotation_matrix);
        } else {
            // 绕法向量旋转，保持相对姿态
            q = Eigen::Quaterniond(Eigen::AngleAxisd(phi, e3));
        }

        // 归一化四元数
        q.normalize();

        // 创建pose消息
        geometry_msgs::msg::Pose pose;
        pose.position.x    = position.x();
        pose.position.y    = position.y();
        pose.position.z    = position.z();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        trajectory.push_back(pose);
    }
}

void plan_and_execute_node::robot_state_callback(robo_ctrl::msg::RobotState::SharedPtr msg) {
    // 使用互斥锁保护对机器人状态的访问
    std::lock_guard<std::mutex> lock(robot_mutex_);
    robot_state_msg_ = msg; // 更新机器人状态消息
    RCLCPP_DEBUG(this->get_logger(), "接收到机器人状态更新");

    // 示例：变换功能演示
    static bool example_shown = false;
    if (!example_shown && this->now().seconds() > 5.0) { // 5秒后运行一次示例

        // 示例1: 点变换
        geometry_msgs::msg::Point point_in_gripper;
        point_in_gripper.x = 0.1; // 夹具坐标系下的点：前方10cm
        point_in_gripper.y = 0.0;
        point_in_gripper.z = 0.0;

        geometry_msgs::msg::Point point_in_base;
        if (transformPointFromLfakeGripper(point_in_gripper, point_in_base, robot_name + "robot_base")) {
            RCLCPP_INFO(
                this->get_logger(),
                "点变换示例成功: 夹具[%.3f, %.3f, %.3f] -> 基座[%.3f, %.3f, %.3f]",
                point_in_gripper.x,
                point_in_gripper.y,
                point_in_gripper.z,
                point_in_base.x,
                point_in_base.y,
                point_in_base.z
            );
        }

        // 示例2: 位姿向量变换
        std::vector<geometry_msgs::msg::Pose> poses_in_gripper;

        // 创建几个测试位姿
        for (int i = 0; i < 3; ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x    = 0.05 * (i + 1); // 5cm, 10cm, 15cm
            pose.position.y    = 0.0;
            pose.position.z    = 0.0;
            pose.orientation.w = 1.0; // 单位四元数
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            poses_in_gripper.push_back(pose);
        }

        std::vector<geometry_msgs::msg::Pose> poses_in_base;
        if (transformPosesFromLfakeGripper(poses_in_gripper, poses_in_base, "robot_base")) {
            RCLCPP_INFO(this->get_logger(), "位姿向量变换示例成功: 变换了%zu个位姿", poses_in_base.size());
        }

        example_shown = true;
    }
}

// 坐标变换相关
bool plan_and_execute_node::transformPointFromLfakeGripper(
    const geometry_msgs::msg::Point& point_in_gripper,
    geometry_msgs::msg::Point& point_out,
    const std::string& target_frame
) {
    // 检查并更新缓存
    if (!updateTransformCache(target_frame)) {
        RCLCPP_WARN(this->get_logger(), "无法获取从Lfake_gripper_frame到%s的变换", target_frame.c_str());
        return false;
    }

    // 从缓存中安全地读取变换
    geometry_msgs::msg::TransformStamped transform;
    {
        std::lock_guard<std::mutex> lock(transform_cache_mutex_);
        auto it = transform_cache_.find(target_frame);
        if (it == transform_cache_.end()) {
            RCLCPP_ERROR(this->get_logger(), "缓存中未找到%s的变换", target_frame.c_str());
            return false;
        }
        transform = it->second;
    }

    // 创建输入点的PointStamped消息
    geometry_msgs::msg::PointStamped point_in_stamped;
    point_in_stamped.header.frame_id = "Lfake_gripper_frame";
    point_in_stamped.header.stamp    = transform.header.stamp;
    point_in_stamped.point           = point_in_gripper;

    // 执行变换
    try {
        geometry_msgs::msg::PointStamped point_out_stamped;
        tf2::doTransform(point_in_stamped, point_out_stamped, transform);
        point_out = point_out_stamped.point;

        RCLCPP_DEBUG(
            this->get_logger(),
            "点变换成功: Lfake_gripper[%.3f, %.3f, %.3f] -> %s[%.3f, %.3f, %.3f]",
            point_in_gripper.x,
            point_in_gripper.y,
            point_in_gripper.z,
            target_frame.c_str(),
            point_out.x,
            point_out.y,
            point_out.z
        );

        return true;

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "点变换失败: %s", ex.what());
        return false;
    }
}

bool plan_and_execute_node::transformPosesFromLfakeGripper(
    const std::vector<geometry_msgs::msg::Pose>& poses_in_gripper,
    std::vector<geometry_msgs::msg::Pose>& poses_out,
    const std::string& target_frame
) {
    // 检查输入
    if (poses_in_gripper.empty()) {
        RCLCPP_WARN(this->get_logger(), "输入的位姿向量为空");
        poses_out.clear();
        return false;
    }

    // 检查并更新缓存
    if (!updateTransformCache(target_frame)) {
        RCLCPP_WARN(this->get_logger(), "无法获取从Lfake_gripper_frame到%s的变换", target_frame.c_str());
        return false;
    }

    // 从缓存中安全地读取变换
    geometry_msgs::msg::TransformStamped transform;
    {
        std::lock_guard<std::mutex> lock(transform_cache_mutex_);
        auto it = transform_cache_.find(target_frame);
        if (it == transform_cache_.end()) {
            RCLCPP_ERROR(this->get_logger(), "缓存中未找到%s的变换", target_frame.c_str());
            return false;
        }
        transform = it->second;
    }

    // 清空输出向量并预分配空间
    poses_out.clear();
    poses_out.reserve(poses_in_gripper.size());
    // 添加初始点
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.pose.position.x    = 0.0;
    initial_pose.pose.position.y    = 0.0;
    initial_pose.pose.position.z    = 0.0;
    initial_pose.pose.orientation.w = 1.0; // 单位四元数
    initial_pose.pose.orientation.x = 0.0;
    initial_pose.pose.orientation.y = 0.0;
    initial_pose.pose.orientation.z = 0.0;
    initial_pose.header.frame_id    = "Lfake_gripper_frame";
    initial_pose.header.stamp       = transform.header.stamp;
    geometry_msgs::msg::PoseStamped initial_pose_out_stamped;
    tf2::doTransform(initial_pose, initial_pose_out_stamped, transform);
    poses_out.push_back(initial_pose_out_stamped.pose);
    // 逐个变换位姿
    for (size_t i = 0; i < poses_in_gripper.size(); ++i) {
        try {
            // 创建输入位姿的PoseStamped消息
            geometry_msgs::msg::PoseStamped pose_in_stamped;
            pose_in_stamped.header.frame_id = "Lfake_gripper_frame";
            pose_in_stamped.header.stamp    = transform.header.stamp;
            pose_in_stamped.pose            = poses_in_gripper[i];

            // 执行变换
            geometry_msgs::msg::PoseStamped pose_out_stamped;
            tf2::doTransform(pose_in_stamped, pose_out_stamped, transform);

            // 添加到输出向量
            poses_out.push_back(pose_out_stamped.pose);

            RCLCPP_DEBUG(
                this->get_logger(),
                "位姿[%zu]变换成功: Lfake_gripper[%.3f, %.3f, %.3f] -> %s[%.3f, %.3f, %.3f]",
                i,
                poses_in_gripper[i].position.x,
                poses_in_gripper[i].position.y,
                poses_in_gripper[i].position.z,
                target_frame.c_str(),
                pose_out_stamped.pose.position.x,
                pose_out_stamped.pose.position.y,
                pose_out_stamped.pose.position.z
            );

        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "位姿[%zu]变换失败: %s", i, ex.what());
            poses_out.clear(); // 清空已处理的结果
            return false;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "成功变换%zu个位姿从Lfake_gripper_frame到%s",
        poses_out.size(),
        target_frame.c_str()
    );

    return true;
}

bool plan_and_execute_node::updateTransformCache(const std::string& target_frame) {
    rclcpp::Time current_time = this->now();

    // 检查缓存是否需要更新
    {
        std::lock_guard<std::mutex> lock(transform_cache_mutex_);
        auto it = transform_cache_.find(target_frame);
        if (it != transform_cache_.end()) {
            // 检查缓存是否还有效
            double time_diff = (current_time - last_cache_update_).seconds();
            if (time_diff < CACHE_TIMEOUT_SEC) {
                return true; // 缓存仍然有效
            }
        }
    }

    // 需要更新缓存，获取最新的变换
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            target_frame,
            robot_name + "fake_gripper_frame",
            tf2::TimePointZero,       // 获取最新可用的变换
            tf2::durationFromSec(0.5) // 等待0.5秒超时
        );

        // 安全地更新缓存
        {
            std::lock_guard<std::mutex> lock(transform_cache_mutex_);
            transform_cache_[target_frame] = transform;
            last_cache_update_             = current_time;
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "更新变换缓存: %s <- Lfake_gripper_frame [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
            target_frame.c_str(),
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );

        return true;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "无法获取从%s到%s的变换: %s",
            robot_name.c_str(),
            target_frame.c_str(),
            ex.what()
        );
        return false;
    }
}

// 入口点函数，用于启动ROS节点
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<plan_and_execute_node>());
    rclcpp::shutdown();
    return 0;
}
