/*
AI-Deep:
    task1 v2 — 简化版：识别可乐 → 左臂抓取 → 后退 → 回初始位
    从原 task1 (开瓶盖+倒水完整流程) 精简而来，删去 Step 7-14
    Step 8-14 (开瓶盖/倒水/右臂操作) 待后续任务使用
*/
#include "dualarm/headers.hpp"
#include "dualarm/service_server_template.hpp"
#include <thread>
#include "main.h"

// AI-Deep: 左右爪分节点后,各自独立RS485总线,slave_id均为9
#define GRIPPER_ID_L    9
#define GRIPPER_ID_R    9
#define GRIPPER_DISABLE 0
#define GRIPPER_ENABLE  1
#define GRIPPER_SET     2
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   255

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMain>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() { executor.spin(); });

    // ══════════════════════════════════════════════════════════════════════
    // Step 1: 使能左夹爪
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 1: 使能左夹爪");

    {
        auto req      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
        req->slave_id = GRIPPER_ID_L;
        req->command  = GRIPPER_ENABLE;
        req->position = GRIPPER_OPEN;
        req->speed    = 255;
        req->torque   = 255;
        auto resp     = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->gripper_command_client_, req, node,
            std::chrono::seconds(5), "gripper_enable_L");
        if (!resp->success) {
            RCLCPP_ERROR(node->get_logger(), "Failed to enable left gripper");
            rclcpp::shutdown(); spin_thread.join(); return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Left gripper enabled");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ══════════════════════════════════════════════════════════════════════
    // Step 2: 打开左夹爪
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 2: 打开左夹爪");

    auto gripper_open_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_open_request->slave_id = GRIPPER_ID_L;
    gripper_open_request->command  = GRIPPER_SET;
    gripper_open_request->position = GRIPPER_OPEN;
    gripper_open_request->speed    = 255;
    gripper_open_request->torque   = 255;

    auto gripper_open_response =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->gripper_command_client_, gripper_open_request, node,
            std::chrono::seconds(5), "gripper_open_L");

    if (!gripper_open_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s",
                     gripper_open_response->message.c_str());
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");

    // ══════════════════════════════════════════════════════════════════════
    // Step 3: 等待物体检测数据，寻找可乐 (ID=1)
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 3: 等待检测可乐...");

    std::vector<double> tcp_to_cola_increment;
    std::vector<double> cola_position;
    bool cola_found = false;

    // 等待检测出现
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(node->get_logger(), "Detected %zu objects", node->getDetectedObjectsCount());

    if (node->hasObject(1)) {
        cola_position = node->getObjectPosition(1);
        if (!cola_position.empty()) {
            RCLCPP_INFO(node->get_logger(), "Cola (ID=1) found at position: [%.3f, %.3f, %.3f]",
                        cola_position[0], cola_position[1], cola_position[2]);
            tcp_to_cola_increment = node->calculateTcpToObjectIncrement(cola_position);
            cola_found            = true;
        }
    }

    if (!cola_found) {
        RCLCPP_ERROR(node->get_logger(), "Cola (ID=1) not found! Cannot proceed.");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // 计算目标TCP位置（增量）
    std::vector<double> target_tcp_position = {
        tcp_to_cola_increment[0], tcp_to_cola_increment[1], tcp_to_cola_increment[2]};

    RCLCPP_INFO(node->get_logger(), "Target TCP position: [%.3f, %.3f, %.3f]",
                target_tcp_position[0], target_tcp_position[1], target_tcp_position[2]);

    // 禁用物体位置更新
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    node->setObjectUpdateEnabled(false);
    RCLCPP_INFO(node->get_logger(), "Object update disabled for grasping operation");

    // ══════════════════════════════════════════════════════════════════════
    // Step 4: 修正左臂姿态到 (-90, 0, -90)
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 4: 修正姿态");

    bool retFlag;
    int retVal;
    double total_angle_diff;
    std::vector<double> orientation_increment;
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Request>  fix_request;
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Response> fix_response;

    retVal = L_fix(node, orientation_increment, total_angle_diff,
                   fix_request, fix_response, retFlag);
    if (retFlag) {
        rclcpp::shutdown(); spin_thread.join(); return retVal;
    }

    int wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully");

    // ═══════════════════════════════════════════════════════════════
    // 用 TCP 标定结果计算夹爪尖端增量 (替代硬编码 -132, +45)
    //
    // 标定数据 (static_transforms.yaml 2026-06-04):
    //   Ltcp→Lgripper_tip: [-0.00166, 0.00182, 0.18564] m
    //
    // 抓取姿态 rx=-90°, ry=0°, rz=-90° 下:
    //   R = Rz(-90°)*Rx(-90°) = [[0,0,1],[-1,0,0],[0,-1,0]]
    //   tcp_offset_base = R * [-1.66, 1.82, 185.64]^T
    //                   = [185.64, 1.66, -1.82] mm
    // ═══════════════════════════════════════════════════════════════
    const double TCP_OFFSET_TCP_X = -1.66;   // mm
    const double TCP_OFFSET_TCP_Y =  1.82;
    const double TCP_OFFSET_TCP_Z = 185.64;

    // R * [tx, ty, tz]^T = [tz, -tx, -ty]  (at grasp orientation)
    const double TIP_BASE_DX =  TCP_OFFSET_TCP_Z;   // +185.64 mm
    const double TIP_BASE_DY = -TCP_OFFSET_TCP_X;   //   +1.66 mm
    const double TIP_BASE_DZ = -TCP_OFFSET_TCP_Y;   //   -1.82 mm

    double flange_x = 0, flange_y = 0, flange_z = 0;
    {
        std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
        if (node->L_robot_state_) {
            flange_x = node->L_robot_state_->tcp_pose.x;
            flange_y = node->L_robot_state_->tcp_pose.y;
            flange_z = node->L_robot_state_->tcp_pose.z;
        }
    }

    // 夹爪尖端当前位置
    double tip_x = flange_x + TIP_BASE_DX;
    double tip_y = flange_y + TIP_BASE_DY;
    double tip_z = flange_z + TIP_BASE_DZ;

    // 目标: 夹爪尖端→可乐中心
    double desired_x = cola_position[0] * 1000.0;              // m→mm
    double desired_y = cola_position[1] * 1000.0;
    double desired_z = node->desk_height_ + node->cola_height_; // 桌面+可乐半高

    // 法兰增量 = 目标尖端位置 - 当前尖端位置
    target_tcp_position[0] = desired_x - tip_x;
    target_tcp_position[1] = desired_y - tip_y;
    target_tcp_position[2] = desired_z - tip_z;

    RCLCPP_INFO(node->get_logger(),
        "法兰:[%.1f,%.1f,%.1f] 尖端:[%.1f,%.1f,%.1f] → 目标:[%.1f,%.1f,%.1f] 增量:[%.1f,%.1f,%.1f]",
        flange_x, flange_y, flange_z,
        tip_x, tip_y, tip_z,
        desired_x, desired_y, desired_z,
        target_tcp_position[0], target_tcp_position[1], target_tcp_position[2]);

    // ══════════════════════════════════════════════════════════════════════
    // Step 5: 左臂移动到可乐位置
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 5: 移动到可乐位置");

    // 增量已在 Step 4.5 中通过 TCP 标定精确计算
    auto act_request             = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    act_request->command_type    = 0; // ServoMoveStart
    act_request->tcp_pose.x      = target_tcp_position[0];
    act_request->tcp_pose.y      = target_tcp_position[1];
    act_request->tcp_pose.z      = target_tcp_position[2];
    act_request->tcp_pose.rx     = 0.0;
    act_request->tcp_pose.ry     = 0.0;
    act_request->tcp_pose.rz     = 0.0;
    act_request->point_count     = 180;
    act_request->message_time    = 0.01;
    act_request->plan_type       = 0;    // 直线规划
    act_request->use_incremental = true; // 增量运动

    auto act_response = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
        node->L_robot_act_client_, act_request, node, std::chrono::seconds(10),
        "L/robot_act_cola");

    if (!act_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move robot to cola position: %s",
                     act_response->message.c_str());
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 3000)));
    RCLCPP_INFO(node->get_logger(), "Robot moved to cola position successfully");

    // ══════════════════════════════════════════════════════════════════════
    // Step 6: 合上左夹爪 → 后退离开桌面
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 6: 夹住可乐并后退");

    auto gripper_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_request->slave_id = GRIPPER_ID_L;
    gripper_request->command  = GRIPPER_SET;
    gripper_request->position = GRIPPER_CLOSE;
    gripper_request->speed    = 255;
    gripper_request->torque   = 255;
    auto gripper_response     = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_request, node, std::chrono::seconds(5),
        "/epg50_gripper/command");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");

    // 后退离开桌面
    auto exit_request           = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    exit_request->tcp_pose.x    = -150;
    exit_request->tcp_pose.y    = 0.0;
    exit_request->tcp_pose.z    = 30;
    exit_request->tcp_pose.rx   = 0.0;
    exit_request->tcp_pose.ry   = 0.0;
    exit_request->tcp_pose.rz   = 0.0;
    exit_request->acceleration  = 100;
    exit_request->velocity      = 10;
    exit_request->config        = -1;
    exit_request->blend_time    = 0.0;
    exit_request->use_increment = true;
    exit_request->tool          = -1;
    exit_request->user          = -1;
    exit_request->ovl           = 0;
    auto exit_response          = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, exit_request, node, std::chrono::seconds(10),
        "L/robot_move_cart_exit");

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000 + 1500)));
    RCLCPP_INFO(node->get_logger(), "Robot exited danger zone");

    // ══════════════════════════════════════════════════════════════════════
    // Step 7: 回到初始观察位
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 7: 回到初始位");

    rclcpp::sleep_for(std::chrono::seconds(2));

    auto look_at_table_request           = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    look_at_table_request->tcp_pose.x    = node->init_tcp_pose_vec_[0];
    look_at_table_request->tcp_pose.y    = node->init_tcp_pose_vec_[1];
    look_at_table_request->tcp_pose.z    = node->init_tcp_pose_vec_[2];
    look_at_table_request->tcp_pose.rx   = node->init_tcp_pose_vec_[3];
    look_at_table_request->tcp_pose.ry   = node->init_tcp_pose_vec_[4];
    look_at_table_request->tcp_pose.rz   = node->init_tcp_pose_vec_[5];
    look_at_table_request->acceleration  = 100;
    look_at_table_request->velocity      = 10;
    look_at_table_request->config        = -1;
    look_at_table_request->blend_time    = 0.0;
    look_at_table_request->use_increment = false;
    look_at_table_request->tool          = -1;
    look_at_table_request->user          = -1;
    look_at_table_request->ovl           = 0;

    auto look_at_table_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, look_at_table_request, node,
        std::chrono::seconds(10), "L/robot_move_cart_init");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 重新启用物体检测
    node->setObjectUpdateEnabled(true);

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), " Task1: 识别 → 抓取 完成！");
    RCLCPP_INFO(node->get_logger(), "========================================");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
