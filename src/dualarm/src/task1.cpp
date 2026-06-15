/*
AI-Deep:
    task1 v3 — 2026-06-10: 一步到位版
      识别可乐 → 一次MoveCart(位置+姿态) → 抓取 → 后退 → 回初始位
    v2→v3 改动: 删掉 L_fix(纯姿态)+RobotAct(纯位置) 两步走,
      改为一次 MoveCart PTP规划, 同时到达目标位置和抓取姿态,
      解决两步走中间IK无解/过关节极限 → 154/14 的问题
    v2: 从原 task1 (开瓶盖+倒水完整流程) 精简而来，删去 Step 7-14
*/
#include "dualarm/headers.hpp"
#include "dualarm/service_server_template.hpp"
#include <thread>
// AI-Deep 2026-06-10: L_fix 已移除, 改为一步 MoveCart, main.h 不再需要
// #include "main.h"

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
    // Step 0: MoveCart PTP 移动到观察位
    //   TCP [110.6, -35.5, 636.7] 姿态 [174.9, -42.4, 53.9] (config.yaml)
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 0: MoveCart 移动到观察位");

    {
        // 等待 robot_state 就绪
        RCLCPP_INFO(node->get_logger(), "等待 robot_state 就绪...");
        auto t0 = node->get_clock()->now();
        while (rclcpp::ok()) {
            bool ready = false;
            {
                std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
                ready = (node->L_robot_state_ != nullptr);
            }
            if (ready) break;
            if ((node->get_clock()->now() - t0).seconds() > 10.0) {
                RCLCPP_ERROR(node->get_logger(), "等待 robot_state 超时");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(node->get_logger(), "robot_state 就绪");
    }

    // AI-Deep: 使用 RobotMoveCart (MoveCart PTP) 到观察位
    //   观察位 = init_tcp_pose (config.yaml 中定义)
    {
        auto obs_req = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
        obs_req->tcp_pose.x    = node->init_tcp_pose_vec_[0];
        obs_req->tcp_pose.y    = node->init_tcp_pose_vec_[1];
        obs_req->tcp_pose.z    = node->init_tcp_pose_vec_[2];
        obs_req->tcp_pose.rx   = node->init_tcp_pose_vec_[3];
        obs_req->tcp_pose.ry   = node->init_tcp_pose_vec_[4];
        obs_req->tcp_pose.rz   = node->init_tcp_pose_vec_[5];
        obs_req->tool           = -1;
        obs_req->user           = -1;
        obs_req->velocity       = 30;
        obs_req->acceleration   = 30;
        obs_req->ovl            = 100;
        obs_req->blend_time     = -1.0;   // 阻塞运动
        obs_req->config         = -1;
        obs_req->use_increment  = false;  // 绝对位置

        auto obs_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
            node->L_robot_move_cart_client_, obs_req, node,
            std::chrono::seconds(30), "L/robot_move_cart_observe");

        if (!obs_resp->success) {
            RCLCPP_ERROR(node->get_logger(), "移动到观察位失败: %s", obs_resp->message.c_str());
            rclcpp::shutdown(); spin_thread.join(); return 1;
        }
    }
    RCLCPP_INFO(node->get_logger(), "已到达观察位");

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

    std::vector<double> cola_position;
    bool cola_found = false;

    // 等待检测出现
    rclcpp::sleep_for(std::chrono::seconds(4));
    RCLCPP_INFO(node->get_logger(), "Detected %zu objects", node->getDetectedObjectsCount());

    if (node->hasObject(1)) {
        cola_position = node->getObjectPosition(1);
        if (!cola_position.empty()) {
            RCLCPP_INFO(node->get_logger(), "Cola (ID=1) found at position: [%.3f, %.3f, %.3f]",
                        cola_position[0], cola_position[1], cola_position[2]);
            cola_found = true;
        }
    }

    if (!cola_found) {
        RCLCPP_ERROR(node->get_logger(), "Cola (ID=1) not found! Cannot proceed.");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // 2026-06-10: 不再使用增量计算, 改为 Step 4 中直接计算 MoveCart 绝对目标位姿
    RCLCPP_INFO(node->get_logger(), "Cola position confirmed, proceeding to one-step MoveCart");

    // 禁用物体位置更新
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    node->setObjectUpdateEnabled(false);
    RCLCPP_INFO(node->get_logger(), "Object update disabled for grasping operation");

    // ══════════════════════════════════════════════════════════════════════
    // Step 4: MoveCart PTP 直接到达目标位姿 (2026-06-10: 改用 MoveCart 绕过 servo 路径)
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 4: MoveCart 移动到可乐");

    const double TIP_BASE_DX =  185.64;
    const double TIP_BASE_DY =    1.66;
    const double TIP_BASE_DZ =   -1.82;

    const double TARGET_X  = cola_position[0] * 1000.0 - TIP_BASE_DX;
    const double TARGET_Y  = cola_position[1] * 1000.0 - TIP_BASE_DY;
    const double TARGET_Z  = node->desk_height_ + node->cola_height_ - TIP_BASE_DZ;
    const double TARGET_RX = -90.0;
    const double TARGET_RY = 0.0;
    const double TARGET_RZ = -90.0;

    RCLCPP_INFO(node->get_logger(),
        "目标 TCP:[%.1f, %.1f, %.1f]mm 姿态:[%.1f, %.1f, %.1f]deg",
        TARGET_X, TARGET_Y, TARGET_Z, TARGET_RX, TARGET_RY, TARGET_RZ);

    // AI-Deep: 使用 RobotMoveCart (MoveCart PTP)，阻塞调用
    auto cart_req = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    cart_req->tcp_pose.x    = TARGET_X;
    cart_req->tcp_pose.y    = TARGET_Y;
    cart_req->tcp_pose.z    = TARGET_Z;
    cart_req->tcp_pose.rx   = TARGET_RX;
    cart_req->tcp_pose.ry   = TARGET_RY;
    cart_req->tcp_pose.rz   = TARGET_RZ;
    cart_req->tool           = -1;    // 默认工具
    cart_req->user           = -1;    // 默认工件
    cart_req->velocity       = 30;
    cart_req->acceleration   = 30;
    cart_req->ovl            = 100;
    cart_req->blend_time     = -1.0;  // 阻塞运动
    cart_req->config         = -1;    // 参考当前关节构型
    cart_req->use_increment  = false; // 绝对位置

    auto cart_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, cart_req, node,
        std::chrono::seconds(30), "L/robot_move_cart_cola");

    if (!cart_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "MoveCart 失败: %s", cart_resp->message.c_str());
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "已到达可乐抓取位姿");

    // ══════════════════════════════════════════════════════════════════════
    // Step 5: 合上左夹爪 → 后退离开桌面 (2026-06-10: 原Step 6)
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 5: 夹住可乐并后退");

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

    // 后退离开桌面 (MoveCart 增量)
    {
        auto exit_req = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
        exit_req->tcp_pose.x    = -150;
        exit_req->tcp_pose.y    = 0;
        exit_req->tcp_pose.z    = 30;
        exit_req->tcp_pose.rx   = 0;
        exit_req->tcp_pose.ry   = 0;
        exit_req->tcp_pose.rz   = 0;
        exit_req->tool           = -1;
        exit_req->user           = -1;
        exit_req->velocity       = 30;
        exit_req->acceleration   = 30;
        exit_req->ovl            = 100;
        exit_req->blend_time     = -1.0;
        exit_req->config         = -1;
        exit_req->use_increment  = true;

        auto resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
            node->L_robot_move_cart_client_, exit_req, node, std::chrono::seconds(10),
            "L/robot_move_cart_exit");
        if (!resp->success)
            RCLCPP_WARN(node->get_logger(), "后退失败(非致命): %s", resp->message.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "Robot exited danger zone");

    // ══════════════════════════════════════════════════════════════════════
    // Step 6: 回到初始观察位 (RobotAct 增量)
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 6: 回到初始位");

    {
        // AI-Deep: 使用 RobotMoveCart 绝对位置回到初始 TCP 位姿
        auto ret_req = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
        ret_req->tcp_pose.x    = node->init_tcp_pose_vec_[0];
        ret_req->tcp_pose.y    = node->init_tcp_pose_vec_[1];
        ret_req->tcp_pose.z    = node->init_tcp_pose_vec_[2];
        ret_req->tcp_pose.rx   = node->init_tcp_pose_vec_[3];
        ret_req->tcp_pose.ry   = node->init_tcp_pose_vec_[4];
        ret_req->tcp_pose.rz   = node->init_tcp_pose_vec_[5];
        ret_req->tool           = -1;
        ret_req->user           = -1;
        ret_req->velocity       = 30;
        ret_req->acceleration   = 30;
        ret_req->ovl            = 100;
        ret_req->blend_time     = -1.0;
        ret_req->config         = -1;
        ret_req->use_increment  = false;

        auto resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
            node->L_robot_move_cart_client_, ret_req, node, std::chrono::seconds(30),
            "L/robot_move_cart_init");
        if (!resp->success)
            RCLCPP_WARN(node->get_logger(), "回初始位失败(非致命): %s", resp->message.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "已回到初始位");

    // 重新启用物体检测
    node->setObjectUpdateEnabled(true);

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), " Task1: 识别 → 抓取 完成！");
    RCLCPP_INFO(node->get_logger(), "========================================");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
