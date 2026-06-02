/*
AI-Deep:
    把 main.cpp 里被注释掉的开瓶盖+倒水完整流程抽出来，
    单独编成 task1 可执行文件。
    流程：识别可乐→左臂抓取→左右臂到开瓶盖位→右臂拧瓶盖→
          右臂退开→左臂倒水→放回瓶子→回初始位。
    使用 ServiceCaller 模板同步调用，与原代码风格一致。
 */
#include "dualarm/headers.hpp"
#include "dualarm/service_server_template.hpp"
#include <thread>
#include "main.h"

#define GRIPPER_ID_L    9
#define GRIPPER_ID_R    10
#define GRIPPER_DISABLE 0
#define GRIPPER_ENABLE  1
#define GRIPPER_SET     2
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   255

#define GRIPPER_LIST \
    { GRIPPER_ID_L, GRIPPER_ID_R }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMain>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() { executor.spin(); });

    // ══════════════════════════════════════════════════════════════════════
    // Step 1: 使能夹爪
    // ══════════════════════════════════════════════════════════════════════
    // AI-Deep: 左右爪分节点后，分别使能
    RCLCPP_INFO(node->get_logger(), "Step 1: 使能夹爪");

    // 使能左爪 (L_gripper_node)
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

    // 使能右爪 (R_gripper_node)
    {
        auto req      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
        req->slave_id = GRIPPER_ID_R;
        req->command  = GRIPPER_ENABLE;
        req->position = GRIPPER_OPEN;
        req->speed    = 255;
        req->torque   = 255;
        auto resp     = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, req, node,
            std::chrono::seconds(5), "gripper_enable_R");
        if (!resp->success) {
            RCLCPP_ERROR(node->get_logger(), "Failed to enable right gripper");
            rclcpp::shutdown(); spin_thread.join(); return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Right gripper enabled");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ══════════════════════════════════════════════════════════════════════
    // Step 2: 打开右夹爪
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 2: 打开右夹爪");

    auto gripper_open_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_open_request->slave_id = GRIPPER_ID_R;
    gripper_open_request->command  = GRIPPER_SET;
    gripper_open_request->position = GRIPPER_OPEN;
    gripper_open_request->speed    = 255;
    gripper_open_request->torque   = 255;

    auto gripper_open_response =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_open_request, node,
            std::chrono::seconds(5), "gripper_open_right");

    if (!gripper_open_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s",
                     gripper_open_response->message.c_str());
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
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
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
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
        rclcpp::shutdown();
        spin_thread.join();
        return retVal;
    }

    int wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully");

    // 更新目标位置，Z 取桌面高度 + 可乐高度
    {
        std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
        if (node->L_robot_state_) {
            target_tcp_position[2] =
                node->desk_height_ + node->cola_height_ - node->L_robot_state_->tcp_pose.z;
        }
    }
    RCLCPP_INFO(node->get_logger(),
                "Corrected Target TCP position: [%.3f, %.3f, %.3f]",
                target_tcp_position[0], target_tcp_position[1], target_tcp_position[2]);

    // ══════════════════════════════════════════════════════════════════════
    // Step 5: 左臂打开夹爪 → 移动到可乐位置
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 5: 移动到可乐位置");

    // 打开左夹爪
    auto gripper_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_request->slave_id = GRIPPER_ID_L;
    gripper_request->command  = GRIPPER_SET;
    gripper_request->position = GRIPPER_OPEN;
    gripper_request->speed    = 255;
    gripper_request->torque   = 255;
    auto gripper_response     = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_request, node, std::chrono::seconds(5),
        "/epg50_gripper/command");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 用 RobotAct 直线规划移动
    auto act_request             = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    act_request->command_type    = 0; // ServoMoveStart
    act_request->tcp_pose.x      = target_tcp_position[0] - 132;
    act_request->tcp_pose.y      = target_tcp_position[1] + 45;
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
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 3000)));
    RCLCPP_INFO(node->get_logger(), "Robot moved to cola position successfully");

    // ══════════════════════════════════════════════════════════════════════
    // Step 6: 合上左夹爪 → 后退离开桌面
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 6: 夹住可乐并后退");

    gripper_request->slave_id = GRIPPER_ID_L;
    gripper_request->command  = GRIPPER_SET;
    gripper_request->position = GRIPPER_CLOSE;
    gripper_request->speed    = 255;
    gripper_request->torque   = 255;
    gripper_response          = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
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
    exit_request->velocity      = 30;
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
    // Step 7: 左臂移动到开瓶盖位 CAP_OPEN_JOINTS_L
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 7: 左臂→开瓶盖位");

    auto goto_opencap_request             = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    goto_opencap_request->command_type    = 0;
    goto_opencap_request->target_joints   = node->CAP_OPEN_JOINTS_L;
    goto_opencap_request->point_count     = 100;
    goto_opencap_request->message_time    = 0.01;
    goto_opencap_request->use_incremental = false;

    auto goto_opencap_response =
        ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
            node->L_robot_act_j_client_, goto_opencap_request, node,
            std::chrono::seconds(10), "L/robot_act_j_opencap");

    if (!goto_opencap_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move L to opencap position: %s",
                     goto_opencap_response->message.c_str());
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500)));
    RCLCPP_INFO(node->get_logger(), "L moved to opencap position");

    // ══════════════════════════════════════════════════════════════════════
    // Step 8: 右臂移动到开瓶盖位 CAP_OPEN_JOINTS_R
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 8: 右臂→开瓶盖位");

    auto open_cap_request             = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    open_cap_request->command_type    = 0;
    open_cap_request->target_joints   = node->CAP_OPEN_JOINTS_R;
    open_cap_request->point_count     = 100;
    open_cap_request->message_time    = 0.008;
    open_cap_request->use_incremental = false;

    auto open_cap_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, open_cap_request, node, std::chrono::seconds(10),
        "R/robot_act_j_opencap");

    if (!open_cap_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move R to opencap position: %s",
                     open_cap_response->message.c_str());
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500 + 1500)));
    RCLCPP_INFO(node->get_logger(), "R moved to opencap position");

    // ══════════════════════════════════════════════════════════════════════
    // Step 9: 拧瓶盖循环（9轮 + 最后一轮30°）
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 9: 拧瓶盖");

    // 准备请求（不变部分）
    auto gripper_close_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_close_request->slave_id = GRIPPER_ID_R;
    gripper_close_request->command  = GRIPPER_SET;
    gripper_close_request->position = GRIPPER_CLOSE;
    gripper_close_request->speed    = 255;
    gripper_close_request->torque   = 255;

    auto gripper_open_request_r      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_open_request_r->slave_id = GRIPPER_ID_R;
    gripper_open_request_r->command  = GRIPPER_SET;
    gripper_open_request_r->position = GRIPPER_OPEN;
    gripper_open_request_r->speed    = 255;
    gripper_open_request_r->torque   = 255;

    auto circle_request                   = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    circle_request->command_type          = 0;
    circle_request->tcp_pose.x            = 0.0;
    circle_request->tcp_pose.y            = 0.0;
    circle_request->tcp_pose.z            = 0.0;
    circle_request->tcp_pose.rx           = 0.0;
    circle_request->tcp_pose.ry           = 0.0;
    circle_request->tcp_pose.rz           = 0.0;
    circle_request->point_count           = 200;
    circle_request->message_time          = 0.006;
    circle_request->plan_type             = 1;    // 圆弧规划
    circle_request->use_incremental       = true;
    circle_request->circle_center.x       = 155;
    circle_request->circle_center.y       = 0;
    circle_request->circle_center.z       = 0;
    circle_request->radian                = 60;
    circle_request->initial_orientation.x = 0;
    circle_request->initial_orientation.y = -1;
    circle_request->initial_orientation.z = 0;
    circle_request->face_center           = true;

    for (int round = 0; round < 9; round++) {
        RCLCPP_INFO(node->get_logger(), "Round %d: Opening cap...", round + 1);

        // 合上右夹爪
        auto gripper_close_response =
            ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
                node->R_gripper_command_client_, gripper_close_request, node,
                std::chrono::seconds(5), "gripper_close_R");
        if (!gripper_close_response->success) {
            RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s",
                         gripper_close_response->message.c_str());
            break;
        }

        // 圆弧旋转
        auto circle_response = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
            node->R_robot_act_client_, circle_request, node,
            std::chrono::seconds(10), "R/robot_act_circle");
        if (!circle_response->success) {
            RCLCPP_ERROR(node->get_logger(), "Failed to rotate: %s",
                         circle_response->message.c_str());
            break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(200 * 0.006 * 1000 + 2000)));
        RCLCPP_INFO(node->get_logger(), "Round %d: rotated", round + 1);

        // 打开右夹爪
        auto gripper_open_r_resp =
            ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
                node->R_gripper_command_client_, gripper_open_request_r, node,
                std::chrono::seconds(5), "gripper_open_R");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 右臂回到初始拧瓶盖位
        auto open_cap_resp = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
            node->R_robot_act_j_client_, open_cap_request, node,
            std::chrono::seconds(10), "R/robot_act_j_opencap");
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500 + 1200)));
    }

    // 最后一轮：只转30°
    RCLCPP_INFO(node->get_logger(), "Final round: Opening cap...");

    auto gripper_close_response =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_close_request, node,
            std::chrono::seconds(5), "gripper_close_R_final");
    if (!gripper_close_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to close gripper final: %s",
                     gripper_close_response->message.c_str());
    }

    circle_request->radian = 30;
    auto circle_response   = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
        node->R_robot_act_client_, circle_request, node, std::chrono::seconds(10),
        "R/robot_act_circle_final");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(150 * 0.008 * 1000 + 500 + 1200)));
    RCLCPP_INFO(node->get_logger(), "Final round: Cap opened successfully");

    // 打开右夹爪
    auto gripper_open_r_resp =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_open_request_r, node,
            std::chrono::seconds(5), "gripper_open_R_final");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 合上右夹爪（抓住瓶盖）
    gripper_close_response =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_close_request, node,
            std::chrono::seconds(5), "gripper_close_R_grab");
    std::this_thread::sleep_for(std::chrono::milliseconds(750));

    // 向上移动 30mm
    auto move_up_request           = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    move_up_request->tcp_pose.x    = 0;
    move_up_request->tcp_pose.y    = 0;
    move_up_request->tcp_pose.z    = 30;
    move_up_request->tcp_pose.rx   = 0.0;
    move_up_request->tcp_pose.ry   = 0.0;
    move_up_request->tcp_pose.rz   = 0.0;
    move_up_request->acceleration  = 100;
    move_up_request->velocity      = 100;
    move_up_request->blend_time    = -1;
    move_up_request->config        = -1;
    move_up_request->use_increment = true;
    move_up_request->tool          = 0;
    move_up_request->user          = 0;
    move_up_request->ovl           = 100;

    auto move_up_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->R_robot_move_cart_client_, move_up_request, node,
        std::chrono::seconds(10), "R/robot_move_cart_up");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500)));
    RCLCPP_INFO(node->get_logger(), "R moved up 30mm");

    // ══════════════════════════════════════════════════════════════════════
    // Step 10: 右臂过渡位移开
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 10: 右臂过渡位");

    auto R_act_j_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    node->gen_actJ_request(
        R_act_j_request,
        std::vector<double>{85.777, -156.055, 96.643, -157.912, -52.075, 2.212}, false);
    auto R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, R_act_j_request, node, std::chrono::seconds(10),
        "R/robot_act_j_trans1");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 打开右夹爪丢瓶盖
    gripper_open_r_resp =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_open_request_r, node,
            std::chrono::seconds(5), "gripper_open_R_drop");

    // 修正左臂姿态
    retVal = L_fix(node, orientation_increment, total_angle_diff,
                   fix_request, fix_response, retFlag);
    if (retFlag) {
        rclcpp::shutdown();
        spin_thread.join();
        return retVal;
    }
    wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    RCLCPP_INFO(node->get_logger(), "L orientation fixed after dropping cap");

    // 右臂过渡位2
    R_act_j_request->target_joints = {108.607, -30.389, 90.947, -240.608, -138.361, 0};
    R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, R_act_j_request, node, std::chrono::seconds(10),
        "R/robot_act_j_trans2");
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));

    // 右臂过渡位3
    R_act_j_request->target_joints = {99.506, -27.672, 84.605, -234.074, -126.131, 0};
    R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, R_act_j_request, node, std::chrono::seconds(10),
        "R/robot_act_j_trans3");
    std::this_thread::sleep_for(std::chrono::milliseconds(4600));

    // 合上右夹爪
    gripper_close_response =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_close_request, node,
            std::chrono::seconds(5), "gripper_close_R_pre_pour");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 右臂过渡位4
    R_act_j_request->target_joints = {119.072, -100.935, 144.690, -208.412, -100.500, 0.744};
    R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, R_act_j_request, node, std::chrono::seconds(10),
        "R/robot_act_j_trans4");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // ══════════════════════════════════════════════════════════════════════
    // Step 11: 左臂倒可乐
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 11: 倒可乐");

    auto pour_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    node->gen_actJ_request(pour_request, std::vector<double>{0, 0, 0, 0, 0, 60}, true);
    pour_request->point_count  = 70;
    pour_request->message_time = 0.06;
    auto pour_response         = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_, pour_request, node, std::chrono::seconds(10),
        "L/robot_act_j_pour1");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(70 * 0.06 * 1000 + 5000)));

    pour_request->point_count   = 10;
    pour_request->message_time  = 0.06;
    pour_request->target_joints = std::vector<double>{0, 0, 0, 0, 0, 30};
    pour_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_, pour_request, node, std::chrono::seconds(10),
        "L/robot_act_j_pour2");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(30 * 0.08 * 1000 + 1500)));

    RCLCPP_INFO(node->get_logger(), "Cola poured successfully!");

    pour_request->target_joints = std::vector<double>{0, 0, 0, 0, 0, -90};
    pour_request->point_count   = 30;
    pour_request->message_time  = 0.03;
    pour_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_, pour_request, node, std::chrono::seconds(10),
        "L/robot_act_j_pour3");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(30 * 0.03 * 1000 + 2000)));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ══════════════════════════════════════════════════════════════════════
    // Step 12: 右臂最终收拢
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 12: 右臂最终收拢");

    R_act_j_request->target_joints = {91.399, -39.548, 107.352, -241.810, -100.444, 0.737};
    R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, R_act_j_request, node, std::chrono::seconds(10),
        "R/robot_act_j_retreat1");
    std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    // 下移10mm 松开杯子
    RCLCPP_INFO(node->get_logger(), "Robot returned to initial position after pouring cola");
    gripper_open_r_resp =
        ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
            node->R_gripper_command_client_, gripper_open_request_r, node,
            std::chrono::seconds(5), "gripper_open_R_pour_done");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));

    auto R_act_request          = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    R_act_request->command_type = 0;
    R_act_request->tcp_pose.x   = 0.0;
    R_act_request->tcp_pose.y   = 0.0;
    R_act_request->tcp_pose.z   = -10;
    R_act_request->tcp_pose.rx  = 0.0;
    R_act_request->tcp_pose.ry  = 0.0;
    R_act_request->tcp_pose.rz  = 0.0;
    R_act_request->point_count  = 15;
    R_act_request->message_time = 0.01;
    R_act_request->plan_type    = 0;
    R_act_request->use_incremental = true;
    auto R_act_response         = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
        node->R_robot_act_client_, R_act_request, node, std::chrono::seconds(10),
        "R/robot_act_down");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(15 * 0.01 * 1000 + 700)));

    R_act_j_request->target_joints = {91.695, -36.073, 123.177, -264.383, -100.583, 0.737};
    R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_, R_act_j_request, node, std::chrono::seconds(10),
        "R/robot_act_j_retreat2");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 右臂回到瓶盖位附近
    auto R_go_to_opencap_request             = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    R_go_to_opencap_request->command_type    = 0;
    R_go_to_opencap_request->target_joints   = {85.777, -156.055, 96.643, -157.912, -52.075, 50.0};
    R_go_to_opencap_request->point_count     = 100;
    R_go_to_opencap_request->message_time    = 0.01;
    R_go_to_opencap_request->use_incremental = false;
    auto R_go_to_opencap_response =
        ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
            node->R_robot_act_j_client_, R_go_to_opencap_request, node,
            std::chrono::seconds(10), "R/robot_act_j_back");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ══════════════════════════════════════════════════════════════════════
    // Step 13: 左臂放回可乐
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 13: 放回可乐");

    RCLCPP_INFO(node->get_logger(), "putting cola back to: [%.3f, %.3f, %.3f]",
                cola_position[0] * 1000 - 135, cola_position[1] * 1000 + 5,
                cola_position[2] * 1000 + 10);

    act_request->tcp_pose.x      = cola_position[0] * 1000 - 135;
    act_request->tcp_pose.y      = cola_position[1] * 1000 + 5;
    act_request->tcp_pose.z      = node->desk_height_ + node->cola_height_ - 15;
    act_request->tcp_pose.rx     = -90.0;
    act_request->tcp_pose.ry     = 0.0;
    act_request->tcp_pose.rz     = -90.0;
    act_request->point_count     = 180;
    act_request->message_time    = 0.01;
    act_request->plan_type       = 0;
    act_request->use_incremental = false;

    act_response = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
        node->L_robot_act_client_, act_request, node, std::chrono::seconds(10),
        "L/robot_act_putback");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1800 + 3000)));

    // 松开左夹爪
    gripper_request->slave_id = GRIPPER_ID_L;
    gripper_request->command  = GRIPPER_SET;
    gripper_request->position = GRIPPER_OPEN;
    gripper_request->speed    = 255;
    gripper_request->torque   = 255;
    gripper_response          = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_request, node, std::chrono::seconds(5),
        "/epg50_gripper/command");

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 3200)));

    // 后退
    exit_request->tcp_pose.x    = -40;
    exit_request->tcp_pose.y    = 0.0;
    exit_request->tcp_pose.z    = 0;
    exit_request->velocity      = 30;
    exit_request->use_increment = true;
    exit_response               = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, exit_request, node, std::chrono::seconds(10),
        "L/robot_move_cart_back");

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000 + 1700)));

    // 重新启用物体检测
    node->setObjectUpdateEnabled(true);
    RCLCPP_INFO(node->get_logger(), "Object update enabled for next operations");

    // ══════════════════════════════════════════════════════════════════════
    // Step 14: 回到初始观察位
    // ══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(node->get_logger(), "Step 14: 回到初始位");

    rclcpp::sleep_for(std::chrono::seconds(2));

    auto look_at_table_request           = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    look_at_table_request->tcp_pose.x    = node->init_tcp_pose_vec_[0];
    look_at_table_request->tcp_pose.y    = node->init_tcp_pose_vec_[1];
    look_at_table_request->tcp_pose.z    = node->init_tcp_pose_vec_[2];
    look_at_table_request->tcp_pose.rx   = node->init_tcp_pose_vec_[3];
    look_at_table_request->tcp_pose.ry   = node->init_tcp_pose_vec_[4];
    look_at_table_request->tcp_pose.rz   = node->init_tcp_pose_vec_[5];
    look_at_table_request->acceleration  = 100;
    look_at_table_request->velocity      = 100;
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

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), " Task1: 开瓶盖 + 倒水 完成！");
    RCLCPP_INFO(node->get_logger(), "========================================");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
