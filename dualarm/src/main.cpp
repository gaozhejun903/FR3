#include "dualarm/headers.hpp"
#include "dualarm/service_server_template.hpp"
#include <thread>
#include <future>
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

// inline void wait_until_key_pressed() {
//     // 等待用户按下任意键
//     std::cout << "Press any key to continue..." << std::endl;
//     std::cin.get();
// }

// clang-format off

/*
TODO: #3 写个随便什么模版类把request和response的处理逻辑封装起来，
      这样就可以直接使用模版类来处理请求和响应了。
      例如：
      template<typename RequestT, typename ResponseT>
      class ServiceHandler {
          // ...
      };
      这样就可以在RobotMain类中使用ServiceHandler<robo_ctrl::srv::RobotAct, robo_ctrl::srv::RobotAct::Response>来处理RobotAct服务的请求和响应。
      这样可以减少重复代码，提高代码的可读性和可维护性。
      另外，考虑到可能会有多个服务需要处理，可以使用一个map来存储不同服务的处理器，
      这样就可以根据服务名动态选择处理器进行处理了。
      例如：
      std::map<std::string, std::shared_ptr<ServiceHandlerBase>> service_handlers_;
      这样就可以在RobotMain类的构造函数中注册不同服务的处理器了。
      例如：
      service_handlers_["robot_act"] = std::make_shared<ServiceHandler<robo_ctrl::srv::RobotAct, robo_ctrl::srv::RobotAct::Response>>(this, "robot_act");
      这样就可以在RobotMain类中使用service_handlers_["robot_act"]->handle_request(request)来处理RobotAct服务的请求了。
      这样可以大大简化服务处理的代码逻辑，
      使得RobotMain类的代码更加清晰和易于维护。
*/

/*
TODO: 把一些固定的位置转换成关节的角度。

*/

/*
marble703:
    这个包我编不过，写了扔robo_ctrl包里了，在
    robo_ctrl/include/robo_ctrl/service_server_template.hpp
    能编过，里面看着没问题
 */

// clang-format on

RobotMain::RobotMain(const rclcpp::NodeOptions& options): Node("robot_main", options) {
    // Initialize tf2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize subscribers
    L_robot_state_sub_ = this->create_subscription<robo_ctrl::msg::RobotState>(
        "/L/robot_state",
        10,
        [this](const robo_ctrl::msg::RobotState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
            L_robot_state_ = msg;
            // std::cout << "Received L robot state update" << std::endl;
        }
    );
    L_gripper_status_sub_ = this->create_subscription<epg50_gripper_ros::msg::GripperStatus>(
        "gripper_status",
        10,
        [this](const epg50_gripper_ros::msg::GripperStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(L_gripper_status_mutex_);
            L_gripper_status_ = msg;
        }
    );
    obj_detect_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = obj_detect_cb_group_;

    L_bbox3d_array_sub_ = this->create_subscription<depth_handler::msg::Bbox3dArray>(
        "depth_handler/bbox3d",
        10,
        std::bind(&RobotMain::handleObjDetection, this, std::placeholders::_1),
        sub_options
    );

    // Initialize clients
    L_robot_servo_line_client_ = this->create_client<robo_ctrl::srv::RobotServoLine>(ROBOT_L + "/robot_servo_line");
    L_robot_act_client_        = this->create_client<robo_ctrl::srv::RobotAct>(ROBOT_L + "/robot_act");
    L_robot_move_client_       = this->create_client<robo_ctrl::srv::RobotMove>(ROBOT_L + "/robot_move");
    L_robot_move_cart_client_  = this->create_client<robo_ctrl::srv::RobotMoveCart>(ROBOT_L + "/robot_move_cart");
    L_robot_servo_client_      = this->create_client<robo_ctrl::srv::RobotServo>(ROBOT_L + "/robot_servo");
    L_robot_set_speed_client_  = this->create_client<robo_ctrl::srv::RobotSetSpeed>(ROBOT_L + "/robot_set_speed");
    L_robot_act_j_client_      = this->create_client<robo_ctrl::srv::RobotActJ>(ROBOT_L + "/robot_act_j");
    gripper_command_client_    = this->create_client<epg50_gripper_ros::srv::GripperCommand>("/epg50_gripper/command");

    R_robot_servo_line_client_ = this->create_client<robo_ctrl::srv::RobotServoLine>(ROBOT_R + "/robot_servo_line");
    R_robot_act_client_        = this->create_client<robo_ctrl::srv::RobotAct>(ROBOT_R + "/robot_act");
    R_robot_move_client_       = this->create_client<robo_ctrl::srv::RobotMove>(ROBOT_R + "/robot_move");
    R_robot_move_cart_client_  = this->create_client<robo_ctrl::srv::RobotMoveCart>(ROBOT_R + "/robot_move_cart");
    R_robot_servo_client_      = this->create_client<robo_ctrl::srv::RobotServo>(ROBOT_R + "/robot_servo");
    R_robot_set_speed_client_  = this->create_client<robo_ctrl::srv::RobotSetSpeed>(ROBOT_R + "/robot_set_speed");
    R_robot_act_j_client_      = this->create_client<robo_ctrl::srv::RobotActJ>(ROBOT_R + "/robot_act_j");
    gripper_command_client_    = this->create_client<epg50_gripper_ros::srv::GripperCommand>("/epg50_gripper/command");

    this->declare_parameter("init_tcp_pose", std::vector<double> { 168.0, -102.0, 394.0, -111.556, 0.0, -90.0 });
    this->declare_parameter("open_cap_joint_pose", std::vector<double> { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    this->declare_parameter("desk_height", 190);
    this->declare_parameter("cola_height", 91);
    this->declare_parameter("cestbon_height", 91);
    this->declare_parameter("robot_speed", 0.5);
    init_tcp_pose_vec_ = this->get_parameter("init_tcp_pose").as_double_array();
    desk_height_       = this->get_parameter("desk_height").as_int();
    cola_height_       = this->get_parameter("cola_height").as_int();
    cestbon_height_    = this->get_parameter("cestbon_height").as_int();
    ball_R_joint_init_pose_ = this->get_parameter("ball_R_joint_init_pose").as_double_array();
    ball_L_joint_init_pose_ = this->get_parameter("ball_L_joint_init_pose").as_double_array();
    ball_L_1_joint_pose_    = this->get_parameter("ball_L_1_joint_pose").as_double_array();
    ball_L_2_joint_pose_    = this->get_parameter("ball_L_2_joint_pose").as_double_array();
    ball_R_1_joint_pose_    = this->get_parameter("ball_R_1_joint_pose").as_double_array();
    ball_R_2_joint_pose_    = this->get_parameter("ball_R_2_joint_pose").as_double_array();
    ball_R_per_1_joint_pose_ = this->get_parameter("ball_R_per_1_joint_pose").as_double_array();
    rclcpp::sleep_for(std::chrono::seconds(2));
}
// 定义析构函数，生成 vtable
RobotMain::~RobotMain() {}

std::vector<double> RobotMain::calculateTcpToObjectIncrement(const std::vector<double>& object_position) {
    // 获取当前TCP位置（在基坐标系下）
    robo_ctrl::msg::TCPPose current_tcp_pose;
    {
        std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
        if (L_robot_state_) {
            current_tcp_pose = L_robot_state_->tcp_pose;
        } else {
            RCLCPP_WARN(this->get_logger(), "Robot state not available");
            return { 0.0, 0.0, 0.0 };
        }
    }

    // 计算从当前TCP位置到物体位置的增量（都在基坐标系下）
    std::vector<double> increment = { (object_position[0] * 1000) - current_tcp_pose.x,
                                      (object_position[1] * 1000) - current_tcp_pose.y,
                                      (object_position[2] * 1000) - current_tcp_pose.z };

    RCLCPP_INFO(
        this->get_logger(),
        "Current TCP position (baselink): [%.3f, %.3f, %.3f]",
        current_tcp_pose.x,
        current_tcp_pose.y,
        current_tcp_pose.z
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Object position (baselink): [%.3f, %.3f, %.3f]",
        object_position[0],
        object_position[1],
        object_position[2]
    );

    RCLCPP_INFO(
        this->get_logger(),
        "TCP to object increment: [%.3f, %.3f, %.3f]",
        increment[0],
        increment[1],
        increment[2]
    );

    return increment;
}

void RobotMain::handleObjDetection(const depth_handler::msg::Bbox3dArray::SharedPtr bbox3d_array_msg) {
    // 遮断开关 - 控制是否更新物体位置
    std::async(std::launch::async, [this, bbox3d_array_msg]() {
        // 等待1秒，确保其他订阅者有时间处理
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // 检查更新开关状态（使用线程安全的方法）
        if (!isObjectUpdateEnabled()) {
            RCLCPP_DEBUG(this->get_logger(), "Object detection update is disabled");
            return;
        }

        // 使用互斥锁保护整个更新过程
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);

        // 首先增加所有现有物体的年龄
        for (auto& obj: detected_objects_) {
            obj.age++;
        }

        // 处理检测到的物体
        for (const auto& bbox3d: bbox3d_array_msg->results) {
            // 处理3D边界框
            RCLCPP_INFO(this->get_logger(), "Received bbox3d with class_id: %d", bbox3d.class_id);

            // 提取物体信息
            object_detection new_obj;
            new_obj.id          = bbox3d.class_id;
            new_obj.position    = { bbox3d.x, bbox3d.y, bbox3d.z };
            new_obj.age         = 0; // 新检测到的物体年龄为0
            new_obj.valid_count = 1; // 初始有效计数为1

            // 查找是否已存在相同ID的物体
            bool found_existing = false;

            for (size_t i = 0; i < detected_objects_.size(); ++i) {
                if (detected_objects_[i].id == new_obj.id) {
                    // 计算与已存在物体的距离
                    double distance = std::sqrt(
                        std::pow(detected_objects_[i].position[0] - new_obj.position[0], 2)
                        + std::pow(detected_objects_[i].position[1] - new_obj.position[1], 2)
                        + std::pow(detected_objects_[i].position[2] - new_obj.position[2], 2)
                    );

                    if (distance < distance_threshold_) {
                        // 重置年龄，因为重新检测到了
                        detected_objects_[i].age = 0;
                        detected_objects_[i].valid_count++;

                        // 使用卡尔曼滤波更新位置
                        std::vector<double> filtered_position = kalman_filters_[i].filterPose(new_obj.position);
                        detected_objects_[i].position         = filtered_position;
                        found_existing                        = true;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Updated object %d position: [%.3f, %.3f, %.3f], age: %d, valid_count: %d",
                            new_obj.id,
                            filtered_position[0],
                            filtered_position[1],
                            filtered_position[2],
                            detected_objects_[i].age,
                            detected_objects_[i].valid_count
                        );
                        break;
                    }
                }
            }

            // 如果是新物体，添加到列表中
            if (!found_existing) {
                // 创建新的卡尔曼滤波器
                RobotKalmanFilter new_filter;

                // 初始化滤波器位置
                std::vector<double> filtered_position = new_filter.filterPose(new_obj.position);
                new_obj.position                      = filtered_position;

                // 添加到列表
                detected_objects_.push_back(new_obj);
                kalman_filters_.push_back(new_filter);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Added new object %d at position: [%.3f, %.3f, %.3f], age: %d, valid_count: %d",
                    new_obj.id,
                    filtered_position[0],
                    filtered_position[1],
                    filtered_position[2],
                    new_obj.age,
                    new_obj.valid_count
                );
            }
        }

        // 移除过期的物体（年龄超过阈值的物体）
        for (size_t i = 0; i < detected_objects_.size();) {
            if (detected_objects_[i].age > age_threshold_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Removing expired object %d (age: %d > threshold: %d)",
                    detected_objects_[i].id,
                    detected_objects_[i].age,
                    age_threshold_
                );
                detected_objects_.erase(detected_objects_.begin() + i);
                kalman_filters_.erase(kalman_filters_.begin() + i);
                // 不增加i，因为元素被删除了
            } else {
                ++i;
            }
        }

        // 输出当前检测到的所有物体数量和状态
        RCLCPP_INFO(this->get_logger(), "Total detected objects: %zu", detected_objects_.size());
        for (const auto& obj: detected_objects_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Object %d: age=%d, valid_count=%d, valid=%s",
                obj.id,
                obj.age,
                obj.valid_count,
                obj.is_valid() ? "true" : "false"
            );
        }
    });
}

template<typename ServiceType>
class ServiceCaller {
public:
    using Request  = typename ServiceType::Request;
    using Response = typename ServiceType::Response;
    using Client   = rclcpp::Client<ServiceType>;

    static std::shared_ptr<Response> callServiceSync(
        std::shared_ptr<Client> client,
        std::shared_ptr<Request> request,
        rclcpp::Node::SharedPtr node,
        std::chrono::seconds timeout    = std::chrono::seconds(5),
        const std::string& service_name = "unknown"
    ) {
        if (!client) {
            auto response     = std::make_shared<Response>();
            response->success = false;
            response->message = "Client is null for service: " + service_name;
            RCLCPP_ERROR(node->get_logger(), "Client is null for service: %s", service_name.c_str());
            return response;
        }

        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, future, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Service '%s' succeeded: %s",
                    service_name.c_str(),
                    response->message.c_str()
                );
            } else {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "Service '%s' failed: %s",
                    service_name.c_str(),
                    response->message.c_str()
                );
            }
            return response;
        } else {
            auto response     = std::make_shared<Response>();
            response->success = false;
            response->message = "Service call timeout for: " + service_name;
            RCLCPP_ERROR(node->get_logger(), "Service call timeout for: %s", service_name.c_str());
            return response;
        }
    }
};

/**
 * @brief 机器人服务调用的便捷宏定义
 */
// 原有的宏定义已被上面的ServiceCaller模板替代

/**
 * @brief 增强的等待服务函数
 */
template<typename ClientType>
bool waitForServiceEnhanced(
    std::shared_ptr<ClientType> client,
    rclcpp::Node::SharedPtr node,
    const std::string& service_name,
    std::chrono::seconds timeout = std::chrono::seconds(30)
) {
    RCLCPP_INFO(node->get_logger(), "Waiting for service: %s", service_name.c_str());

    auto start_time = std::chrono::steady_clock::now();
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > timeout) {
            RCLCPP_ERROR(node->get_logger(), "Timeout waiting for service: %s", service_name.c_str());
            return false;
        }

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service: %s", service_name.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(), "Still waiting for service: %s", service_name.c_str());
    }

    RCLCPP_INFO(node->get_logger(), "Service '%s' is available", service_name.c_str());
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executor();
    auto node = std::make_shared<RobotMain>(options);
    RCLCPP_INFO(node->get_logger(), "Waiting for object detection data...");
    auto start_time = node->get_clock()->now();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    }
    // 等待所有必要的服务可用 - 使用增强的等待函数
    RCLCPP_INFO(node->get_logger(), "Waiting for all required services...");

    // 等待机器人移动服务
    if (!waitForServiceEnhanced(node->L_robot_move_client_, node, "L/robot_move")) {
        return 1;
    }

    // 等待机器人操作服务
    if (!waitForServiceEnhanced(node->L_robot_act_client_, node, "L/robot_act")) {
        return 1;
    }

    // 等待机器人笛卡尔移动服务
    if (!waitForServiceEnhanced(node->L_robot_move_cart_client_, node, "L/robot_move_cart")) {
        return 1;
    }

    // 等待夹爪命令服务
    if (!waitForServiceEnhanced(node->gripper_command_client_, node, "gripper_command")) {
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "All required services are now available!");
    // rclcpp::sleep_for(std::chrono::seconds(5));
    // 看向台面 - 使用robot_move绝对 坐标
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
    look_at_table_request->blend_time    = 0.0;   // 不使用混合时间
    look_at_table_request->use_increment = false; // 使用绝对位置
    look_at_table_request->tool          = -1;    // 使用默认工具
    look_at_table_request->user          = -1;    // 使用默认用户
    look_at_table_request->ovl           = 0;

    // auto look_at_table_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     look_at_table_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_move_cart"
    // );

    // auto R_go_to_opencap_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // /*
    // TODO: 找个好点的初始位置
    // */
    // // 等待初始位置移动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(3500))); // 运动时间 + 额外500ms

    // R_go_to_opencap_request->command_type  = 0; // ServoMoveStart
    // R_go_to_opencap_request->target_joints = std::vector<double>({ 85.777, -156.055, 96.643, -157.912, -52.075, 30.0 });
    // R_go_to_opencap_request->point_count   = 100;     // 100个点
    // R_go_to_opencap_request->message_time  = 0.01;    // 0.01s/点
    // R_go_to_opencap_request->use_incremental = false; // 使用绝对位置
    // auto R_go_to_opencap_response            = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_go_to_opencap_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // std::vector<double> orientation_increment;
    // {
    //     std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
    //     if (node->L_robot_state_) {
    //         orientation_increment = { 0,
    //                                   0,
    //                                   0,
    //                                   -90.0 - node->init_tcp_pose_vec_[3],
    //                                   0.0 - node->init_tcp_pose_vec_[4],
    //                                   -90.0 - node->init_tcp_pose_vec_[5] };
    //     }
    // }

    // // 计算总角度差值
    // double total_angle_diff =
    //     std::abs(orientation_increment[3]) + std::abs(orientation_increment[4]) + std::abs(orientation_increment[5]);

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Orientation correction: rx=%.3f, ry=%.3f, rz=%.3f, total_diff=%.3f",
    //     orientation_increment[3],
    //     orientation_increment[4],
    //     orientation_increment[5],
    //     total_angle_diff
    // );
    // // 找到目标物体后，禁用物体位置更新，避免抓取过程中目标位置发生变化
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // node->setObjectUpdateEnabled(false);
    // RCLCPP_INFO(node->get_logger(), "Object update disabled for grasping operation");

    // auto fix_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // node->gen_move_request(fix_request, orientation_increment, true); // 使用增量模式

    // auto fix_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     fix_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L_robot_move_cart_orientation_fix"
    // );

    // if (!fix_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to fix robot orientation: %s", fix_response->message.c_str());
    //     return 1;
    // }

    // // 等待姿态修正完成 - 基于角度大小估算时间
    // int wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0)); // 每度50ms
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");
    // RCLCPP_INFO(node->get_logger(), "Robot moved to initial position successfully using robot_act");
    // // 记录初始位置
    // robo_ctrl::msg::TCPPose start_tcp_pose;
    // {
    //     std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
    //     if (node->L_robot_state_) {
    //         start_tcp_pose = node->L_robot_state_->tcp_pose;
    //     }
    // }

    // rclcpp::sleep_for(std::chrono::microseconds(1500));

    // // wait_until_key_pressed();

    // // 使能夹爪 - 使用简化的服务调用
    // for (auto gripper_id: GRIPPER_LIST) {
    //     auto gripper_enable_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    //     gripper_enable_request->slave_id = gripper_id;
    //     gripper_enable_request->command  = GRIPPER_ENABLE;
    //     gripper_enable_request->position = GRIPPER_OPEN;
    //     gripper_enable_request->speed    = 255;
    //     gripper_enable_request->torque   = 255;

    //     auto gripper_enable_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //         node->gripper_command_client_,
    //         gripper_enable_request,
    //         node,
    //         std::chrono::seconds(5),
    //         "gripper_enable_" + std::to_string(gripper_id)
    //     );

    //     if (!gripper_enable_response->success) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to enable gripper %d", gripper_id);
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Gripper %d enabled successfully", gripper_id);
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // 打开右夹爪 - 使用简化的服务调用
    // auto gripper_open_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_open_request->slave_id = GRIPPER_ID_R;
    // gripper_open_request->command  = GRIPPER_SET;
    // gripper_open_request->position = GRIPPER_OPEN;
    // gripper_open_request->speed    = 255;
    // gripper_open_request->torque   = 255;

    // auto gripper_open_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_open_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "gripper_open_right"
    // );

    // if (!gripper_open_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s", gripper_open_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");

    // // wait_until_key_pressed();

    // // 等待物体检测数据
    // RCLCPP_INFO(node->get_logger(), "Detected %zu objects", node->getDetectedObjectsCount());

    // // 寻找可乐 id:1 (使用线程安全的方法)
    // std::vector<double> tcp_to_cola_increment;
    // std::vector<double> cola_position;
    // bool cola_found = false;

    // // 检查是否找到可乐（使用线程安全的方法）
    // if (node->hasObject(1)) {
    //     cola_position = node->getObjectPosition(1);
    //     if (!cola_position.empty()) {
    //         // 使用计算TCP到可乐的运动增量
    //         RCLCPP_INFO(
    //             node->get_logger(),
    //             "Cola (ID=1) found at position: [%.3f, %.3f, %.3f]",
    //             cola_position[0],
    //             cola_position[1],
    //             cola_position[2]
    //         );
    //         tcp_to_cola_increment = node->calculateTcpToObjectIncrement(cola_position);
    //         cola_found            = true;
    //         RCLCPP_INFO(
    //             node->get_logger(),
    //             "Found cola at position: [%.3f, %.3f, %.3f]",
    //             cola_position[0],
    //             cola_position[1],
    //             cola_position[2]
    //         );
    //         RCLCPP_INFO(
    //             node->get_logger(),
    //             "TCP to cola increment: [%.3f, %.3f, %.3f]",
    //             tcp_to_cola_increment[0],
    //             tcp_to_cola_increment[1],
    //             tcp_to_cola_increment[2]
    //         );
    //     }
    // }

    // // 检查是否找到可乐
    // if (!cola_found) {
    //     RCLCPP_ERROR(node->get_logger(), "Cola (ID=1) not found! Cannot proceed.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // 计算目标TCP位置（增量）
    // std::vector<double> target_tcp_position = { tcp_to_cola_increment[0],
    //                                             tcp_to_cola_increment[1],
    //                                             tcp_to_cola_increment[2] };

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Target TCP position: [%.3f, %.3f, %.3f]",
    //     target_tcp_position[0],
    //     target_tcp_position[1],
    //     target_tcp_position[2]
    // );

    // // 修正rx,ry,rz到-90，0，-90 - 使用robot_move_cart增量模式
    // bool retFlag;
    // int retVal = L_fix(node, orientation_increment, total_angle_diff, fix_request, fix_response, retFlag);
    // if (retFlag)
    //     return retVal;

    // // 等待姿态修正完成 - 基于角度大小估算时间
    // wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0)); // 每度50ms
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");
    // // 更新目标位置,z位置取桌面高度+ cola高度
    // {
    //     std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
    //     if (node->L_robot_state_) {
    //         target_tcp_position[2] = node->desk_height_ + node->cola_height_ - node->L_robot_state_->tcp_pose.z;
    //     }
    // }
    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Corrected Target TCP position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //     target_tcp_position[0],
    //     target_tcp_position[1],
    //     target_tcp_position[2],
    //     target_tcp_position[3],
    //     target_tcp_position[4],
    //     target_tcp_position[5]
    // );
    // RCLCPP_INFO(node->get_logger(), "Moving to cola position...");
    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Target TCP position: [%.3f, %.3f, %.3f]",
    //     target_tcp_position[0],
    //     target_tcp_position[1],
    //     target_tcp_position[2]
    // );
    // // 打开架爪
    // auto gripper_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_request->slave_id = GRIPPER_ID_L;
    // gripper_request->command  = GRIPPER_SET;
    // gripper_request->position = GRIPPER_OPEN;
    // gripper_request->speed    = 255; // 设置速度为255
    // gripper_request->torque   = 255; // 设置扭矩为255
    // auto gripper_future       = node->gripper_command_client_->async_send_request(gripper_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper command");
    //     return 1;
    // }
    // auto gripper_response = gripper_future.get();
    // if (!gripper_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to open gripper");
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // // 移动到可乐位置 - 使用robot_act直线规划
    // auto act_request             = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    // act_request->command_type    = 0; // ServoMoveStart
    // act_request->tcp_pose.x      = target_tcp_position[0] - 132;
    // act_request->tcp_pose.y      = target_tcp_position[1] + 45;
    // act_request->tcp_pose.z      = target_tcp_position[2];
    // act_request->tcp_pose.rx     = 0.0; // 固定姿态
    // act_request->tcp_pose.ry     = 0.0;
    // act_request->tcp_pose.rz     = 0.0;
    // act_request->point_count     = 180;  // 180个点
    // act_request->message_time    = 0.01; // 0.01s/点
    // act_request->plan_type       = 0;    // 直线规划
    // act_request->use_incremental = true; // 使用增量运动

    // auto act_future = node->L_robot_act_client_->async_send_request(act_request);
    // if (rclcpp::spin_until_future_complete(node, act_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for cola position");
    //     return 1;
    // }
    // auto act_response = act_future.get();
    // if (!act_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move robot to cola position: %s", act_response->message.c_str());
    //     return 1;
    // }

    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 3000))); // 运动时间 +
    //                                                                                                     // 额外500ms
    // RCLCPP_INFO(node->get_logger(), "Robot moved to cola position successfully using robot_act");
    // // 关闭夹抓
    // gripper_request->slave_id = GRIPPER_ID_L;
    // gripper_request->command  = GRIPPER_SET;
    // gripper_request->position = GRIPPER_CLOSE;
    // gripper_request->speed    = 255; // 设置速度为255
    // gripper_request->torque   = 255; // 设置扭矩为255

    // gripper_future = node->gripper_command_client_->async_send_request(gripper_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper command");
    //     return 1;
    // }
    // gripper_response = gripper_future.get();
    // if (!gripper_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to closed gripper");
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // // 退出危险区域 - 使用robot_move_cart增量模式
    // auto exit_request           = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // exit_request->tcp_pose.x    = -150; // 向后移动150mm
    // exit_request->tcp_pose.y    = 0.0;  // 不改变y
    // exit_request->tcp_pose.z    = 30;   // 不改变z
    // exit_request->tcp_pose.rx   = 0.0;  // 固定姿态
    // exit_request->tcp_pose.ry   = 0.0;
    // exit_request->tcp_pose.rz   = 0.0;
    // exit_request->acceleration  = 100;
    // exit_request->velocity      = 30;
    // exit_request->config        = -1;   // 使用默认配置
    // exit_request->blend_time    = 0.0;  // 不使用混合
    // exit_request->use_increment = true; // 使用增量运动
    // exit_request->tool          = -1;   // 使用默认工具
    // exit_request->user          = -1;   // 使用默认用户
    // exit_request->ovl           = 0;
    // auto exit_response          = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     exit_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_move_cart"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 1500))); // 运动时间 + 额外500ms

    // //  移动到指定位置，准备打开瓶盖
    // auto goto_opencap_request             = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // goto_opencap_request->command_type    = 0; // ServoMoveStart
    // goto_opencap_request->target_joints   = node->CAP_OPEN_JOINTS_L;
    // goto_opencap_request->point_count     = 100;   // 100个点
    // goto_opencap_request->message_time    = 0.01;  // 0.01s/点
    // goto_opencap_request->use_incremental = false; // 使用绝对位置
    // auto goto_opencap_future              = node->L_robot_act_j_client_->async_send_request(goto_opencap_request);
    // if (rclcpp::spin_until_future_complete(node, goto_opencap_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for goto opencap position");
    //     return 1;
    // }
    // auto goto_opencap_response = goto_opencap_future.get();
    // if (!goto_opencap_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot to opencap position: %s",
    //         goto_opencap_response->message.c_str()
    //     );
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved to opencap position successfully using robot_act");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500))); // 运动时间 +
    //                                                                                                    // 额外500ms

    // // 打开瓶盖 - 初始位置
    // auto open_cap_request             = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // open_cap_request->command_type    = 0; // ServoMoveStart
    // open_cap_request->target_joints   = node->CAP_OPEN_JOINTS_R;
    // open_cap_request->point_count     = 100;   // 100个点
    // open_cap_request->message_time    = 0.008; // 0.01s
    // open_cap_request->use_incremental = false; // 使用绝对位置
    // auto open_cap_future              = node->R_robot_act_j_client_->async_send_request(open_cap_request);
    // if (rclcpp::spin_until_future_complete(node, open_cap_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //     return 1;
    // }
    // auto open_cap_response = open_cap_future.get();
    // if (!open_cap_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot to open cap position: %s",
    //         open_cap_response->message.c_str()
    //     );
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500 + 1500))
    // ); // 运动时间 + 额外2500ms

    // /*
    // TODO: 计算需要多少次来打开瓶盖
    // 目前假设需要5次来打开瓶盖
    // */
    // auto gripper_close_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_close_request->slave_id = GRIPPER_ID_R;  // 右夹
    // gripper_close_request->command  = GRIPPER_SET;   // 设置夹爪
    // gripper_close_request->position = GRIPPER_CLOSE; // 合上夹爪
    // gripper_close_request->speed    = 255;           // 设置速度为255
    // gripper_close_request->torque   = 255;           // 设置扭矩为255

    // auto circle_open_cap_request                   = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    // circle_open_cap_request->command_type          = 0;     // ServoMoveStart
    // circle_open_cap_request->tcp_pose.x            = 0.0;   // 固定位置
    // circle_open_cap_request->tcp_pose.y            = 0.0;   // 固定位置
    // circle_open_cap_request->tcp_pose.z            = 0.0;   // 固定位置
    // circle_open_cap_request->tcp_pose.rx           = 0.0;   // 固定姿
    // circle_open_cap_request->tcp_pose.ry           = 0.0;   // 固定姿态
    // circle_open_cap_request->tcp_pose.rz           = 0.0;   // 固定
    // circle_open_cap_request->point_count           = 200;   // 200个点
    // circle_open_cap_request->message_time          = 0.006; // 0.006s/点
    // circle_open_cap_request->plan_type             = 1;     // 直线规划
    // circle_open_cap_request->use_incremental       = true;  // 使用增量运动
    // circle_open_cap_request->circle_center.x       = 155;   // 圆心位置
    // circle_open_cap_request->circle_center.y       = 0;     // 圆心位置
    // circle_open_cap_request->circle_center.z       = 0;     // 圆心位置
    // circle_open_cap_request->radian                = 60;    // 圆弧度
    // circle_open_cap_request->initial_orientation.x = 0;     // 初始姿态
    // circle_open_cap_request->initial_orientation.y = -1;    // 初始姿态
    // circle_open_cap_request->initial_orientation.z = 0;     // 初始姿态
    // circle_open_cap_request->face_center           = true;  // 面向圆心

    // gripper_open_request           = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_open_request->slave_id = GRIPPER_ID_R; // 右夹
    // gripper_open_request->command  = GRIPPER_SET;  // 设置夹爪
    // gripper_open_request->position = GRIPPER_OPEN; // 打开夹爪
    // gripper_open_request->speed    = 255;          // 设置速度为255
    // gripper_open_request->torque   = 255;          // 设置扭矩为255

    // for (int round = 0; round < 9; round++) {
    //     RCLCPP_INFO(node->get_logger(), "Round %d: Opening cap...", round + 1);

    //     // 合上夹抓
    //     auto gripper_close_future = node->gripper_command_client_->async_send_request(gripper_close_request);
    //     if (rclcpp::spin_until_future_complete(node, gripper_close_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper close");
    //         return 1;
    //     }
    //     auto gripper_close_response = gripper_close_future.get();
    //     if (!gripper_close_response->success) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_response->message.c_str());
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");

    //     // 转瓶盖
    //     auto circle_open_cap_response_future = node->R_robot_act_client_->async_send_request(circle_open_cap_request);
    //     if (rclcpp::spin_until_future_complete(node, circle_open_cap_response_future)
    //         != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //         return 1;
    //     }
    //     auto circle_open_cap_response = circle_open_cap_response_future.get();
    //     if (!circle_open_cap_response->success) {
    //         RCLCPP_ERROR(
    //             node->get_logger(),
    //             "Failed to move robot to open cap position: %s",
    //             circle_open_cap_response->message.c_str()
    //         );
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    //     // 等待运动完成
    //     std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(200 * 0.01 * 1000  + 2000))
    //     ); // 运动时间 + 额外2500ms
    //     RCLCPP_INFO(node->get_logger(), "Round %d: Cap opened successfully", round + 1);

    //     // 打开夹抓
    //     auto gripper_open_future = node->gripper_command_client_->async_send_request(gripper_open_request);
    //     if (rclcpp::spin_until_future_complete(node, gripper_open_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper open");
    //         return 1;
    //     }
    //     auto gripper_open_response = gripper_open_future.get();
    //     if (!gripper_open_response->success) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s", gripper_open_response->message.c_str());
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     // 回到初始位置
    //     auto open_cap_future = node->R_robot_act_j_client_->async_send_request(open_cap_request);
    //     if (rclcpp::spin_until_future_complete(node, open_cap_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //         return 1;
    //     }
    //     auto open_cap_response = open_cap_future.get();
    //     if (!open_cap_response->success) {
    //         RCLCPP_ERROR(
    //             node->get_logger(),
    //             "Failed to move robot to open cap position: %s",
    //             open_cap_response->message.c_str()
    //         );
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    //     // 等待运动完成
    //     std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500 + 1200))
    //     ); // 运动时间 + 额外2500ms
    // }
    // // 最后一次打开瓶盖
    // RCLCPP_INFO(node->get_logger(), "Final round: Opening cap...");
    // // 合上夹抓
    // auto gripper_close_future = node->gripper_command_client_->async_send_request(gripper_close_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_close_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper close");
    //     return 1;
    // }
    // auto gripper_close_response = gripper_close_future.get();
    // if (!gripper_close_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");

    // // 转30度
    // circle_open_cap_request->radian      = 30; // 最后一次只转30
    // auto circle_open_cap_response_future = node->R_robot_act_client_->async_send_request(circle_open_cap_request);
    // if (rclcpp::spin_until_future_complete(node, circle_open_cap_response_future)
    //     != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //     return 1;
    // }
    // auto circle_open_cap_response = circle_open_cap_response_future.get();
    // if (!circle_open_cap_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot to open cap position: %s",
    //         circle_open_cap_response->message.c_str()
    //     );
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(150 * 0.008 * 1000 + 500 + 1200))
    // ); // 运动时间 + 额外2500ms
    // RCLCPP_INFO(node->get_logger(), "Final round: Cap opened successfully");
    // // 打开夹抓
    // auto gripper_open_future = node->gripper_command_client_->async_send_request(gripper_open_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_open_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper open");
    //     return 1;
    // }
    // gripper_open_response = gripper_open_future.get();
    // if (!gripper_open_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s", gripper_open_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // // 合上夹抓
    // gripper_close_future = node->gripper_command_client_->async_send_request(gripper_close_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_close_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper close");
    //     return 1;
    // }
    // gripper_close_response = gripper_close_future.get();
    // if (!gripper_close_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(750));

    // //向上移动30mm
    // auto move_up_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // node->gen_move_request(move_up_request, { 0, 0, 30, 0, 0, 0 }, true); // 使用增量模式
    // auto move_up_future = node->R_robot_move_cart_client_->async_send_request(move_up_request);
    // if (rclcpp::spin_until_future_complete(node, move_up_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_move_cart for move up");
    //     return 1;
    // }
    // auto move_up_response = move_up_future.get();
    // if (!move_up_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move robot up: %s", move_up_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved up successfully using robot_move_cart");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500))); // 运动时间 +
    //                                                                                                    // 额外500ms

    // RCLCPP_INFO(node->get_logger(), "All rounds completed, bottle cap opened successfully!");

    // auto R_act_j_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // node->gen_actJ_request(
    //     R_act_j_request,
    //     std::vector<double> { 85.777, -156.055, 96.643, -157.912, -52.075, 2.212 },
    //     false
    // ); // 使用绝对位置
    // auto R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );

    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // auto R_gripper_open_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_open_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );

    // retFlag;
    // retVal = L_fix(node, orientation_increment, total_angle_diff, fix_request, fix_response, retFlag);
    // if (retFlag)
    //     return retVal;

    // // 等待姿态修正完成 - 基于角度大小估算时间
    // wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0)); // 每度50ms
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");

    // R_act_j_request->target_joints = std::vector<double> { 108.607, -30.389, 90.947, -240.608, -138.361, 0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");

    // R_act_j_request->target_joints = std::vector<double> { 99.506, -27.672, 84.605, -234.074, -126.131, 0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(4600));

    // auto R_gripper_close_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_close_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // R_act_j_request->target_joints = std::vector<double> { 119.072, -100.935, 144.690, -208.412, -100.500, 0.744 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // // 开始倒可乐
    // RCLCPP_INFO(node->get_logger(), "Starting to pour cola...");
    // auto pour_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // node->gen_actJ_request(
    //     pour_request,
    //     std::vector<double> { 0, 0, 0, 0, 0, 60 }, // 增量模式，旋转30度
    //     true
    // ); // 使用增量模式
    // pour_request->point_count  = 70;
    // pour_request->message_time = 0.06; // 0.1s/点
    // auto pour_response         = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->L_robot_act_j_client_,
    //     pour_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(70 * 0.06 * 1000 + 5000)));

    // pour_request->point_count   = 10;
    // pour_request->message_time  = 0.06;
    // pour_request->target_joints = std::vector<double> { 0, 0, 0, 0, 0, 30 }; // 增量模式，旋转30度
    // pour_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->L_robot_act_j_client_,
    //     pour_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(30 * 0.08 * 1000 + 1500)));

    // RCLCPP_INFO(node->get_logger(), "Cola poured successfully!");
    // pour_request->target_joints = std::vector<double> { 0, 0, 0, 0, 0, -90 }; // 增量模式，旋转-72度
    // pour_request->point_count   = 30;
    // pour_request->message_time  = 0.03;
    // pour_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->L_robot_act_j_client_,
    //     pour_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(30 * 0.03 * 1000 + 2000)));

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // R_act_j_request->target_joints = std::vector<double> { 91.399, -39.548, 107.352, -241.810, -100.444, 0.737 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    // // 下移10mm松开杯子

    // RCLCPP_INFO(node->get_logger(), "Robot returned to initial position after pouring cola");
    // R_gripper_open_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_open_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(700));

    // auto R_act_request             = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    // R_act_request->command_type    = 0;   // ServoMoveStart
    // R_act_request->tcp_pose.x      = 0.0; // 不改变x
    // R_act_request->tcp_pose.y      = 0.0; // 不改变y
    // R_act_request->tcp_pose.z      = -10; // 向下移动10
    // R_act_request->tcp_pose.rx     = 0.0; // 固定姿
    // R_act_request->tcp_pose.ry     = 0.0;
    // R_act_request->tcp_pose.rz     = 0.0;  // 固定
    // R_act_request->point_count     = 15;   // 15个点
    // R_act_request->message_time    = 0.01; // 0.01s/点
    // R_act_request->plan_type       = 0;    // 直线规划
    // R_act_request->use_incremental = true; // 使用增量运动
    // auto R_act_response            = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
    //     node->R_robot_act_client_,
    //     R_act_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act"
    // );
    // if (!R_act_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot down after pouring cola: %s",
    //         R_act_response->message.c_str()
    //     );
    //     return 1;
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(15 * 0.01 * 1000 + 700))); // 运动时间 +
    //                                                                                                    // 额外500ms

    // R_act_j_request->target_joints = std::vector<double> { 91.695, -36.073, 123.177, -264.383, -100.583, 0.737 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );

    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // R_go_to_opencap_request->command_type  = 0; // ServoMoveStart
    // R_go_to_opencap_request->target_joints = std::vector<double>({ 85.777, -156.055, 96.643, -157.912, -52.075, 50.0 });
    // R_go_to_opencap_request->point_count   = 100;     // 100个点
    // R_go_to_opencap_request->message_time  = 0.01;    // 0.01s/点
    // R_go_to_opencap_request->use_incremental = false; // 使用绝对位置
    // R_go_to_opencap_response                 = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_go_to_opencap_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "putting cola back to: [%.3f, %.3f, %.3f]",
    //     cola_position[0] * 1000 - 135,
    //     cola_position[1] * 1000 + 50,
    //     cola_position[2] * 1000 + 10
    // );

    // act_request->tcp_pose.x      = cola_position[0] * 1000 - 135;                // 向前移动135mm
    // act_request->tcp_pose.y      = cola_position[1] * 1000 + 5;                 //
    // act_request->tcp_pose.z      = node->desk_height_ + node->cola_height_ - 15; // 向上移动10mm
    // act_request->tcp_pose.rx     = -90.0;                                        // 固定姿态
    // act_request->tcp_pose.ry     = 0.0;
    // act_request->tcp_pose.rz     = -90.0;
    // act_request->point_count     = 180;   // 180个点
    // act_request->message_time    = 0.01;  // 0.01s/点
    // act_request->plan_type       = 0;     // 直线规划
    // act_request->use_incremental = false; // 使用增量运动

    // act_future = node->L_robot_act_client_->async_send_request(act_request);
    // if (rclcpp::spin_until_future_complete(node, act_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for cola position");
    //     return 1;
    // }
    // act_response = act_future.get();
    // if (!act_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move robot to cola position: %s", act_response->message.c_str());
    //     return 1;
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1800 + 3000))); // 运动时间 + 额外500ms

    // gripper_request->slave_id = GRIPPER_ID_L;
    // gripper_request->command  = GRIPPER_SET;
    // gripper_request->position = GRIPPER_OPEN;
    // gripper_request->speed    = 255; // 设置速度为255
    // gripper_request->torque   = 255; // 设置扭矩为255
    // gripper_response          = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );

    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 3200))); // 运动时间 +
    //                                                                                                     // 额外500ms

    // exit_request                = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // exit_request->tcp_pose.x    = -40; // 向后移动40mm
    // exit_request->tcp_pose.y    = 0.0; // 不改变y
    // exit_request->tcp_pose.z    = 0;   // 不改变z
    // exit_request->tcp_pose.rx   = 0.0; // 固定姿态
    // exit_request->tcp_pose.ry   = 0.0;
    // exit_request->tcp_pose.rz   = 0.0;
    // exit_request->acceleration  = 100;
    // exit_request->velocity      = 30;
    // exit_request->config        = -1;   // 使用默认配置
    // exit_request->blend_time    = 0.0;  // 不使用混合
    // exit_request->use_increment = true; // 使用增量运动
    // exit_request->tool          = -1;   // 使用默认工具
    // exit_request->user          = -1;   // 使用默认用户
    // exit_request->ovl           = 0;
    // exit_response               = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     exit_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_move_cart"
    // );

    // /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // // open_castbon();
    // ████████╗  ██╗   ██╗  █████╗  ██╗       █████╗  ██████╗  ███╗   ███╗
    // ██╔══  ██║ ██║   ██║ ██╔══██╗ ██║      ██╔══██╗ ██╔══██╗ ████╗ ████║
    // ██║    ██║ ██║   ██║ ███████║ ██║      ███████║ ██████╔╝ ██╔████╔██║
    // ██║    ██║ ██║   ██║ ██╔══██║ ██║      ██╔══██║ ██╔══██╗ ██║╚██╔╝██║
    // ████████║  ╚██████╔╝ ██║  ██║ ███████╗ ██║  ██║ ██║  ██║ ██║ ╚═╝ ██║
    // ╚═══════╝   ╚═════╝  ╚═╝  ╚═╝ ╚══════╝ ╚═╝  ╚═╝ ╚═╝  ╚═╝ ╚═╝     ╚═╝
    //     f. u. c. k. i. n. g.
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //     // /
    // */
    // // 开启识别
    // node->setObjectUpdateEnabled(true);
    // RCLCPP_INFO(node->get_logger(), "Object update enabled for next operations");

    // rclcpp::sleep_for(std::chrono::seconds(2));
    // // 看向台面 - 使用robot_move绝对 坐标
    // look_at_table_request                = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // look_at_table_request->tcp_pose.x    = node->init_tcp_pose_vec_[0];
    // look_at_table_request->tcp_pose.y    = node->init_tcp_pose_vec_[1];
    // look_at_table_request->tcp_pose.z    = node->init_tcp_pose_vec_[2];
    // look_at_table_request->tcp_pose.rx   = node->init_tcp_pose_vec_[3];
    // look_at_table_request->tcp_pose.ry   = node->init_tcp_pose_vec_[4];
    // look_at_table_request->tcp_pose.rz   = node->init_tcp_pose_vec_[5];
    // look_at_table_request->acceleration  = 100;
    // look_at_table_request->velocity      = 100;
    // look_at_table_request->config        = -1;
    // look_at_table_request->blend_time    = 0.0;   // 不使用混合时间
    // look_at_table_request->use_increment = false; // 使用绝对位置
    // look_at_table_request->tool          = -1;    // 使用默认工具
    // look_at_table_request->user          = -1;    // 使用默认用户
    // look_at_table_request->ovl           = 0;

    // look_at_table_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     look_at_table_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_move_cart"
    // );

    // R_go_to_opencap_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // /*
    // TODO: 找个好点的初始位置
    // */
    // // 等待初始位置移动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(3200))); // 运动时间 + 额外500ms

    // R_go_to_opencap_request->command_type = 0; // ServoMoveStart
    // R_go_to_opencap_request->target_joints =
    //     std::vector<double>({ 85.777, -156.055, 96.643, -157.912, -52.075, -24.846 });
    // R_go_to_opencap_request->point_count     = 100;   // 100个点
    // R_go_to_opencap_request->message_time    = 0.01;  // 0.01s/点
    // R_go_to_opencap_request->use_incremental = false; // 使用绝对位置
    // R_go_to_opencap_response                 = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_go_to_opencap_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(1300));

    // orientation_increment;
    // {
    //     std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
    //     if (node->L_robot_state_) {
    //         orientation_increment = { 0,
    //                                   0,
    //                                   0,
    //                                   -90.0 - node->init_tcp_pose_vec_[3],
    //                                   0.0 - node->init_tcp_pose_vec_[4],
    //                                   -90.0 - node->init_tcp_pose_vec_[5] };
    //     }
    // }

    // // 计算总角度差值
    // total_angle_diff =
    //     std::abs(orientation_increment[3]) + std::abs(orientation_increment[4]) + std::abs(orientation_increment[5]);

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Orientation correction: rx=%.3f, ry=%.3f, rz=%.3f, total_diff=%.3f",
    //     orientation_increment[3],
    //     orientation_increment[4],
    //     orientation_increment[5],
    //     total_angle_diff
    // );
    // // 找到目标物体后，禁用物体位置更新，避免抓取过程中目标位置发生变化
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // node->setObjectUpdateEnabled(false);
    // RCLCPP_INFO(node->get_logger(), "Object update disabled for grasping operation");

    // fix_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // node->gen_move_request(fix_request, orientation_increment, true); // 使用增量模式

    // fix_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     fix_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L_robot_move_cart_orientation_fix"
    // );

    // if (!fix_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to fix robot orientation: %s", fix_response->message.c_str());
    //     return 1;
    // }

    // // 等待姿态修正完成 - 基于角度大小估算时间
    // wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0)); // 每度50ms
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");
    // RCLCPP_INFO(node->get_logger(), "Robot moved to initial position successfully using robot_act");
    // // 记录初始位置
    // start_tcp_pose;
    // {
    //     std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
    //     if (node->L_robot_state_) {
    //         start_tcp_pose = node->L_robot_state_->tcp_pose;
    //     }
    // }

    // rclcpp::sleep_for(std::chrono::milliseconds(1200));

    // // wait_until_key_pressed();

    // // 使能夹爪 - 使用简化的服务调用
    // for (auto gripper_id: GRIPPER_LIST) {
    //     auto gripper_enable_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    //     gripper_enable_request->slave_id = gripper_id;
    //     gripper_enable_request->command  = GRIPPER_ENABLE;
    //     gripper_enable_request->position = GRIPPER_OPEN;
    //     gripper_enable_request->speed    = 255;
    //     gripper_enable_request->torque   = 255;

    //     auto gripper_enable_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //         node->gripper_command_client_,
    //         gripper_enable_request,
    //         node,
    //         std::chrono::seconds(5),
    //         "gripper_enable_" + std::to_string(gripper_id)
    //     );

    //     if (!gripper_enable_response->success) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to enable gripper %d", gripper_id);
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Gripper %d enabled successfully", gripper_id);
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(750));

    // // 打开右夹爪 - 使用简化的服务调用
    // gripper_open_request           = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_open_request->slave_id = GRIPPER_ID_R;
    // gripper_open_request->command  = GRIPPER_SET;
    // gripper_open_request->position = GRIPPER_OPEN;
    // gripper_open_request->speed    = 255;
    // gripper_open_request->torque   = 255;

    // gripper_open_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_open_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "gripper_open_right"
    // );

    // if (!gripper_open_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s", gripper_open_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");

    // // wait_until_key_pressed();

    // // 等待物体检测数据
    // RCLCPP_INFO(node->get_logger(), "Detected %zu objects", node->getDetectedObjectsCount());

    // // 寻找可乐 id:1 (使用线程安全的方法)
    // tcp_to_cola_increment;
    // cola_position;
    // cola_found = false;

    // // 检查是否找到可乐（使用线程安全的方法）
    // if (node->hasObject(0)) {
    //     cola_position = node->getObjectPosition(0);
    //     if (!cola_position.empty()) {
    //         // 使用计算TCP到可乐的运动增量
    //         tcp_to_cola_increment = node->calculateTcpToObjectIncrement(cola_position);
    //         cola_found            = true;
    //         RCLCPP_INFO(
    //             node->get_logger(),
    //             "Found cola at position: [%.3f, %.3f, %.3f]",
    //             cola_position[0],
    //             cola_position[1],
    //             cola_position[2]
    //         );
    //         RCLCPP_INFO(
    //             node->get_logger(),
    //             "TCP to cola increment: [%.3f, %.3f, %.3f]",
    //             tcp_to_cola_increment[0],
    //             tcp_to_cola_increment[1],
    //             tcp_to_cola_increment[2]
    //         );
    //     }
    // }

    // // 检查是否找到可乐
    // if (!cola_found) {
    //     RCLCPP_ERROR(node->get_logger(), "Cola (ID=1) not found! Cannot proceed.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // 计算目标TCP位置（增量）
    // target_tcp_position = { tcp_to_cola_increment[0], tcp_to_cola_increment[1], tcp_to_cola_increment[2] };

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Target TCP position: [%.3f, %.3f, %.3f]",
    //     target_tcp_position[0],
    //     target_tcp_position[1],
    //     target_tcp_position[2]
    // );

    // // 修正rx,ry,rz到-90，0，-90 - 使用robot_move_cart增量模式
    // retVal = L_fix(node, orientation_increment, total_angle_diff, fix_request, fix_response, retFlag);
    // if (retFlag)
    //     return retVal;

    // // 等待姿态修正完成 - 基于角度大小估算时间
    // wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0)); // 每度50ms
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");
    // // 更新目标位置,z位置取桌面高度+ cola高度
    // {
    //     std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
    //     if (node->L_robot_state_) {
    //         target_tcp_position[2] = node->desk_height_ + node->cola_height_ - node->L_robot_state_->tcp_pose.z;
    //     }
    // }
    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Corrected Target TCP position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //     target_tcp_position[0],
    //     target_tcp_position[1],
    //     target_tcp_position[2],
    //     target_tcp_position[3],
    //     target_tcp_position[4],
    //     target_tcp_position[5]
    // );
    // RCLCPP_INFO(node->get_logger(), "Moving to cola position...");
    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "Target TCP position: [%.3f, %.3f, %.3f]",
    //     target_tcp_position[0],
    //     target_tcp_position[1],
    //     target_tcp_position[2]
    // );
    // // 打开架爪
    // gripper_request           = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_request->slave_id = GRIPPER_ID_L;
    // gripper_request->command  = GRIPPER_SET;
    // gripper_request->position = GRIPPER_OPEN;
    // gripper_request->speed    = 255; // 设置速度为255
    // gripper_request->torque   = 255; // 设置扭矩为255
    // gripper_future            = node->gripper_command_client_->async_send_request(gripper_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper command");
    //     return 1;
    // }
    // gripper_response = gripper_future.get();
    // if (!gripper_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to open gripper");
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // // 移动到可乐位置 - 使用robot_act直线规划
    // act_request                  = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    // act_request->command_type    = 0; // ServoMoveStart
    // act_request->tcp_pose.x      = target_tcp_position[0] - 135;
    // act_request->tcp_pose.y      = target_tcp_position[1] + 25;
    // act_request->tcp_pose.z      = target_tcp_position[2];
    // act_request->tcp_pose.rx     = 0.0; // 固定姿态
    // act_request->tcp_pose.ry     = 0.0;
    // act_request->tcp_pose.rz     = 0.0;
    // act_request->point_count     = 180;  // 180个点
    // act_request->message_time    = 0.01; // 0.01s/点
    // act_request->plan_type       = 0;    // 直线规划
    // act_request->use_incremental = true; // 使用增量运动

    // act_future = node->L_robot_act_client_->async_send_request(act_request);
    // if (rclcpp::spin_until_future_complete(node, act_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for cola position");
    //     return 1;
    // }
    // act_response = act_future.get();
    // if (!act_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move robot to cola position: %s", act_response->message.c_str());
    //     return 1;
    // }

    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 4700))); // 运动时间 +
    //                                                                                                     // 额外500ms
    // RCLCPP_INFO(node->get_logger(), "Robot moved to cola position successfully using robot_act");
    // // 关闭夹抓
    // gripper_request->slave_id = GRIPPER_ID_L;
    // gripper_request->command  = GRIPPER_SET;
    // gripper_request->position = GRIPPER_CLOSE;
    // gripper_request->speed    = 255; // 设置速度为255
    // gripper_request->torque   = 255; // 设置扭矩为255

    // gripper_future = node->gripper_command_client_->async_send_request(gripper_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper command");
    //     return 1;
    // }
    // gripper_response = gripper_future.get();
    // if (!gripper_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to closed gripper");
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // // 退出危险区域 - 使用robot_move_cart增量模式
    // exit_request                = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // exit_request->tcp_pose.x    = -50; // 向后移动50mm
    // exit_request->tcp_pose.y    = 0.0;  // 不改变y
    // exit_request->tcp_pose.z    = 30;   // 不改变z
    // exit_request->tcp_pose.rx   = 0.0;  // 固定姿态
    // exit_request->tcp_pose.ry   = 0.0;
    // exit_request->tcp_pose.rz   = 0.0;
    // exit_request->acceleration  = 100;
    // exit_request->velocity      = 30;
    // exit_request->config        = -1;   // 使用默认配置
    // exit_request->blend_time    = 0.0;  // 不使用混合
    // exit_request->use_increment = true; // 使用增量运动
    // exit_request->tool          = -1;   // 使用默认工具
    // exit_request->user          = -1;   // 使用默认用户
    // exit_request->ovl           = 0;
    // exit_response               = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     exit_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_move_cart"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 1800))); // 运动时间 + 额外500ms

    // //  移动到指定位置，准备打开瓶盖
    // goto_opencap_request                  = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // goto_opencap_request->command_type    = 0; // ServoMoveStart
    // goto_opencap_request->target_joints   = node->CAP_OPEN_JOINTS_L;
    // goto_opencap_request->point_count     = 100;   // 100个点
    // goto_opencap_request->message_time    = 0.01;  // 0.01s/点
    // goto_opencap_request->use_incremental = false; // 使用绝对位置
    // goto_opencap_future                   = node->L_robot_act_j_client_->async_send_request(goto_opencap_request);
    // if (rclcpp::spin_until_future_complete(node, goto_opencap_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for goto opencap position");
    //     return 1;
    // }
    // goto_opencap_response = goto_opencap_future.get();
    // if (!goto_opencap_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot to opencap position: %s",
    //         goto_opencap_response->message.c_str()
    //     );
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved to opencap position successfully using robot_act");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500))); // 运动时间 +
    //                                                                                                    // 额外500ms

    // // 打开瓶盖 - 初始位置
    // open_cap_request                  = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // open_cap_request->command_type    = 0; // ServoMoveStart
    // open_cap_request->target_joints   = node->castbon_CAP_OPEN_JOINTS_R;
    // open_cap_request->point_count     = 100;   // 100个点
    // open_cap_request->message_time    = 0.008; // 0.01s
    // open_cap_request->use_incremental = false; // 使用绝对位置
    // open_cap_future                   = node->R_robot_act_j_client_->async_send_request(open_cap_request);
    // if (rclcpp::spin_until_future_complete(node, open_cap_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //     return 1;
    // }
    // open_cap_response = open_cap_future.get();
    // if (!open_cap_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot to open cap position: %s",
    //         open_cap_response->message.c_str()
    //     );
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500 + 1500))
    // ); // 运动时间 + 额外2500ms

    // /*
    // TODO: 计算需要多少次来打开瓶盖
    // 目前假设需要5次来打开瓶盖
    // */
    // gripper_close_request           = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_close_request->slave_id = GRIPPER_ID_R;  // 右夹
    // gripper_close_request->command  = GRIPPER_SET;   // 设置夹爪
    // gripper_close_request->position = GRIPPER_CLOSE; // 合上夹爪
    // gripper_close_request->speed    = 255;           // 设置速度为255
    // gripper_close_request->torque   = 255;           // 设置扭矩为255

    // circle_open_cap_request                        = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    // circle_open_cap_request->command_type          = 0;     // ServoMoveStart
    // circle_open_cap_request->tcp_pose.x            = 0.0;   // 固定位置
    // circle_open_cap_request->tcp_pose.y            = 0.0;   // 固定位置
    // circle_open_cap_request->tcp_pose.z            = 0.0;   // 固定位置
    // circle_open_cap_request->tcp_pose.rx           = 0.0;   // 固定姿
    // circle_open_cap_request->tcp_pose.ry           = 0.0;   // 固定姿态
    // circle_open_cap_request->tcp_pose.rz           = 0.0;   // 固定
    // circle_open_cap_request->point_count           = 200;   // 200个点
    // circle_open_cap_request->message_time          = 0.006; // 0.006s/点
    // circle_open_cap_request->plan_type             = 1;     // 直线规划
    // circle_open_cap_request->use_incremental       = true;  // 使用增量运动
    // circle_open_cap_request->circle_center.x       = 155;   // 圆心位置
    // circle_open_cap_request->circle_center.y       = 0;     // 圆心位置
    // circle_open_cap_request->circle_center.z       = 0;     // 圆心位置
    // circle_open_cap_request->radian                = 60;    // 圆弧度
    // circle_open_cap_request->initial_orientation.x = 0;     // 初始姿态
    // circle_open_cap_request->initial_orientation.y = -1;    // 初始姿态
    // circle_open_cap_request->initial_orientation.z = 0;     // 初始姿态
    // circle_open_cap_request->face_center           = true;  // 面向圆心

    // gripper_open_request           = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    // gripper_open_request->slave_id = GRIPPER_ID_R; // 右夹
    // gripper_open_request->command  = GRIPPER_SET;  // 设置夹爪
    // gripper_open_request->position = GRIPPER_OPEN; // 打开夹爪
    // gripper_open_request->speed    = 255;          // 设置速度为255
    // gripper_open_request->torque   = 255;          // 设置扭矩为255

    // for (int round = 0; round < 3; round++) {
    //     RCLCPP_INFO(node->get_logger(), "Round %d: Opening cap...", round + 1);

    //     // 合上夹抓
    //     auto gripper_close_future = node->gripper_command_client_->async_send_request(gripper_close_request);
    //     if (rclcpp::spin_until_future_complete(node, gripper_close_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper close");
    //         return 1;
    //     }
    //     auto gripper_close_response = gripper_close_future.get();
    //     if (!gripper_close_response->success) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_response->message.c_str());
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");

    //     // 转瓶盖
    //     auto circle_open_cap_response_future = node->R_robot_act_client_->async_send_request(circle_open_cap_request);
    //     if (rclcpp::spin_until_future_complete(node, circle_open_cap_response_future)
    //         != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //         return 1;
    //     }
    //     auto circle_open_cap_response = circle_open_cap_response_future.get();
    //     if (!circle_open_cap_response->success) {
    //         RCLCPP_ERROR(
    //             node->get_logger(),
    //             "Failed to move robot to open cap position: %s",
    //             circle_open_cap_response->message.c_str()
    //         );
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    //     // 等待运动完成
    //     std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(200 * 0.01 * 1000 + 500 + 2200))
    //     ); // 运动时间 + 额外2500ms
    //     RCLCPP_INFO(node->get_logger(), "Round %d: Cap opened successfully", round + 1);

    //     // 打开夹抓
    //     auto gripper_open_future = node->gripper_command_client_->async_send_request(gripper_open_request);
    //     if (rclcpp::spin_until_future_complete(node, gripper_open_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper open");
    //         return 1;
    //     }
    //     auto gripper_open_response = gripper_open_future.get();
    //     if (!gripper_open_response->success) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s", gripper_open_response->message.c_str());
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     // 回到初始位置
    //     auto open_cap_future = node->R_robot_act_j_client_->async_send_request(open_cap_request);
    //     if (rclcpp::spin_until_future_complete(node, open_cap_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //         return 1;
    //     }
    //     auto open_cap_response = open_cap_future.get();
    //     if (!open_cap_response->success) {
    //         RCLCPP_ERROR(
    //             node->get_logger(),
    //             "Failed to move robot to open cap position: %s",
    //             open_cap_response->message.c_str()
    //         );
    //         return 1;
    //     }
    //     RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    //     // 等待运动完成
    //     std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500 + 1500))
    //     ); // 运动时间 + 额外2500ms
    // }
    // // 最后一次打开瓶盖
    // RCLCPP_INFO(node->get_logger(), "Final round: Opening cap...");
    // // 合上夹抓
    // gripper_close_future = node->gripper_command_client_->async_send_request(gripper_close_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_close_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper close");
    //     return 1;
    // }
    // gripper_close_response = gripper_close_future.get();
    // if (!gripper_close_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");

    // // 转30度
    // circle_open_cap_request->radian = 30; // 最后一次只转30
    // circle_open_cap_response_future = node->R_robot_act_client_->async_send_request(circle_open_cap_request);
    // if (rclcpp::spin_until_future_complete(node, circle_open_cap_response_future)
    //     != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for open cap position");
    //     return 1;
    // }
    // circle_open_cap_response = circle_open_cap_response_future.get();
    // if (!circle_open_cap_response->success) {
    //     RCLCPP_ERROR(
    //         node->get_logger(),
    //         "Failed to move robot to open cap position: %s",
    //         circle_open_cap_response->message.c_str()
    //     );
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved to open cap position successfully using robot_act");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(150 * 0.008 * 1000 + 500 + 1500))
    // ); // 运动时间 + 额外2500ms
    // RCLCPP_INFO(node->get_logger(), "Final round: Cap opened successfully");
    // // 打开夹抓
    // gripper_open_future = node->gripper_command_client_->async_send_request(gripper_open_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_open_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper open");
    //     return 1;
    // }
    // gripper_open_response = gripper_open_future.get();
    // if (!gripper_open_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to open gripper: %s", gripper_open_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper opened successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // // 合上夹抓
    // gripper_close_future = node->gripper_command_client_->async_send_request(gripper_close_request);
    // if (rclcpp::spin_until_future_complete(node, gripper_close_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper close");
    //     return 1;
    // }
    // gripper_close_response = gripper_close_future.get();
    // if (!gripper_close_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Gripper closed successfully");
    // std::this_thread::sleep_for(std::chrono::milliseconds(750));

    // //向上移动30mm
    // move_up_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // node->gen_move_request(move_up_request, { 0, 0, 30, 0, 0, 0 }, true); // 使用增量模式
    // move_up_future = node->R_robot_move_cart_client_->async_send_request(move_up_request);
    // if (rclcpp::spin_until_future_complete(node, move_up_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_move_cart for move up");
    //     return 1;
    // }
    // move_up_response = move_up_future.get();
    // if (!move_up_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move robot up: %s", move_up_response->message.c_str());
    //     return 1;
    // }
    // RCLCPP_INFO(node->get_logger(), "Robot moved up successfully using robot_move_cart");
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100 * 0.01 * 1000 + 500))); // 运动时间 +
    //                                                                                                    // 额外500ms

    // RCLCPP_INFO(node->get_logger(), "All rounds completed, bottle cap opened successfully!");

    // R_act_j_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // node->gen_actJ_request(
    //     R_act_j_request,
    //     std::vector<double> { 85.777, -156.055, 96.643, -157.912, -52.075, 2.212 },
    //     false
    // ); // 使用绝对位置
    // R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );

    // std::this_thread::sleep_for(std::chrono::seconds(1));

    // R_gripper_open_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_open_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );

    // retVal = L_fix(node, orientation_increment, total_angle_diff, fix_request, fix_response, retFlag);
    // if (retFlag)
    //     return retVal;

    // // 等待姿态修正完成 - 基于角度大小估算时间
    // wait_time_ms = static_cast<int>(std::max(1000.0, total_angle_diff * 50.0)); // 每度50ms
    // std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");

    // // 拿杯子

    // R_act_j_request->target_joints = std::vector<double> { 66.277, -98.980, 143.351, -221.254, -84.855, 0.0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // R_act_j_request->target_joints = std::vector<double> { 64.469, -55.465, 135.943, -246.132, -75.673, 0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // RCLCPP_INFO(node->get_logger(), "Robot orientation fixed successfully using robot_move_cart");

    // R_act_j_request->target_joints = std::vector<double> { 65.462, -39.377, 114.429, -249.593, -75.604, 0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(5700));

    // R_gripper_close_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_close_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // R_act_j_request->target_joints = std::vector<double> { 66.277, -98.980, 143.351, -221.254, -84.855, 0.0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(3200));

    // R_act_j_request->target_joints = std::vector<double> { 52.820, -131.190, 118.217, -69.251, -21.944, -100.506 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // // 开始倒可乐

    // RCLCPP_INFO(node->get_logger(), "Starting to pour cola...");
    // pour_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    // node->gen_actJ_request(
    //     pour_request,
    //     std::vector<double> { 0, 0, 0, 0, 0, 30 }, 
    //     true
    // ); // 使用增量模式
    // pour_request->point_count  = 70;
    // pour_request->message_time = 0.03; // 0.1s/点
    // pour_response              = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->L_robot_act_j_client_,
    //     pour_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(70 * 0.06 * 1000 + 3000)));

    // pour_request->point_count   = 30;
    // pour_request->message_time  = 0.03;
    // pour_request->target_joints = std::vector<double> { 0, 0, 0, 0, 0, 57 }; 
    // pour_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->L_robot_act_j_client_,
    //     pour_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(30 * 0.08 * 1000 + 500)));

    // RCLCPP_INFO(node->get_logger(), "Cola poured successfully!");
    // pour_request->target_joints = std::vector<double> { 0, 0, 0, 0, 0, -87 }; // 增量模式，旋转-80度
    // pour_request->point_count   = 30;
    // pour_request->message_time  = 0.03;
    // pour_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->L_robot_act_j_client_,
    //     pour_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(30 * 0.03 * 1000 + 5000)));

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // R_act_j_request->target_joints = std::vector<double> { 66.277, -98.980, 143.351, -221.254, -84.855, 0.0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // R_act_j_request->target_joints = std::vector<double> { 70.263, -44.729, 116.790, -242.467, -84.812, 0.0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    // RCLCPP_INFO(node->get_logger(), "Robot returned to initial position after pouring cola");
    // R_gripper_open_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_open_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // R_act_j_request->target_joints = std::vector<double> { 69.742, -58.387, 132.074, -245.621, -84.943, 0.0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );

    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // R_act_j_request->target_joints = std::vector<double> { 66.277, -98.980, 143.351, -221.254, -84.855, 0.0 };
    // R_act_j_response               = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_act_j_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(3700));

    // R_go_to_opencap_request->command_type  = 0; // ServoMoveStart
    // R_go_to_opencap_request->target_joints = std::vector<double>({ 85.777, -156.055, 96.643, -157.912, -52.075, 30.0 });
    // R_go_to_opencap_request->point_count   = 100; // 100个点 R_go_to_opencap_request->message_time  = 0.01;
    // // 0.01s/点 R_go_to_opencap_request->use_incremental = false; // 使用绝对位置 R_go_to_opencap_response =
    // ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
    //     node->R_robot_act_j_client_,
    //     R_go_to_opencap_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "R/robot_act_j"
    // );
    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // act_request->tcp_pose.x      = node->init_tcp_pose_vec_[0];
    // act_request->tcp_pose.y      = node->init_tcp_pose_vec_[1];
    // act_request->tcp_pose.z      = node->init_tcp_pose_vec_[2];
    // act_request->tcp_pose.rx     = -90.0; // 固定姿态
    // act_request->tcp_pose.ry     = 0.0;
    // act_request->tcp_pose.rz     = -90.0;
    // act_request->point_count     = 180;   // 180个点
    // act_request->message_time    = 0.01;  // 0.01
    // act_request->plan_type       = 0;     // 直线规划
    // act_request->use_incremental = false; // 使用绝对位置
    // auto act_response_1 = ServiceCaller<robo_ctrl::srv::RobotAct>::callServiceSync(
    //     node->L_robot_act_client_,
    //     act_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_act"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 1000))); // 运动时间 +
    //                                                                                                     // 额外700ms

    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "putting cola back to: [%.3f, %.3f, %.3f]",
    //     cola_position[0] * 1000 - 135,
    //     cola_position[1] * 1000 + 50,
    //     cola_position[2] * 1000 + 10
    // );

    // act_request->tcp_pose.x      = cola_position[0] * 1000 - 135;              // 向前移动135mm
    // act_request->tcp_pose.y      = cola_position[1] * 1000 + 10;               //
    // act_request->tcp_pose.z      = node->desk_height_ + node->cestbon_height_- 10; // 向上移动10mm
    // act_request->tcp_pose.rx     = -90.0;                                      // 固定姿态
    // act_request->tcp_pose.ry     = 0.0;
    // act_request->tcp_pose.rz     = -90.0;
    // act_request->point_count     = 180;   // 180个点
    // act_request->message_time    = 0.01;  // 0.01s/点
    // act_request->plan_type       = 0;     // 直线规划
    // act_request->use_incremental = false; // 使用增量运动

    // act_future = node->L_robot_act_client_->async_send_request(act_request);
    // if (rclcpp::spin_until_future_complete(node, act_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for cola position");
    //     return 1;
    // }
    // act_response = act_future.get();
    // if (!act_response->success) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move robot to cola position: %s", act_response->message.c_str());
    //     return 1;
    // }

    // // 等待运动完成
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(180 * 0.01 * 1000 + 3700))); // 运动时间 +
    //                                                                                                     // 额外500ms
    // gripper_request->slave_id = GRIPPER_ID_L;
    // gripper_request->command  = GRIPPER_SET;
    // gripper_request->position = GRIPPER_OPEN;
    // gripper_request->speed    = 255; // 设置速度为255
    // gripper_request->torque   = 255; // 设置扭矩为255
    // gripper_response          = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    //     node->gripper_command_client_,
    //     gripper_request,
    //     node,
    //     std::chrono::seconds(5),
    //     "/epg50_gripper/command"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(750));

    // exit_request                = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    // exit_request->tcp_pose.x    = -80; // 向后移动80mm
    // exit_request->tcp_pose.y    = 0.0; // 不改变y
    // exit_request->tcp_pose.z    = 30;  // 不改变z
    // exit_request->tcp_pose.rx   = 0.0; // 固定姿态
    // exit_request->tcp_pose.ry   = 0.0;
    // exit_request->tcp_pose.rz   = 0.0;
    // exit_request->acceleration  = 100;
    // exit_request->velocity      = 30;
    // exit_request->config        = -1;   // 使用默认配置
    // exit_request->blend_time    = 0.0;  // 不使用混合
    // exit_request->use_increment = true; // 使用增量运动
    // exit_request->tool          = -1;   // 使用默认工具
    // exit_request->user          = -1;   // 使用默认用户
    // exit_request->ovl           = 0;
    // exit_response               = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //     node->L_robot_move_cart_client_,
    //     exit_request,
    //     node,
    //     std::chrono::seconds(10),
    //     "L/robot_move_cart"
    // );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 1700))); // 运动时间 + 额外500ms

    // RCLCPP_INFO(node->get_logger(), "Object update enabled for next operations");

    // rclcpp::sleep_for(std::chrono::seconds(5));
    // // 看向台面 - 使用robot_move绝对 坐标
    look_at_table_request                = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    look_at_table_request->tcp_pose.x    = node->init_tcp_pose_vec_[0];
    look_at_table_request->tcp_pose.y    = node->init_tcp_pose_vec_[1];
    look_at_table_request->tcp_pose.z    = node->init_tcp_pose_vec_[2];
    look_at_table_request->tcp_pose.rx   = node->init_tcp_pose_vec_[3];
    look_at_table_request->tcp_pose.ry   = node->init_tcp_pose_vec_[4];
    look_at_table_request->tcp_pose.rz   = node->init_tcp_pose_vec_[5];
    look_at_table_request->acceleration  = 100;
    look_at_table_request->velocity      = 100;
    look_at_table_request->config        = -1;
    look_at_table_request->blend_time    = 0.0;   // 不使用混合时间
    look_at_table_request->use_increment = false; // 使用绝对位置
    look_at_table_request->tool          = -1;    // 使用默认工具
    look_at_table_request->user          = -1;    // 使用默认用户
    look_at_table_request->ovl           = 0;

    auto look_at_table_response_ball = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_,
        look_at_table_request,
        node,
        std::chrono::seconds(10),
        "L/robot_move_cart"
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(2000))); // 运动时间 + 额外500ms

    /*
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // get_ball();
    ████████╗  ██╗   ██╗  █████╗  ██╗       █████╗  ██████╗  ███╗   ███╗
    ██╔══  ██║ ██║   ██║ ██╔══██╗ ██║      ██╔══██╗ ██╔══██╗ ████╗ ████║
    ██║    ██║ ██║   ██║ ███████║ ██║      ███████║ ██████╔╝ ██╔████╔██║
    ██║    ██║ ██║   ██║ ██╔══██║ ██║      ██╔══██║ ██╔══██╗ ██║╚██╔╝██║
    ████████║  ╚██████╔╝ ██║  ██║ ███████╗ ██║  ██║ ██║  ██║ ██║ ╚═╝ ██║
    ╚═══════╝   ╚═════╝  ╚═╝  ╚═╝ ╚══════╝ ╚═╝  ╚═╝ ╚═╝  ╚═╝ ╚═╝     ╚═╝
        // f. u. c. k.  b. a. l. l.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    */

    // 等待初始位置移动完成
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    auto R_act_j_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    auto L_act_j_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    auto R_act_request = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    auto L_act_request = std::make_shared<robo_ctrl::srv::RobotAct::Request>();

    // L goto init pose
    L_act_j_request->target_joints = node->ball_L_joint_init_pose_;
    L_act_j_request->point_count = 100; // 100个点
    L_act_j_request->message_time = 0.01; // 0.01s
    L_act_j_request->use_incremental = false; // 使用绝对位置
    auto L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "L robot moved to initial position");
    // R goto init pose
    R_act_j_request->target_joints = node->ball_R_joint_init_pose_;
    R_act_j_request->point_count = 100; // 100个点
    R_act_j_request->message_time = 0.01; // 0.01
    R_act_j_request->use_incremental = false; // 使用绝对位置
    auto R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 1000))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "R robot moved to initial position");

    //R robot move to ball 1 position
    R_act_j_request->target_joints = node->ball_R_per_1_joint_pose_;
    R_act_j_request->point_count = 100; // 100个点
    R_act_j_request->message_time = 0.01; // 0.01
    R_act_j_request->use_incremental = false; // 使用绝对位置
    R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "R robot moved to ball per 1 position");
    //R robot move to ball 1 position
    R_act_j_request->target_joints = node->ball_R_1_joint_pose_;
    R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "R robot moved to ball 1 position");

        // L robot move to ball 1 position
        L_act_j_request->target_joints = node->ball_L_1_joint_pose_;
    L_act_j_request->point_count = 100; // 100个点
    L_act_j_request->message_time = 0.01; // 0.01
    L_act_j_request->use_incremental = false; // 使用绝对位置
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "L robot moved to ball 1 position");

    L_act_j_request->target_joints   = std::vector<double> { 0, 0, 0, 0, -10, 0 }; // L robot 抓取球1
    L_act_j_request->use_incremental = true;
    L_act_j_response                 = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "L robot released ball 1");

    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 等待松手

    // 松手
    L_act_j_request->target_joints = std::vector<double> { 0, 0, 0, 0, 10, 0 }; // L robot 松手
    L_act_j_request->use_incremental = true; 
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "L robot released ball 1");

    // R robot move back to per 1 position
    R_act_j_request->target_joints = node->ball_R_per_1_joint_pose_;
    R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "R robot moved back to ball per 1 position");
    //L & R robot move back to initial position
    L_act_j_request->target_joints = node->ball_L_joint_init_pose_;
    L_act_j_request->use_incremental = false; // 使用绝对位置
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    R_act_j_request->target_joints = node->ball_R_joint_init_pose_;
    R_act_j_request->use_incremental = false; // 使用绝对位置
    R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 1000))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "L & R robots moved back to initial positions");

    // R robot move to ball 2 position
    R_act_j_request->target_joints   = node->ball_R_2_joint_pose_;
    R_act_j_request->point_count     = 100;   // 100个点
    R_act_j_request->message_time    = 0.01;  // 0.01
    R_act_j_request->use_incremental = false; // 使用绝对位置
    R_act_j_response                 = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "R robot moved to ball 2 position");

    // L robot move to ball 2 position
    L_act_j_request->target_joints = node->ball_L_2_joint_pose_;
    L_act_j_request->point_count = 100; // 100个点
    L_act_j_request->message_time = 0.01; // 0.01
    L_act_j_request->use_incremental = false; // 使用绝对位置
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    L_act_j_request->target_joints   = std::vector<double> { 0, 0, 0, 0, -10, 0 };        // L robot 抓取球1
    L_act_j_request->use_incremental = true;
    L_act_j_response                 = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    RCLCPP_INFO(node->get_logger(), "L robot moved to ball 2 position");


    //L&R open hand
    L_act_j_request->target_joints = std::vector<double> { 0, 0, 0, 0, 16, 0 }; // L robot 松手
    L_act_j_request->use_incremental = true;
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    R_act_j_request->target_joints = std::vector<double> { 0, 0, 0, 0, 8, 0 }; // R robot 松手
    R_act_j_request->use_incremental = true;
    R_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->R_robot_act_j_client_,
        R_act_j_request,
        node,
        std::chrono::seconds(10),
        "R/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 + 750))); // 运动时间 + 额外500ms
    RCLCPP_INFO(node->get_logger(), "L robot released ball 2");





    rclcpp::shutdown();
    return 0;
}

void open_castbon() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Open castbon function called");
}

int L_fix(
    std::shared_ptr<RobotMain>& node,
    std::vector<double, std::allocator<double>>& orientation_increment,
    double& total_angle_diff,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart_Request>& fix_request,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart_Response>& fix_response,
    bool& retFlag
) {
    retFlag = true;
    {
        std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
        if (node->L_robot_state_) {
            orientation_increment = { 0,
                                      0,
                                      0,
                                      -90.0 - node->L_robot_state_->tcp_pose.rx,
                                      0.0 - node->L_robot_state_->tcp_pose.ry,
                                      -90.0 - node->L_robot_state_->tcp_pose.rz };
        }
    }

    // 计算总角度差值
    total_angle_diff =
        std::abs(orientation_increment[3]) + std::abs(orientation_increment[4]) + std::abs(orientation_increment[5]);

    RCLCPP_INFO(
        node->get_logger(),
        "Orientation correction: rx=%.3f, ry=%.3f, rz=%.3f, total_diff=%.3f",
        orientation_increment[3],
        orientation_increment[4],
        orientation_increment[5],
        total_angle_diff
    );

    fix_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(fix_request, orientation_increment, true); // 使用增量模式
    fix_request->velocity = 90;

    auto fix_future = node->L_robot_move_cart_client_->async_send_request(fix_request);
    if (rclcpp::spin_until_future_complete(node, fix_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_move_cart for fixing orientation");
        return 1;
    }
    fix_response = fix_future.get();
    if (!fix_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to fix robot orientation: %s", fix_response->message.c_str());
        return 1;
    }
    retFlag = false;
    return {};
}

// f. u. c. k.  a. l. l. 


// Q.E.D.  