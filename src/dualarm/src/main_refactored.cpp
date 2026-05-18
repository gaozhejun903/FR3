#include "dualarm/headers.hpp"
#include "dualarm/service_server_template.hpp" // 使用正确的路径

#define GRIPPER_ID_L    9
#define GRIPPER_ID_R    10
#define GRIPPER_DISABLE 0
#define GRIPPER_ENABLE  1
#define GRIPPER_SET     2
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   255

#define GRIPPER_LIST \
    { GRIPPER_ID_L, GRIPPER_ID_R }

/**
 * @brief 使用ServiceServerTemplate简化的服务处理器管理类
 */
class ServiceManager {
public:
    using RobotMoveHandler      = ServiceServerTemplate<robo_ctrl::srv::RobotMove>;
    using RobotMoveCartHandler  = ServiceServerTemplate<robo_ctrl::srv::RobotMoveCart>;
    using RobotActHandler       = ServiceServerTemplate<robo_ctrl::srv::RobotAct>;
    using RobotActJHandler      = ServiceServerTemplate<robo_ctrl::srv::RobotActJ>;
    using GripperCommandHandler = ServiceServerTemplate<epg50_gripper_ros::srv::GripperCommand>;

    explicit ServiceManager(rclcpp::Node* parent_node): parent_node_(parent_node) {}

    /**
     * @brief 创建并启动所有服务处理器
     */
    void initialize() {
        setupLeftRobotServices();
        setupRightRobotServices();
        setupGripperServices();

        RCLCPP_INFO(
            parent_node_->get_logger(),
            "Service Manager initialized with %zu service handlers",
            service_handlers_.size()
        );
    }

    /**
     * @brief 获取统计信息
     */
    // void printStatistics() {
    //     for (const auto& [name, handler]: service_handlers_) {
    //         auto stats = handler->get_statistics();
    //         RCLCPP_INFO(
    //             parent_node_->get_logger(),
    //             "Service '%s': %lu requests, %.2f%% success rate",
    //             name.c_str(),
    //             stats.request_count,
    //             stats.success_rate
    //         );
    //     }
    // }

private:
    rclcpp::Node* parent_node_;
    std::map<std::string, std::shared_ptr<rclcpp::Node>> service_handlers_;

    void setupLeftRobotServices() {
        // 左臂机器人移动服务
        auto left_move_handler = std::make_shared<RobotMoveHandler>(
            "left_robot_move_server",
            "/L/robot_move",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotMove::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotMove::Response> response
            ) { handleRobotMoveRequest("L", request, response); },
            rmw_qos_profile_services_default,
            true // 启用统计
        );
        service_handlers_["L_robot_move"] = left_move_handler;

        // 左臂机器人笛卡尔移动服务
        auto left_move_cart_handler = std::make_shared<RobotMoveCartHandler>(
            "left_robot_move_cart_server",
            "/L/robot_move_cart",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Response> response
            ) { handleRobotMoveCartRequest("L", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["L_robot_move_cart"] = left_move_cart_handler;

        // 左臂机器人动作服务
        auto left_act_handler = std::make_shared<RobotActHandler>(
            "left_robot_act_server",
            "/L/robot_act",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotAct::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotAct::Response> response
            ) { handleRobotActRequest("L", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["L_robot_act"] = left_act_handler;

        // 左臂机器人关节动作服务
        auto left_act_j_handler = std::make_shared<RobotActJHandler>(
            "left_robot_act_j_server",
            "/L/robot_act_j",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotActJ::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotActJ::Response> response
            ) { handleRobotActJRequest("L", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["L_robot_act_j"] = left_act_j_handler;
    }

    void setupRightRobotServices() {
        // 右臂机器人移动服务
        auto right_move_handler = std::make_shared<RobotMoveHandler>(
            "right_robot_move_server",
            "/R/robot_move",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotMove::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotMove::Response> response
            ) { handleRobotMoveRequest("R", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["R_robot_move"] = right_move_handler;

        // 右臂机器人笛卡尔移动服务
        auto right_move_cart_handler = std::make_shared<RobotMoveCartHandler>(
            "right_robot_move_cart_server",
            "/R/robot_move_cart",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Response> response
            ) { handleRobotMoveCartRequest("R", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["R_robot_move_cart"] = right_move_cart_handler;

        // 右臂机器人动作服务
        auto right_act_handler = std::make_shared<RobotActHandler>(
            "right_robot_act_server",
            "/R/robot_act",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotAct::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotAct::Response> response
            ) { handleRobotActRequest("R", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["R_robot_act"] = right_act_handler;

        // 右臂机器人关节动作服务
        auto right_act_j_handler = std::make_shared<RobotActJHandler>(
            "right_robot_act_j_server",
            "/R/robot_act_j",
            [this](
                const std::shared_ptr<robo_ctrl::srv::RobotActJ::Request> request,
                std::shared_ptr<robo_ctrl::srv::RobotActJ::Response> response
            ) { handleRobotActJRequest("R", request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["R_robot_act_j"] = right_act_j_handler;
    }

    void setupGripperServices() {
        // 夹爪命令服务
        auto gripper_handler = std::make_shared<GripperCommandHandler>(
            "gripper_command_server",
            "/epg50_gripper/command",
            [this](
                const std::shared_ptr<epg50_gripper_ros::srv::GripperCommand::Request> request,
                std::shared_ptr<epg50_gripper_ros::srv::GripperCommand::Response> response
            ) { handleGripperCommandRequest(request, response); },
            rmw_qos_profile_services_default,
            true
        );
        service_handlers_["gripper_command"] = gripper_handler;
    }

    // 服务处理回调函数
    void handleRobotMoveRequest(
        const std::string& robot_id,
        const std::shared_ptr<robo_ctrl::srv::RobotMove::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotMove::Response> response
    ) {
        RCLCPP_INFO(parent_node_->get_logger(), "Processing robot move request for robot %s", robot_id.c_str());

        // 这里应该转发到实际的机器人控制器
        // 暂时模拟成功响应
        response->success = true;
        response->message = "Robot move command received for robot " + robot_id;
    }

    void handleRobotMoveCartRequest(
        const std::string& robot_id,
        const std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Response> response
    ) {
        RCLCPP_INFO(parent_node_->get_logger(), "Processing robot move cart request for robot %s", robot_id.c_str());

        response->success = true;
        response->message = "Robot move cart command received for robot " + robot_id;
    }

    void handleRobotActRequest(
        const std::string& robot_id,
        const std::shared_ptr<robo_ctrl::srv::RobotAct::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotAct::Response> response
    ) {
        RCLCPP_INFO(
            parent_node_->get_logger(),
            "Processing robot act request for robot %s, command_type: %d",
            robot_id.c_str(),
            request->command_type
        );

        response->success = true;
        response->message = "Robot act command received for robot " + robot_id;
    }

    void handleRobotActJRequest(
        const std::string& robot_id,
        const std::shared_ptr<robo_ctrl::srv::RobotActJ::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotActJ::Response> response
    ) {
        RCLCPP_INFO(
            parent_node_->get_logger(),
            "Processing robot act j request for robot %s, command_type: %d",
            robot_id.c_str(),
            request->command_type
        );

        response->success = true;
        response->message = "Robot act j command received for robot " + robot_id;
    }

    void handleGripperCommandRequest(
        const std::shared_ptr<epg50_gripper_ros::srv::GripperCommand::Request> request,
        std::shared_ptr<epg50_gripper_ros::srv::GripperCommand::Response> response
    ) {
        RCLCPP_INFO(
            parent_node_->get_logger(),
            "Processing gripper command: slave_id=%d, command=%d, position=%d",
            request->slave_id,
            request->command,
            request->position
        );

        response->success = true;
        response->message = "Gripper command received";
    }
};

/**
 * @brief 简化的服务客户端管理器
 */
class ServiceClientManager {
public:
    explicit ServiceClientManager(rclcpp::Node* parent_node): parent_node_(parent_node) {
        initializeClients();
    }

    /**
     * @brief 等待所有服务可用
     */
    bool waitForAllServices(std::chrono::seconds timeout = std::chrono::seconds(30)) {
        RCLCPP_INFO(parent_node_->get_logger(), "Waiting for all required services...");

        auto start_time = std::chrono::steady_clock::now();
        for (const auto& [name, client]: clients_) {
            RCLCPP_INFO(parent_node_->get_logger(), "Waiting for service: %s", name.c_str());

            while (!client->wait_for_service(std::chrono::seconds(1))) {
                auto elapsed = std::chrono::steady_clock::now() - start_time;
                if (elapsed > timeout) {
                    RCLCPP_ERROR(parent_node_->get_logger(), "Timeout waiting for service: %s", name.c_str());
                    return false;
                }

                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(parent_node_->get_logger(), "Interrupted while waiting for service: %s", name.c_str());
                    return false;
                }

                RCLCPP_INFO(parent_node_->get_logger(), "Still waiting for service: %s", name.c_str());
            }

            RCLCPP_INFO(parent_node_->get_logger(), "Service '%s' is available", name.c_str());
        }

        RCLCPP_INFO(parent_node_->get_logger(), "All required services are now available!");
        return true;
    }

    /**
     * @brief 获取服务客户端的便捷访问器
     */
    template<typename ServiceType>
    std::shared_ptr<rclcpp::Client<ServiceType>> getClient(const std::string& name) {
        auto it = clients_.find(name);
        if (it != clients_.end()) {
            return std::static_pointer_cast<rclcpp::Client<ServiceType>>(it->second);
        }
        return nullptr;
    }

private:
    rclcpp::Node* parent_node_;
    std::map<std::string, std::shared_ptr<rclcpp::ClientBase>> clients_;

    void initializeClients() {
        // 左臂机器人客户端
        clients_["L_robot_move"] = parent_node_->create_client<robo_ctrl::srv::RobotMove>("/L/robot_move");
        clients_["L_robot_move_cart"] =
            parent_node_->create_client<robo_ctrl::srv::RobotMoveCart>("/L/robot_move_cart");
        clients_["L_robot_act"]   = parent_node_->create_client<robo_ctrl::srv::RobotAct>("/L/robot_act");
        clients_["L_robot_act_j"] = parent_node_->create_client<robo_ctrl::srv::RobotActJ>("/L/robot_act_j");
        clients_["L_robot_servo"] = parent_node_->create_client<robo_ctrl::srv::RobotServo>("/L/robot_servo");
        clients_["L_robot_servo_line"] =
            parent_node_->create_client<robo_ctrl::srv::RobotServoLine>("/L/robot_servo_line");
        clients_["L_robot_set_speed"] =
            parent_node_->create_client<robo_ctrl::srv::RobotSetSpeed>("/L/robot_set_speed");

        // 右臂机器人客户端
        clients_["R_robot_move"] = parent_node_->create_client<robo_ctrl::srv::RobotMove>("/R/robot_move");
        clients_["R_robot_move_cart"] =
            parent_node_->create_client<robo_ctrl::srv::RobotMoveCart>("/R/robot_move_cart");
        clients_["R_robot_act"]   = parent_node_->create_client<robo_ctrl::srv::RobotAct>("/R/robot_act");
        clients_["R_robot_act_j"] = parent_node_->create_client<robo_ctrl::srv::RobotActJ>("/R/robot_act_j");
        clients_["R_robot_servo"] = parent_node_->create_client<robo_ctrl::srv::RobotServo>("/R/robot_servo");
        clients_["R_robot_servo_line"] =
            parent_node_->create_client<robo_ctrl::srv::RobotServoLine>("/R/robot_servo_line");
        clients_["R_robot_set_speed"] =
            parent_node_->create_client<robo_ctrl::srv::RobotSetSpeed>("/R/robot_set_speed");

        // 夹爪客户端
        clients_["gripper_command"] =
            parent_node_->create_client<epg50_gripper_ros::srv::GripperCommand>("/epg50_gripper/command");
    }
};

/**
 * @brief 重构后的机器人主控类
 */
class RobotMainRefactored: public rclcpp::Node {
public:
    explicit RobotMainRefactored(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()):
        Node("robot_main_refactored", options),
        service_manager_(this),
        client_manager_(this) {
        initializeParameters();
        initializeTF();
        initializeSubscribers();

        // 初始化服务管理器
        service_manager_.initialize();

        // 等待外部服务可用
        if (!client_manager_.waitForAllServices()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to required services");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RobotMainRefactored initialized successfully");
    }

    ~RobotMainRefactored() {
        RCLCPP_INFO(this->get_logger(), "RobotMainRefactored shutting down");
        // service_manager_.printStatistics();
    }

    /**
     * @brief 便捷的服务调用方法
     */
    template<typename ServiceType>
    auto callService(const std::string& client_name, typename ServiceType::Request::SharedPtr request)
        -> std::shared_future<typename rclcpp::Client<ServiceType>::SharedResponse> {
        auto client = client_manager_.getClient<ServiceType>(client_name);
        if (!client) {
            RCLCPP_ERROR(this->get_logger(), "Client '%s' not found", client_name.c_str());
            // 返回一个已经设置为失败的future
            auto promise      = std::make_shared<std::promise<typename ServiceType::Response::SharedPtr>>();
            auto response     = std::make_shared<typename ServiceType::Response>();
            response->success = false;
            response->message = "Client not found: " + client_name;
            promise->set_value(response);
            return promise->get_future();
        }
        return client->async_send_request(request);
    }

    /**
     * @brief 同步服务调用的便捷方法
     */
    template<typename ServiceType>
    typename ServiceType::Response::SharedPtr callServiceSync(
        const std::string& client_name,
        typename ServiceType::Request::SharedPtr request,
        std::chrono::seconds timeout = std::chrono::seconds(5)
    ) {
        auto future = callService<ServiceType>(client_name, request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, timeout)
            == rclcpp::FutureReturnCode::SUCCESS) {
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call timeout for client: %s", client_name.c_str());
            auto response     = std::make_shared<typename ServiceType::Response>();
            response->success = false;
            response->message = "Service call timeout";
            return response;
        }
    }

private:
    ServiceManager service_manager_;
    ServiceClientManager client_manager_;

    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 机器人状态
    robo_ctrl::msg::RobotState::SharedPtr L_robot_state_;
    robo_ctrl::msg::RobotState::SharedPtr R_robot_state_;
    mutable std::mutex L_robot_state_mutex_;
    mutable std::mutex R_robot_state_mutex_;

    // 参数
    std::vector<double> init_tcp_pose_vec_;
    std::vector<double> CAP_OPEN_JOINTS_L;
    std::vector<double> CAP_OPEN_JOINTS_R;
    int desk_height_;
    int cola_height_;

    void initializeParameters() {
        this->declare_parameter("init_tcp_pose", std::vector<double> { 168.0, -102.0, 394.0, -111.556, 0.0, -90.0 });
        this->declare_parameter("cap_open_joints_l", std::vector<double> { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        this->declare_parameter("cap_open_joints_r", std::vector<double> { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        this->declare_parameter("desk_height", 190);
        this->declare_parameter("cola_height", 91);

        init_tcp_pose_vec_ = this->get_parameter("init_tcp_pose").as_double_array();
        CAP_OPEN_JOINTS_L  = this->get_parameter("cap_open_joints_l").as_double_array();
        CAP_OPEN_JOINTS_R  = this->get_parameter("cap_open_joints_r").as_double_array();
        desk_height_       = this->get_parameter("desk_height").as_int();
        cola_height_       = this->get_parameter("cola_height").as_int();
    }

    void initializeTF() {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void initializeSubscribers() {
        // 左臂机器人状态订阅
        auto L_robot_state_sub_ = this->create_subscription<robo_ctrl::msg::RobotState>(
            "/L/robot_state",
            10,
            [this](const robo_ctrl::msg::RobotState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
                L_robot_state_ = msg;
            }
        );

        // 右臂机器人状态订阅
        auto R_robot_state_sub_ = this->create_subscription<robo_ctrl::msg::RobotState>(
            "/R/robot_state",
            10,
            [this](const robo_ctrl::msg::RobotState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(R_robot_state_mutex_);
                R_robot_state_ = msg;
            }
        );
    }
};

/**
 * @brief 演示如何使用重构后的类执行机器人任务
 */
void demonstrateRobotTasks(std::shared_ptr<RobotMainRefactored> node) {
    RCLCPP_INFO(node->get_logger(), "Starting robot task demonstration...");

    // 示例1: 夹爪使能
    auto gripper_enable_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_enable_request->slave_id = GRIPPER_ID_L;
    gripper_enable_request->command  = GRIPPER_ENABLE;
    gripper_enable_request->position = GRIPPER_OPEN;
    gripper_enable_request->speed    = 255;
    gripper_enable_request->torque   = 255;

    auto gripper_response =
        node->callServiceSync<epg50_gripper_ros::srv::GripperCommand>("gripper_command", gripper_enable_request);

    if (gripper_response && gripper_response->success) {
        RCLCPP_INFO(node->get_logger(), "Gripper enabled successfully: %s", gripper_response->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to enable gripper");
        return;
    }

    // 示例2: 机器人移动到初始位置
    auto robot_act_request             = std::make_shared<robo_ctrl::srv::RobotAct::Request>();
    robot_act_request->command_type    = 0; // ServoMoveStart
    robot_act_request->tcp_pose.x      = 168.0;
    robot_act_request->tcp_pose.y      = -102.0;
    robot_act_request->tcp_pose.z      = 394.0;
    robot_act_request->tcp_pose.rx     = -111.556;
    robot_act_request->tcp_pose.ry     = 0.0;
    robot_act_request->tcp_pose.rz     = -90.0;
    robot_act_request->point_count     = 100;
    robot_act_request->message_time    = 0.02;
    robot_act_request->plan_type       = 0; // 直线规划
    robot_act_request->use_incremental = false;

    auto robot_response = node->callServiceSync<robo_ctrl::srv::RobotAct>("L_robot_act", robot_act_request);

    if (robot_response && robot_response->success) {
        RCLCPP_INFO(node->get_logger(), "Robot moved to initial position: %s", robot_response->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to move robot to initial position");
    }

    RCLCPP_INFO(node->get_logger(), "Robot task demonstration completed");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto node = std::make_shared<RobotMainRefactored>(options);

    // 创建一个多线程执行器来处理并发的服务请求
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 启动演示任务
    std::thread demo_thread([node]() {
        // 等待一段时间让系统稳定
        std::this_thread::sleep_for(std::chrono::seconds(2));
        demonstrateRobotTasks(node);
    });

    RCLCPP_INFO(node->get_logger(), "RobotMainRefactored is running...");

    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    }

    demo_thread.join();
    rclcpp::shutdown();
    return 0;
}
