#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"
#include "epg50_gripper_ros/srv/gripper_command.hpp"
#include "epg50_gripper_ros/srv/gripper_status.hpp"
#include "epg50_gripper_ros/msg/gripper_status.hpp"
#include "epg50_gripper_ros/srv/rename_gripper.hpp"

class EPG50GripperNode: public rclcpp::Node {
public:
    EPG50GripperNode(): Node("epg50_gripper_node") {
        // 声明参数
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("debug", false);
        this->declare_parameter("default_slave_id", 0x09);   // 添加默认从站ID参数
        this->declare_parameter("status_update_rate", 10.0); // 状态更新频率，单位Hz
        this->declare_parameter("command_timeout", 1.0);     // 命令超时时间，单位秒

        // 获取参数
        std::string port         = this->get_parameter("port").as_string();
        bool debug               = this->get_parameter("debug").as_bool();
        uint8_t default_slave_id = this->get_parameter("default_slave_id").as_int();
        status_update_rate_      = this->get_parameter("status_update_rate").as_double();
        command_timeout_         = this->get_parameter("command_timeout").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "初始化EPG50夹爪, 端口: %s, 默认从站ID: 0x%02X, 状态更新频率: %.1f Hz, 命令超时: %.1f s",
            port.c_str(),
            default_slave_id,
            status_update_rate_,
            command_timeout_
        );

        try {
            gripper_        = std::make_unique<EPG50_Serial>(port, default_slave_id);
            gripper_->debug = debug;

            // 创建服务
            command_service_ = this->create_service<epg50_gripper_ros::srv::GripperCommand>(
                "epg50_gripper/command",
                std::bind(&EPG50GripperNode::handle_command, this, std::placeholders::_1, std::placeholders::_2)
            );

            status_service_ = this->create_service<epg50_gripper_ros::srv::GripperStatus>(
                "epg50_gripper/status",
                std::bind(&EPG50GripperNode::handle_status, this, std::placeholders::_1, std::placeholders::_2)
            );

            rename_service_ = this->create_service<epg50_gripper_ros::srv::RenameGripper>(
                "epg50_gripper/rename",
                std::bind(&EPG50GripperNode::handle_rename, this, std::placeholders::_1, std::placeholders::_2)
            );

            // 创建状态发布器
            status_publisher_ =
                this->create_publisher<epg50_gripper_ros::msg::GripperStatus>("epg50_gripper/status_stream", 10);

            // 启动状态更新线程
            status_thread_ = std::thread(&EPG50GripperNode::status_update_thread, this);

            RCLCPP_INFO(this->get_logger(), "EPG50夹爪节点已启动");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化夹爪失败: %s", e.what());
            rclcpp::shutdown();
        }
    }

    ~EPG50GripperNode() {
        // 设置停止标志并等待线程终止
        stop_thread_ = true;
        if (status_thread_.joinable()) {
            status_thread_.join();
        }

        // 确保夹爪在节点关闭时被禁用
        if (gripper_) {
            gripper_->disable();
        }
    }

private:
    std::unique_ptr<EPG50_Serial> gripper_;
    rclcpp::Service<epg50_gripper_ros::srv::GripperCommand>::SharedPtr command_service_;
    rclcpp::Service<epg50_gripper_ros::srv::GripperStatus>::SharedPtr status_service_;
    rclcpp::Publisher<epg50_gripper_ros::msg::GripperStatus>::SharedPtr status_publisher_;
    rclcpp::Service<epg50_gripper_ros::srv::RenameGripper>::SharedPtr rename_service_;

    std::thread status_thread_;
    bool stop_thread_ = false;
    double status_update_rate_; // 状态更新频率，单位Hz
    double command_timeout_;    // 命令超时时间，单位秒

    // 添加用于同步的互斥锁和条件变量
    std::mutex command_mutex_;           // 用于保护命令执行
    std::mutex status_mutex_;            // 用于保护状态更新
    std::condition_variable command_cv_; // 用于命令执行和状态更新的同步
    bool command_executing_ = false;     // 标记命令是否正在执行

    // 处理命令服务回调
    void handle_command(
        const std::shared_ptr<epg50_gripper_ros::srv::GripperCommand::Request> request,
        std::shared_ptr<epg50_gripper_ros::srv::GripperCommand::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "收到夹爪命令: %d, 从站ID: 0x%02X", request->command, request->slave_id);

        bool success = false;
        std::string message;
        uint8_t slave_id = request->slave_id;

        // 获取命令执行的锁
        {
            std::unique_lock<std::mutex> lock(command_mutex_);
            // 等待其他命令或状态查询完成
            if (!command_cv_.wait_for(
                    lock,
                    std::chrono::milliseconds(static_cast<int>(command_timeout_ * 1000)),
                    [this] { return !command_executing_; }
                )) {
                // 超时无法获取锁
                response->success = false;
                response->message = "命令执行超时，无法获取资源锁";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            command_executing_ = true;
        }

        try {
            switch (request->command) {
                case 0:                  // 禁用
                    if (slave_id == 0) { // 如果从站ID为0，使用默认ID
                        success = gripper_->disable();
                    } else {
                        success = gripper_->disable_with_id(slave_id);
                    }
                    message = success ? "夹爪已禁用" : "夹爪禁用失败";
                    break;

                case 1:                  // 使能
                    if (slave_id == 0) { // 如果从站ID为0，使用默认ID
                        success = gripper_->enable();
                    } else {
                        success = gripper_->enable_with_id(slave_id);
                    }
                    message = success ? "夹爪已使能" : "夹爪使能失败";
                    break;

                case 2:                  // 设置参数
                    if (slave_id == 0) { // 如果从站ID为0，使用默认ID
                        success = gripper_->set_parameters(request->position, request->speed, request->torque);
                    } else {
                        success =
                            gripper_
                                ->set_parameters_with_id(slave_id, request->position, request->speed, request->torque);
                    }
                    message = success ? "设置参数成功 [位置=" + std::to_string(request->position) + ", 速度="
                            + std::to_string(request->speed) + ", 力矩=" + std::to_string(request->torque) + "]"
                                      : "设置参数失败";
                    break;

                default:
                    message = "未知命令";
                    success = false;
            }
        } catch (const std::exception& e) {
            // 捕获并记录异常
            message = std::string("命令执行异常: ") + e.what();
            success = false;
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        }

        response->success = success;
        response->message = message;

        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());

        // 释放命令执行锁
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_executing_ = false;
        }
        command_cv_.notify_all(); // 通知所有等待的线程
    }

    // 处理状态服务回调
    void handle_status(
        const std::shared_ptr<epg50_gripper_ros::srv::GripperStatus::Request> request,
        std::shared_ptr<epg50_gripper_ros::srv::GripperStatus::Response> response
    ) {
        uint8_t slave_id = request->slave_id;
        RCLCPP_INFO(this->get_logger(), "收到状态查询请求，从站ID: 0x%02X", slave_id);

        std::vector<uint16_t> status;
        {
            std::unique_lock<std::mutex> lock(command_mutex_);
            command_cv_.wait(lock, [this] { return !command_executing_; });
            command_executing_ = true;
        }

        if (slave_id == 0) { // 如果从站ID为0，使用默认ID
            status = gripper_->read_status();
        } else {
            status = gripper_->read_status_with_id(slave_id);
        }

        if (!status.empty() && status.size() >= 8) { // 检查是否有足够的状态元素（已修改为8）
            response->success = true;

            // 0x07D0: 低字节-电动夹爪状态寄存器，高字节-留空
            response->status = status[0]; // 夹爪状态
            response->mode   = status[1]; // 留空（可能未使用）

            // 解析状态位
            auto status_bits = gripper_->parse_status_bits(status[0]);
            response->gact   = status_bits.gact; // 使能状态
            response->gmod   = status_bits.gmod; // 工作模式
            response->ggto   = status_bits.ggto; // 动作状态
            response->gsta   = status_bits.gsta; // 夹爪状态
            response->gobj   = status_bits.gobj; // 目标检测状态

            // 0x07D1: 低字节-故障错误状态寄存器，高字节-位置状态寄存器
            response->error    = status[2]; // 故障错误状态
            response->position = status[3]; // 位置状态

            // 0x07D2: 低字节-速度状态寄存器，高字节-力状态(即时电流)寄存器
            response->speed = status[4]; // 速度状态
            response->force = status[5]; // 力状态(即时电流)

            // 0x07D3: 低字节-母线电压寄存器，高字节-环境温度寄存器
            response->voltage     = status[6]; // 母线电压
            response->temperature = status[7]; // 环境温度

            // 获取状态描述
            response->error_message = gripper_->check_errors(status[2] & 0xFF);
            response->object_status = gripper_->get_object_status(status_bits);

            std::string gripper_status = gripper_->get_gripper_status(status_bits);

            // 输出全部状态信息
            RCLCPP_INFO(
                this->get_logger(),
                "夹爪状态: %s\n物体状态: %s\n错误状态: %s\n位置: %d, 速度: %d, 力: %d, 母线电压: %d, 环境温度: %d",
                gripper_status.c_str(),
                response->object_status.c_str(),
                response->error_message.c_str(),
                response->position,
                response->speed,
                response->force,
                response->voltage,
                response->temperature
            );
        } else {
            response->success       = false;
            response->error_message = "读取状态失败";
            RCLCPP_ERROR(this->get_logger(), "读取状态失败");
        }

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_executing_ = false;
        }
        command_cv_.notify_one();
    }

    void handle_rename(
        const std::shared_ptr<epg50_gripper_ros::srv::RenameGripper::Request> request,
        std::shared_ptr<epg50_gripper_ros::srv::RenameGripper::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "收到重命名请求，从站ID: 0x%02X", request->current_slave_id);

        // 执行重命名操作
        bool success      = gripper_->rename_gripper(request->current_slave_id, request->target_slave_id);
        response->success = success;
        response->message = success ? "重命名成功" : "重命名失败";

        RCLCPP_INFO(this->get_logger(), "重命名结果: %s", response->message.c_str());
    }

    // 状态更新线程函数
    void status_update_thread() {
        RCLCPP_INFO(this->get_logger(), "状态更新线程已启动，更新频率: %.1f Hz", status_update_rate_);

        // 计算更新间隔时间（毫秒）
        auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / status_update_rate_));

        while (!stop_thread_ && rclcpp::ok()) {
            try {
                // 检查是否可以进行状态查询（避免与命令执行冲突）
                bool can_query = false;
                {
                    std::unique_lock<std::mutex> lock(command_mutex_);
                    // 等待命令执行完成，但最多等待命令超时时间的一半
                    if (!command_executing_
                        || command_cv_.wait_for(
                            lock,
                            std::chrono::milliseconds(static_cast<int>(command_timeout_ * 500)),
                            [this] { return !command_executing_; }
                        )) {
                        // 短暂锁定以表明我们正在查询状态
                        can_query          = true;
                        command_executing_ = true;
                    }
                }

                if (can_query) {
                    // 获取夹爪状态
                    std::vector<uint16_t> status = gripper_->read_status();

                    // 完成状态查询后释放锁
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        command_executing_ = false;
                    }
                    command_cv_.notify_one();

                    if (!status.empty() && status.size() >= 8) { // 确保我们有足够的状态元素（8个）
                        // 创建并填充状态消息
                        auto msg      = std::make_unique<epg50_gripper_ros::msg::GripperStatus>();
                        msg->slave_id = gripper_->get_slave_id();

                        // 0x07D0: 低字节-电动夹爪状态寄存器，高字节-留空
                        msg->status = status[0]; // 夹爪状态
                        msg->mode   = status[1]; // 留空

                        // 解析状态位
                        auto status_bits = gripper_->parse_status_bits(status[0]);
                        msg->gact        = status_bits.gact; // 使能状态
                        msg->gmod        = status_bits.gmod; // 工作模式
                        msg->ggto        = status_bits.ggto; // 动作状态
                        msg->gsta        = status_bits.gsta; // 夹爪状态
                        msg->gobj        = status_bits.gobj; // 目标检测状态

                        // 0x07D1: 低字节-故障错误状态寄存器，高字节-位置状态寄存器
                        msg->error    = status[2]; // 故障错误状态
                        msg->position = status[3]; // 位置状态

                        // 0x07D2: 低字节-速度状态寄存器，高字节-力状态(即时电流)寄存器
                        msg->speed = status[4]; // 速度状态
                        msg->force = status[5]; // 力状态(即时电流)

                        // 0x07D3: 低字节-母线电压寄存器，高字节-环境温度寄存器
                        msg->voltage     = status[6]; // 母线电压
                        msg->temperature = status[7]; // 环境温度

                        // 根据错误代码获取错误信息
                        msg->error_message = gripper_->check_errors(status[2] & 0xFF);
                        // 获取物体抓取状态描述
                        msg->object_status = gripper_->get_object_status(status_bits);

                        // 发布状态消息
                        status_publisher_->publish(std::move(*msg));

                        // 记录调试信息（低频率）
                        static int debug_counter = 0;
                        if (++debug_counter >= 10) { // 每10次更新输出一次日志
                            debug_counter              = 0;
                            std::string gripper_status = gripper_->get_gripper_status(status_bits);
                            RCLCPP_DEBUG(
                                this->get_logger(),
                                "夹爪状态: %s\n物体状态: %s\n错误状态: %s\n位置: %d, 速度: %d, 力: %d, 母线电压: %d, 环境温度: %d",
                                gripper_status.c_str(),
                                msg->object_status.c_str(),
                                msg->error_message.c_str(),
                                msg->position,
                                msg->speed,
                                msg->force,
                                msg->voltage,
                                msg->temperature
                            );
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "获取夹爪状态失败");
                    }
                } else {
                    // 如果因为命令执行而不能进行状态查询，则跳过本次查询
                    RCLCPP_DEBUG(this->get_logger(), "命令执行中，跳过状态查询");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "状态更新线程异常: %s", e.what());
                // 确保在异常情况下也释放锁
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    command_executing_ = false;
                }
                command_cv_.notify_one();
            }

            // 等待下一个更新周期
            std::this_thread::sleep_for(update_period);
        }

        RCLCPP_INFO(this->get_logger(), "状态更新线程已终止");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EPG50GripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}