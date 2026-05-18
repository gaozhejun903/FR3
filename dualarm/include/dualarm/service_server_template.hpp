#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 通用服务器模板类，简化ROS2服务的创建和管理
 * @tparam ServiceType ROS2服务类型
 */
template<typename ServiceType>
class ServiceServerTemplate: public rclcpp::Node {
public:
    using ServiceRequest  = typename ServiceType::Request;
    using ServiceResponse = typename ServiceType::Response;
    using ServiceCallback =
        std::function<void(const std::shared_ptr<ServiceRequest>, std::shared_ptr<ServiceResponse>)>;

    /**
     * @brief 构造函数
     * @param node_name 节点名称
     * @param service_name 服务名称
     * @param callback 服务回调函数
     * @param qos_profile QoS配置
     */
    explicit ServiceServerTemplate(
        const std::string& node_name,
        const std::string& service_name,
        ServiceCallback callback,
        const rmw_qos_profile_t& qos_profile = rmw_qos_profile_services_default,
        const bool enable_statistics         = false
    ):
        Node(node_name),
        service_name_(service_name),
        user_callback_(callback),
        request_count_(0),
        success_count_(0),
        error_count_(0),
        enable_statistics_(enable_statistics) {
        // 创建服务
        service_ = this->create_service<ServiceType>(
            service_name_,
            std::bind(&ServiceServerTemplate::service_callback, this, std::placeholders::_1, std::placeholders::_2),
            qos_profile
        );

        // 创建统计信息发布器（可选）
        if (enable_statistics_) {
            stats_timer_ = this->create_wall_timer(
                std::chrono::seconds(stats_publish_interval_),
                std::bind(&ServiceServerTemplate::publish_statistics, this)
            );
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Service server '%s' started for service '%s'",
            node_name.c_str(),
            service_name_.c_str()
        );
    }

    /**
     * @brief 启用/禁用统计信息
     * @param enable 是否启用
     * @param interval 发布间隔（秒）
     */
    void enable_statistics(bool enable = true, int interval = 10) {
        enable_statistics_      = enable;
        stats_publish_interval_ = interval;
    }

    /**
     * @brief 获取服务统计信息
     */
    struct ServiceStatistics {
        uint64_t request_count;
        uint64_t success_count;
        uint64_t error_count;
        double success_rate;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point last_request_time;
    };

    ServiceStatistics get_statistics() const {
        ServiceStatistics stats;
        stats.request_count = request_count_;
        stats.success_count = success_count_;
        stats.error_count   = error_count_;
        stats.success_rate  = request_count_ > 0 ? static_cast<double>(success_count_) / request_count_ * 100.0 : 0.0;
        stats.start_time    = start_time_;
        stats.last_request_time = last_request_time_;
        return stats;
    }

    /**
     * @brief 重置统计信息
     */
    void reset_statistics() {
        request_count_ = 0;
        success_count_ = 0;
        error_count_   = 0;
        start_time_    = std::chrono::steady_clock::now();
    }

    /**
     * @brief 获取服务名称
     */
    const std::string& get_service_name() const {
        return service_name_;
    }
    int stats_publish_interval_ { 10 };

protected:
    /**
     * @brief 内部服务回调函数，包含统计和错误处理
     */
    void service_callback(const std::shared_ptr<ServiceRequest> request, std::shared_ptr<ServiceResponse> response) {
        auto start_time = std::chrono::steady_clock::now();

        // 更新统计信息
        request_count_++;
        last_request_time_ = start_time;

        try {
            // 调用用户定义的回调函数
            user_callback_(request, response);
            success_count_++;

            // 记录处理时间
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            RCLCPP_DEBUG(
                this->get_logger(),
                "Service '%s' processed request in %ld ms",
                service_name_.c_str(),
                duration.count()
            );

        } catch (const std::exception& e) {
            error_count_++;
            RCLCPP_ERROR(this->get_logger(), "Error processing service '%s': %s", service_name_.c_str(), e.what());

            // 可以在这里设置错误响应的默认值
            on_service_error(request, response, e);
        }
    }

    /**
     * @brief 错误处理虚函数，子类可以重写
     */
    virtual void on_service_error(
        [[maybe_unused]] const std::shared_ptr<ServiceRequest>& request,
        [[maybe_unused]] std::shared_ptr<ServiceResponse>& response,
        [[maybe_unused]] const std::exception& e
    ) {
        // 默认实现
        RCLCPP_WARN(this->get_logger(), "Using default error handling for service '%s'", service_name_.c_str());
    }

    /**
     * @brief 发布统计信息
     */
    void publish_statistics() {
        auto stats = get_statistics();
        RCLCPP_INFO(
            this->get_logger(),
            "Service '%s' statistics: Requests: %lu, Success: %lu, Errors: %lu, Success Rate: %.2f%%",
            service_name_.c_str(),
            stats.request_count,
            stats.success_count,
            stats.error_count,
            stats.success_rate
        );
    }

private:
    std::string service_name_;
    typename rclcpp::Service<ServiceType>::SharedPtr service_;
    ServiceCallback user_callback_;

    // 统计信息
    std::atomic<uint64_t> request_count_;
    std::atomic<uint64_t> success_count_;
    std::atomic<uint64_t> error_count_;
    std::chrono::steady_clock::time_point start_time_ { std::chrono::steady_clock::now() };
    std::chrono::steady_clock::time_point last_request_time_ { std::chrono::steady_clock::now() };

    // 统计信息发布
    bool enable_statistics_ { false };
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

// 快速创建服务器
#define CREATE_SERVICE_SERVER(ServiceType, node_name, service_name, callback) \
    std::make_shared<ServiceServerTemplate<ServiceType>>(node_name, service_name, callback)
