#ifndef TF_POINT_QUERY_HPP
#define TF_POINT_QUERY_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <string>

// 自定义服务消息
#include "tools/srv/transform_point.hpp"

namespace utils {

/**
 * @brief TF点位置查询工具节点
 *
 * 该节点提供一个服务，允许将一个坐标系下的点转换到另一个坐标系下。
 * 可用于手动查询和调试坐标系转换。
 */
class TFPointQuery: public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    explicit TFPointQuery(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~TFPointQuery();

private:
    // TF监听器，用于获取坐标转换信息
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 提供坐标转换查询服务
    rclcpp::Service<tools::srv::TransformPoint>::SharedPtr transform_srv_;

    // 转换服务回调函数
    void transformPointCallback(
        const std::shared_ptr<tools::srv::TransformPoint::Request> request,
        std::shared_ptr<tools::srv::TransformPoint::Response> response
    );

    // 等待TF变换是否可用
    bool waitForTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time,
        const rclcpp::Duration& timeout
    );
};

} // namespace utils

#endif // TF_POINT_QUERY_HPP