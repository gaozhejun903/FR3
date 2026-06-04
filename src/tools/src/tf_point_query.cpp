#include "tools/tf_point_query.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

namespace utils {

TFPointQuery::TFPointQuery(const rclcpp::NodeOptions& options): Node("tf_point_query", options) {
    RCLCPP_INFO(this->get_logger(), "启动TF点位置查询服务节点");

    // 创建TF缓冲区和监听器
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建服务
    transform_srv_ = this->create_service<tools::srv::TransformPoint>(
        "transform_point",
        std::bind(&TFPointQuery::transformPointCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "TF点位置查询服务已启动，等待请求...");
}

TFPointQuery::~TFPointQuery() {
    RCLCPP_INFO(this->get_logger(), "TF点位置查询服务节点关闭");
}

void TFPointQuery::transformPointCallback(
    const std::shared_ptr<tools::srv::TransformPoint::Request> request,
    std::shared_ptr<tools::srv::TransformPoint::Response> response
) {
    RCLCPP_INFO(
        this->get_logger(),
        "接收到坐标转换请求：从 %s 到 %s 转换点 [%f, %f, %f]",
        request->source_frame.c_str(),
        request->target_frame.c_str(),
        request->point.x,
        request->point.y,
        request->point.z
    );

    // 设置默认值
    response->success = false;

    // 创建要转换的点
    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = request->source_frame;
    point_in.header.stamp    = this->now();
    point_in.point           = request->point;

    // 设置超时
    double timeout = request->timeout;
    if (timeout <= 0.0) {
        timeout = 1.0; // 默认超时时间为1秒
    }

    // 等待变换可用
    rclcpp::Time now = this->now();
    if (!waitForTransform(request->target_frame, request->source_frame, now, rclcpp::Duration::from_seconds(timeout))) {
        std::string error_msg = "无法获取从 " + request->source_frame + " 到 " + request->target_frame + " 的变换";
        RCLCPP_WARN(this->get_logger(), "%s", error_msg.c_str());
        response->error_msg = error_msg;
        return;
    }

    try {
        // 执行点的变换
        geometry_msgs::msg::PointStamped point_out;
        tf_buffer_->transform(point_in, point_out, request->target_frame);

        // 设置响应
        response->transformed_point = point_out;
        response->success           = true;

        RCLCPP_INFO(
            this->get_logger(),
            "转换成功：结果点 [%f, %f, %f] 在 %s 坐标系中",
            point_out.point.x,
            point_out.point.y,
            point_out.point.z,
            request->target_frame.c_str()
        );
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "转换点失败: %s", ex.what());
        response->error_msg = ex.what();
    }
}

bool TFPointQuery::waitForTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    const rclcpp::Duration& timeout
) {
    rclcpp::Time start_time = this->now();
    rclcpp::Duration elapsed(0, 0);

    // 循环检查变换是否可用，直到超时
    while (elapsed < timeout) {
        if (tf_buffer_->canTransform(target_frame, source_frame, time)) {
            return true;
        }

        // 短暂休眠以避免CPU占用过高
        rclcpp::sleep_for(std::chrono::milliseconds(10));

        // 更新已经过的时间
        elapsed = this->now() - start_time;
    }

    // 超时，无法获得变换
    return false;
}

} // namespace utils