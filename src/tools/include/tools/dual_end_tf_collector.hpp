#ifndef DUAL_END_TF_COLLECTOR_HPP
#define DUAL_END_TF_COLLECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <iomanip>
#include <sstream>

namespace utils {

/**
 * @brief 双臂末端TF数据收集器
 *
 * 该节点从TF中订阅Rend和Lend的位置信息，
 * 并在接收到触发信号时或自动保存当前双臂末端的位姿，
 * 用于双臂协作标定。
 */
class DualEndTfCollector: public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    explicit DualEndTfCollector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~DualEndTfCollector();

private:
    // TF监听器，用于获取坐标转换信息
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 提供触发服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_;

    // 当前双臂末端位姿
    geometry_msgs::msg::PoseStamped::SharedPtr current_rend_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_lend_pose_;

    // 数据保存路径
    std::string save_path_;

    // 已采集的样本数量
    int sample_count_;

    // 自动采集模式
    bool auto_capture_;
    int auto_capture_interval_; // 自动采集间隔（毫秒）
    rclcpp::TimerBase::SharedPtr auto_capture_timer_;

    // 内部互斥锁，确保数据一致性
    std::mutex data_mutex_;

    // 坐标系信息
    std::string rend_frame_id_;
    std::string lend_frame_id_;
    std::string base_frame_id_;
    std::string rend_base_frame_id_;
    std::string lend_base_frame_id_;

    // TF数据更新定时器
    rclcpp::TimerBase::SharedPtr tf_update_timer_;
    int tf_update_interval_; // TF更新间隔（毫秒）

    // TF数据更新回调函数
    void tfUpdateCallback();

    // 触发采集回调函数
    void captureCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    // 自动采集回调函数
    void autoCaptureTick();

    // 保存双臂末端位姿数据
    bool saveDualEndSample();

    // 从TF获取变换并转换为PoseStamped
    bool getTransformAsPose(
        const std::string& target_frame,
        const std::string& source_frame,
        geometry_msgs::msg::PoseStamped& pose
    );

    // 将变换转换为PoseStamped格式
    geometry_msgs::msg::PoseStamped transformToPoseStamped(const geometry_msgs::msg::TransformStamped& transform);
};

} // namespace utils

#endif // DUAL_END_TF_COLLECTOR_HPP
