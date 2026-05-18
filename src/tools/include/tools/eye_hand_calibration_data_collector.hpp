#ifndef EYE_HAND_CALIBRATION_DATA_COLLECTOR_HPP
#define EYE_HAND_CALIBRATION_DATA_COLLECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// 添加消息同步器相关头文件
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// 包含机器人控制相关消息类型
#include "robo_ctrl/msg/tcp_pose.hpp"
#include "robo_ctrl/msg/robot_state.hpp"

namespace utils {

/**
 * @brief 用于手眼标定的数据收集器
 *
 * 该节点订阅相机图像和机器人状态信息，
 * 使用消息同步器进行时间同步匹配，
 * 并在接收到触发信号时或自动保存当前图像和TCP位姿，
 * 用于手眼标定。
 */
class EyeHandCalibrationDataCollector: public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    explicit EyeHandCalibrationDataCollector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~EyeHandCalibrationDataCollector();

private:
    // 消息同步器定义
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, robo_ctrl::msg::RobotState>
        SyncPolicy;

    // 同步订阅器
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<robo_ctrl::msg::RobotState> robot_state_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

    // 提供触发服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_;

    // 当前图像和TCP位姿
    sensor_msgs::msg::Image::SharedPtr current_image_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;

    // TF监听器，用于获取额外的坐标转换信息
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

    // 话题名称
    std::string image_topic_;
    std::string robot_state_topic_;

    // 坐标系信息
    std::string camera_frame_id_;
    std::string robot_base_frame_id_;

    // TCP-only模式设置
    bool tcp_only_mode_;

    // 同步回调函数
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const robo_ctrl::msg::RobotState::ConstSharedPtr& robot_state_msg
    );

    // TCP-only模式的机器人状态回调函数
    void robotStateCallback(const robo_ctrl::msg::RobotState::ConstSharedPtr& robot_state_msg);

    // 触发采集回调函数
    void captureCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    // 自动采集回调函数
    void autoCaptureTick();

    // 保存数据
    bool saveCalibrationSample();

    // 将TCP位姿转换为PoseStamped消息
    geometry_msgs::msg::PoseStamped tcpPoseToPoseStamped(const robo_ctrl::msg::TCPPose& tcp_pose);
};

} // namespace utils

#endif // EYE_HAND_CALIBRATION_DATA_COLLECTOR_HPP