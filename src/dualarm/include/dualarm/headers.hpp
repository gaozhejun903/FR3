#include "epg50_gripper_ros/srv/gripper_command.hpp"
#include "epg50_gripper_ros/srv/gripper_status.hpp"
#include "epg50_gripper_ros/msg/gripper_status.hpp"
#include "robo_ctrl/msg/tcp_pose.hpp"
#include "robo_ctrl/msg/robot_state.hpp"
#include "robo_ctrl/srv/robot_servo_line.hpp"
#include "robo_ctrl/srv/robot_act.hpp"
#include "robo_ctrl/srv/robot_act_j.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "robo_ctrl/srv/robot_move.hpp"
#include "robo_ctrl/srv/robot_move_cart.hpp"
#include "robo_ctrl/srv/robot_servo.hpp"
#include "robo_ctrl/msg/robot_state.hpp"
#include "robo_ctrl/srv/robot_set_speed.hpp"
#include "robo_ctrl/srv/robot_servo_line.hpp"
#include "robo_ctrl/srv/robot_servo_joint.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "depth_handler/msg/bbox3d.hpp"
#include "depth_handler/msg/bbox3d_array.hpp"
#include "dualarm/kalman.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

struct object_detection {
    int id;                       // 物体ID
    std::vector<double> position; // 物体位置
    int age         = 0;          // 物体检测年龄，初始为0
    int valid_count = 0;          // 有效检测次数，初始为0
    bool is_valid() const {
        // 有效检测次数超过3次且年龄小于5次则认为是有效
        return valid_count >= 1 && age < 5;
    }
    bool is_invalid() const {
        // 有效检测次数小于等于3次或年龄大于等于5则认为是无效
        return valid_count <= 1 || age >= 20;
    }
    bool operator==(const object_detection& other) const {
        return id == other.id;
    }
    bool operator!=(const object_detection& other) const {
        return !(*this == other);
    }
};

class RobotKalmanFilter {
private:
    KalmanFilter pose_filter_;

public:
    RobotKalmanFilter() {
        // 初始化滤波器参数
        pose_filter_.setDeltaTime(0.02);       // 50Hz
        pose_filter_.setProcessNoise(0.005);   // 低过程噪声
        pose_filter_.setMeasurementNoise(0.1); // 观测噪声
    }
    ~RobotKalmanFilter() {}

    // 处理来自机器人的TCP位置数据
    std::vector<double> filterPose(const std::vector<double>& raw_pose) {
        // 预测步骤
        pose_filter_.predict();

        // 使用原始TCP位置更新滤波器
        std::vector<double> filtered_pose = pose_filter_.update(raw_pose);

        return filtered_pose;
    }

    // 获取当前TCP位置和速度
    std::vector<double> getCurrentPosition() {
        return pose_filter_.getPosition();
    }

    std::vector<double> getCurrentVelocity() {
        return pose_filter_.getVelocity();
    }

    // 重置滤波器
    void resetFilter() {
        pose_filter_.reset();
    }
};

class RobotMain: public rclcpp::Node {
public:
    explicit RobotMain(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~RobotMain();
    void handleObjDetection(const depth_handler::msg::Bbox3dArray::SharedPtr bbox3d_array_msg);
    std::vector<double> calculateTcpToObjectIncrement(const std::vector<double>& object_position);

    // 线程安全的访问函数
    robo_ctrl::msg::TCPPose getCurrentTcpPose() const {
        std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
        if (L_robot_state_) {
            return L_robot_state_->tcp_pose;
        }
        return robo_ctrl::msg::TCPPose();
    }

    bool isRobotStateValid() const {
        std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
        return L_robot_state_ != nullptr;
    }

    // tf2 相关成员
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    robo_ctrl::msg::RobotState::SharedPtr L_robot_state_;
    epg50_gripper_ros::msg::GripperStatus::SharedPtr L_gripper_status_;
    robo_ctrl::msg::RobotState::SharedPtr R_robot_state_;

    // 线程锁保护共享资源
    mutable std::mutex L_robot_state_mutex_;
    mutable std::mutex L_gripper_status_mutex_;
    mutable std::mutex detected_objects_mutex_; // 保护物体检测数据的互斥锁

    std::string ROBOT_L = "/L";
    std::string ROBOT_R = "/R";

    // subscribers
    rclcpp::Subscription<robo_ctrl::msg::RobotState>::SharedPtr L_robot_state_sub_;
    rclcpp::Subscription<epg50_gripper_ros::msg::GripperStatus>::SharedPtr L_gripper_status_sub_;
    rclcpp::Subscription<depth_handler::msg::Bbox3dArray>::SharedPtr L_bbox3d_array_sub_;

    // clients
    rclcpp::Client<robo_ctrl::srv::RobotServoLine>::SharedPtr L_robot_servo_line_client_;
    rclcpp::Client<robo_ctrl::srv::RobotAct>::SharedPtr L_robot_act_client_;
    rclcpp::Client<robo_ctrl::srv::RobotActJ>::SharedPtr L_robot_act_j_client_;
    rclcpp::Client<robo_ctrl::srv::RobotMove>::SharedPtr L_robot_move_client_;
    rclcpp::Client<robo_ctrl::srv::RobotMoveCart>::SharedPtr L_robot_move_cart_client_;
    rclcpp::Client<robo_ctrl::srv::RobotServo>::SharedPtr L_robot_servo_client_;
    rclcpp::Client<robo_ctrl::srv::RobotSetSpeed>::SharedPtr L_robot_set_speed_client_;
    rclcpp::Client<epg50_gripper_ros::srv::GripperCommand>::SharedPtr gripper_command_client_;

    // publishers
    rclcpp::Publisher<robo_ctrl::msg::TCPPose>::SharedPtr L_tcp_pose_pub_;
    rclcpp::Publisher<robo_ctrl::msg::RobotState>::SharedPtr L_robot_state_pub_;

    rclcpp::Subscription<robo_ctrl::msg::RobotState>::SharedPtr R_robot_state_sub_;
    rclcpp::Subscription<epg50_gripper_ros::msg::GripperStatus>::SharedPtr R_gripper_status_sub_;

    rclcpp::Client<robo_ctrl::srv::RobotServoLine>::SharedPtr R_robot_servo_line_client_;
    rclcpp::Client<robo_ctrl::srv::RobotAct>::SharedPtr R_robot_act_client_;
    rclcpp::Client<robo_ctrl::srv::RobotActJ>::SharedPtr R_robot_act_j_client_;
    rclcpp::Client<robo_ctrl::srv::RobotMove>::SharedPtr R_robot_move_client_;
    rclcpp::Client<robo_ctrl::srv::RobotMoveCart>::SharedPtr R_robot_move_cart_client_;
    rclcpp::Client<robo_ctrl::srv::RobotServo>::SharedPtr R_robot_servo_client_;
    rclcpp::Client<robo_ctrl::srv::RobotSetSpeed>::SharedPtr R_robot_set_speed_client_;

    robo_ctrl::msg::TCPPose::SharedPtr init_tcp_pose_;
    std::vector<double> init_tcp_pose_vec_;
    std::vector<double> obj_detection_pose_vec_;
    std::vector<RobotKalmanFilter> kalman_filters_;
    std::vector<object_detection> detected_objects_;
    rclcpp::CallbackGroup::SharedPtr obj_detect_cb_group_;
    int desk_height_    = 0;  // 台面高度
    int cola_height_    = 91; // 可乐高度
    int cestbon_height_ = 91; // 矿泉水高度
    std::vector<double> ball_R_joint_init_pose_;
    std::vector<double> ball_L_joint_init_pose_;
    std::vector<double> ball_L_1_joint_pose_;
    std::vector<double> ball_L_2_joint_pose_;
    std::vector<double> ball_R_1_joint_pose_;
    std::vector<double> ball_R_2_joint_pose_;
    std::vector<double> ball_R_per_1_joint_pose_;

    // 物体检测控制参数
    bool enable_object_update_ = true; // 遮断开关：控制是否更新物体位置
    int age_threshold_         = 5;    // 年龄阈值：超过此值的物体将被移除
    double distance_threshold_ = 0.15; // 距离阈值：15cm内认为是同一物体

    std::vector<double> CAP_OPEN_JOINTS_L         = { -55, -90, -120, 30, 81.272, 0 };
    std::vector<double> CAP_OPEN_JOINTS_R         = { 49.94319534301758,  -112.81088256835938, 124.09851837158203,
                                                      -189.9285888671875, 21.93675994873047,   0 };
    std::vector<double> castbon_CAP_OPEN_JOINTS_R = { 46.952, -113.396, 116.363, -188.627, 24.346, 0.0 };

    // 物体检测控制方法
    void setObjectUpdateEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        enable_object_update_ = enabled;
        RCLCPP_INFO(this->get_logger(), "Object update %s", enabled ? "enabled" : "disabled");
    }

    bool isObjectUpdateEnabled() const {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        return enable_object_update_;
    }

    void setAgeThreshold(int threshold) {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        age_threshold_ = threshold;
        RCLCPP_INFO(this->get_logger(), "Age threshold set to %d", threshold);
    }

    void clearDetectedObjects() {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        detected_objects_.clear();
        kalman_filters_.clear();
        RCLCPP_INFO(this->get_logger(), "Cleared all detected objects");
    }

    // 线程安全的获取检测到的物体数量
    size_t getDetectedObjectsCount() const {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        return detected_objects_.size();
    }

    // 线程安全的获取特定物体位置
    std::vector<double> getObjectPosition(int object_id) const {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        for (const auto& obj: detected_objects_) {
            if (obj.id == object_id && obj.is_valid()) {
                return obj.position;
            }
        }
        return {}; // 返回空向量表示未找到
    }

    // 线程安全的检查特定物体是否存在
    bool hasObject(int object_id) const {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        for (const auto& obj: detected_objects_) {
            if (obj.id == object_id && obj.is_valid()) {
                return true;
            }
        }
        return false;
    }

    // 线程安全的获取所有有效物体的副本
    std::vector<object_detection> getValidObjects() const {
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);
        std::vector<object_detection> valid_objects;
        for (const auto& obj: detected_objects_) {
            if (obj.is_valid()) {
                valid_objects.push_back(obj);
            }
        }
        return valid_objects;
    }

    inline void gen_move_request(
        robo_ctrl::srv::RobotMoveCart::Request::SharedPtr& request,
        const std::vector<double>& tcp_pose,
        const bool use_increment = false
    ) {
        request->tcp_pose.x    = tcp_pose[0];
        request->tcp_pose.y    = tcp_pose[1];
        request->tcp_pose.z    = tcp_pose[2];
        request->tcp_pose.rx   = tcp_pose[3];
        request->tcp_pose.ry   = tcp_pose[4];
        request->tcp_pose.rz   = tcp_pose[5];
        request->velocity      = 100;
        request->acceleration  = 100;
        request->tool          = 0;
        request->blend_time    = -1;
        request->config        = -1;
        request->ovl           = 100;
        request->user          = 0;
        request->use_increment = use_increment;
    }

    inline void gen_actJ_request(
        robo_ctrl::srv::RobotActJ::Request::SharedPtr& request,
        const std::vector<double>& joint_pose,
        const bool use_increment = false
    ) {
        request->command_type    = 0; // ServoMoveStart
        request->target_joints   = joint_pose;
        request->point_count     = 100;  // 100个点
        request->message_time    = 0.01; // 0.01s/点
        request->use_incremental = use_increment;
    }

    inline void wait_until_done() {
        // 等待运动完成
        while (rclcpp::ok()) {
            bool motion_done = false;
            {
                std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
                if (L_robot_state_) {
                    motion_done = L_robot_state_->motion_done;
                }
            }

            if (motion_done) {
                RCLCPP_INFO(this->get_logger(), "Motion completed, waiting additional 2 seconds...");
                break;
            }

            // 短暂休眠避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            rclcpp::spin_some(this->shared_from_this());
        }

        // 额外等待2秒
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "Motion done wait completed");
    }
};
