// 提供高层运动接口
#include "robo_ctrl/srv/robot_servo_line.hpp"
#include "robo_ctrl/srv/robot_act.hpp"
#include "robo_ctrl/srv/robot_act_j.hpp"
#include "robo_ctrl/srv/robot_servo_joint.hpp"
#include "robo_ctrl/msg/tcp_pose.hpp"
#include "robo_ctrl/msg/robot_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/point_stamped.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

class plan_and_execute_node: public rclcpp::Node {
public:
    enum PLAN_TYPE { LINEAR = 0, CIRCULAR };
    explicit plan_and_execute_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~plan_and_execute_node();

private:
    rclcpp::Service<robo_ctrl::srv::RobotAct>::SharedPtr robot_move_service_;
    rclcpp::Service<robo_ctrl::srv::RobotActJ>::SharedPtr robot_act_j_service_;
    rclcpp::Client<robo_ctrl::srv::RobotServoLine>::SharedPtr robot_servo_line_client_;
    rclcpp::Client<robo_ctrl::srv::RobotServoJoint>::SharedPtr robot_servo_joint_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<robo_ctrl::msg::RobotState>::SharedPtr robot_state_sub_;
    robo_ctrl::msg::RobotState::SharedPtr robot_state_msg_;
    std::mutex robot_mutex_;

    // TF2 相关成员变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // TF缓存相关
    std::map<std::string, geometry_msgs::msg::TransformStamped> transform_cache_;
    std::mutex transform_cache_mutex_;
    rclcpp::Time last_cache_update_;
    static constexpr double CACHE_TIMEOUT_SEC = 0.1; // 缓存超时时间100ms

    void robot_state_callback(const robo_ctrl::msg::RobotState::SharedPtr msg);

    void handle_robot_act(
        const std::shared_ptr<robo_ctrl::srv::RobotAct::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotAct::Response> response
    );

    void handle_robot_act_j(
        const std::shared_ptr<robo_ctrl::srv::RobotActJ::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotActJ::Response> response
    );

    void visualize_trajectory(const std::vector<geometry_msgs::msg::Pose>& trajectory);

    // TF变换相关方法
    bool transformPointFromLfakeGripper(
        const geometry_msgs::msg::Point& point_in_gripper,
        geometry_msgs::msg::Point& point_out,
        const std::string& target_frame
    );

    bool transformPosesFromLfakeGripper(
        const std::vector<geometry_msgs::msg::Pose>& poses_in_gripper,
        std::vector<geometry_msgs::msg::Pose>& poses_out,
        const std::string& target_frame
    );

    bool updateTransformCache(const std::string& target_frame);

    std::string robot_name;

    // 添加一个公共方法用于测试TF变换功能
    bool testTransformPoint() {
        // 测试点变换功能
        geometry_msgs::msg::Point test_point;
        test_point.x = 0.05; // 5cm
        test_point.y = 0.03; // 3cm
        test_point.z = 0.01; // 1cm

        geometry_msgs::msg::Point result_point;

        // 测试变换到robot_base坐标系
        if (transformPointFromLfakeGripper(test_point, result_point, "robot_base")) {
            RCLCPP_INFO(
                this->get_logger(),
                "测试成功: Lfake_gripper[%.3f, %.3f, %.3f] -> robot_base[%.3f, %.3f, %.3f]",
                test_point.x,
                test_point.y,
                test_point.z,
                result_point.x,
                result_point.y,
                result_point.z
            );
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "点变换测试失败");
            return false;
        }
    }

    // 测试位姿向量变换功能
    bool testTransformPoses() {
        // 创建测试位姿向量
        std::vector<geometry_msgs::msg::Pose> test_poses;

        for (int i = 0; i < 3; ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x    = 0.02 * (i + 1); // 2cm, 4cm, 6cm
            pose.position.y    = 0.01 * i;       // 0cm, 1cm, 2cm
            pose.position.z    = 0.005 * i;      // 0cm, 0.5cm, 1cm
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            test_poses.push_back(pose);
        }

        std::vector<geometry_msgs::msg::Pose> result_poses;

        // 测试变换到robot_base坐标系
        if (transformPosesFromLfakeGripper(test_poses, result_poses, "robot_base")) {
            RCLCPP_INFO(this->get_logger(), "位姿向量变换测试成功: 变换了%zu个位姿", result_poses.size());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "位姿向量变换测试失败");
            return false;
        }
    }
};

class planner {
public:
    planner();
    ~planner();
    enum PLAN_TYPE { LINEAR = 0, CIRCULAR };
    void plan_trajectory(planner::PLAN_TYPE plan_type, const std::vector<robo_ctrl::msg::TCPPose> waypoints);
    void plan_trajectory(planner::PLAN_TYPE plan_type, std::vector<geometry_msgs::msg::Pose> waypoints);
    void set_start_pose(const geometry_msgs::msg::Pose& start_pose);
    void set_target_pose(const geometry_msgs::msg::Pose& target_pose);
    void set_planning_count(int count) {
        planning_count_ = count;
    };
    void set_circle_center(const geometry_msgs::msg::Point& center) {
        circle_center_ = center;
    }
    void set_face_center(bool face_center) {
        face_center_ = face_center;
    }
    inline void get_trajectory(std::vector<geometry_msgs::msg::Pose>& trajectory) const {
        trajectory = trajectory_;
    }
    void get_trajectory(std::vector<robo_ctrl::msg::TCPPose>& trajectory) const {
        trajectory.clear();
        for (const auto& pose: trajectory_) {
            robo_ctrl::msg::TCPPose tcp_pose;
            tcp_pose.x = pose.position.x * 1000.0; // 米转毫米
            tcp_pose.y = pose.position.y * 1000.0;
            tcp_pose.z = pose.position.z * 1000.0;
            tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            tcp_pose.rx = roll * 180.0 / M_PI; // 弧度转度
            tcp_pose.ry = pitch * 180.0 / M_PI;
            tcp_pose.rz = yaw * 180.0 / M_PI;
            trajectory.push_back(tcp_pose);
        }
    }

    double radius_;                      // 圆弧半径
    geometry_msgs::msg::Vector3 vector_; // 圆弧原点处的切线向量
    double rotation_angle_;              // 旋转弧度 沿切向量方向
    bool x_points_to_center_;            // 是否让X轴指向圆心

private:
    geometry_msgs::msg::Pose start_pose_;
    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Pose last_intermediate_pose_;
    geometry_msgs::msg::Point circle_center_;
    int planning_count_;
    bool face_center_;
    std::vector<geometry_msgs::msg::Pose> trajectory_;
    bool planning_done;

    void linear_planning(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& target_pose,
        std::vector<geometry_msgs::msg::Pose>& trajectory
    );

    void circular_planning(
        double radius,
        geometry_msgs::msg::Vector3& vector,
        double rotation_angle,
        bool x_points_to_center,
        std::vector<geometry_msgs::msg::Pose>& trajectory
    );
};
