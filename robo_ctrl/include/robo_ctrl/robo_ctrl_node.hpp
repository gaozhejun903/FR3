#ifndef ROBO_CTRL_NODE_HPP
#define ROBO_CTRL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "libfairino/robot.h"
#include "robo_ctrl/srv/robot_move.hpp"
#include "robo_ctrl/srv/robot_move_cart.hpp"
#include "robo_ctrl/srv/robot_servo.hpp" // 添加新的机器人Servo服务头文件
#include "robo_ctrl/msg/robot_state.hpp"
#include "robo_ctrl/srv/robot_set_speed.hpp"
#include "robo_ctrl/srv/robot_servo_line.hpp"
#include "robo_ctrl/srv/robot_servo_joint.hpp"
#include <memory>
#include <string>
#include <vector>
#include <thread> // 添加线程支持
#include <atomic> // 添加原子变量支持
#include <mutex>  // 添加互斥锁支持
// 添加tf2相关头文件
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
namespace robo_ctrl {

enum SERVO_COMMAND { SERVO_MOVE_START = 0, SERVO_MOVE_STOP, SERVO_J, SERVO_CART };

class RoboCtrlNode: public rclcpp::Node {
public:
    explicit RoboCtrlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~RoboCtrlNode();

private:
    // 服务回调函数
    void handle_robot_move(
        const std::shared_ptr<robo_ctrl::srv::RobotMove::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotMove::Response> response
    );

    // 服务回调函数 - MoveCart
    void handle_robot_move_cart(
        const std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotMoveCart::Response> response
    );

    // 服务回调函数 - RobotServo
    void handle_robot_servo(
        const std::shared_ptr<robo_ctrl::srv::RobotServo::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotServo::Response> response
    );

    void handle_robot_set_speed(
        const std::shared_ptr<robo_ctrl::srv::RobotSetSpeed::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotSetSpeed::Response> response
    );

    void handle_robot_servo_line(
        const std::shared_ptr<robo_ctrl::srv::RobotServoLine::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotServoLine::Response> response
    );

    void handle_robot_servo_joint(
        const std::shared_ptr<robo_ctrl::srv::RobotServoJoint::Request> request,
        std::shared_ptr<robo_ctrl::srv::RobotServoJoint::Response> response
    );

    // 机器人状态定时器回调函数
    void robot_state_timer_callback();

    // 机器人状态线程函数 - 在独立线程中运行
    void robot_state_thread_func();

    // 初始化机器人连接
    bool init_robot_connection();

    // 发布TF变换
    void publish_tf_transforms(const robo_ctrl::msg::TCPPose& tcp_pose);

    // 处理MoveCart错误并尝试消除
    bool handle_move_cart_error(int error_code);

    // 发布MoveCart目标位置TF
    void publish_move_cart_target_tf(const robo_ctrl::msg::TCPPose& target_pose);

    // 机器人对象
    std::unique_ptr<FRRobot> robot_;

    // 机器人连接参数
    std::string robot_ip_;
    int robot_port_;
    bool is_connected_;
    std::string robot_name_;

    // 服务
    rclcpp::Service<robo_ctrl::srv::RobotMove>::SharedPtr robot_move_service_;
    rclcpp::Service<robo_ctrl::srv::RobotMoveCart>::SharedPtr robot_move_cart_service_;
    rclcpp::Service<robo_ctrl::srv::RobotServo>::SharedPtr robot_servo_service_; // 添加Servo服务
    rclcpp::Service<robo_ctrl::srv::RobotSetSpeed>::SharedPtr robot_set_speed_service_;
    rclcpp::Service<robo_ctrl::srv::RobotServoLine>::SharedPtr robot_servo_line_service_;
    rclcpp::Service<robo_ctrl::srv::RobotServoJoint>::SharedPtr robot_servo_joint_service_;

    // 定时器
    rclcpp::TimerBase::SharedPtr robot_state_timer_;

    // 发布器
    rclcpp::Publisher<robo_ctrl::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // TF广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 上次MoveCart目标位置，用于错误处理
    robo_ctrl::msg::TCPPose last_move_cart_target_;
    bool has_move_cart_target_ = false;

    // 状态查询间隔
    double state_query_interval_;

    // servoline相关成员
    std::atomic<bool> is_servo_running_ { false };
    std::thread servo_thread_;

    // 线程相关成员
    std::thread state_thread_;         // 状态读取线程
    std::atomic<bool> thread_running_; // 线程运行标志
    std::mutex robot_mutex_;           // 机器人对象访问互斥锁
    std::mutex servo_thread_mutex_;
};

} // namespace robo_ctrl

#endif // ROBO_CTRL_NODE_HPP
