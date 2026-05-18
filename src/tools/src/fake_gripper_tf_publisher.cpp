#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class FakeGripperTfPublisher: public rclcpp::Node {
public:
    FakeGripperTfPublisher(): Node("fake_gripper_tf_publisher") {
        // 声明参数
        this->declare_parameter("gripper_frame", "gripper_link");
        this->declare_parameter("base_frame", "robot_base");
        this->declare_parameter("fake_frame", "fake_gripper_frame");
        this->declare_parameter("reference_frame", "world");
        this->declare_parameter("rate", 100.0);
        this->declare_parameter("robot_name", "robot");

        // 获取参数
        gripper_frame_         = this->get_parameter("gripper_frame").as_string();
        base_frame_            = this->get_parameter("base_frame").as_string();
        fake_frame_            = this->get_parameter("fake_frame").as_string();
        reference_frame_       = this->get_parameter("reference_frame").as_string();
        double rate            = this->get_parameter("rate").as_double();
        std::string robot_name = this->get_parameter("robot_name").as_string();

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing fake TF from %s with orientation from %s",
            gripper_frame_.c_str(),
            base_frame_.c_str()
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Reference frame: %s, Fake frame: %s",
            reference_frame_.c_str(),
            fake_frame_.c_str()
        );

        // 创建TF缓冲区和监听器
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&FakeGripperTfPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        try {
            // 获取夹爪相对于参考坐标系的变换
            geometry_msgs::msg::TransformStamped gripper_trans =
                tf_buffer_->lookupTransform(reference_frame_, gripper_frame_, tf2::TimePointZero);

            // 创建新的变换
            geometry_msgs::msg::TransformStamped fake_trans;
            fake_trans.header.stamp    = this->get_clock()->now();
            fake_trans.header.frame_id = reference_frame_;
            fake_trans.child_frame_id  = fake_frame_;

            // 使用夹爪的位置
            fake_trans.transform.translation = gripper_trans.transform.translation;

            // 从gripper的旋转中提取z轴方向（gripper的z轴）
            tf2::Quaternion gripper_quat;
            tf2::fromMsg(gripper_trans.transform.rotation, gripper_quat);

            // 将gripper的四元数转换为旋转矩阵
            tf2::Matrix3x3 gripper_matrix(gripper_quat);

            // 提取gripper的z轴向量（第三列）
            tf2::Vector3 gripper_z_axis(gripper_matrix[0][2], gripper_matrix[1][2], gripper_matrix[2][2]);

            // 新坐标系的x轴 = gripper的z轴
            tf2::Vector3 fake_x_axis = gripper_z_axis.normalized();

            // 新坐标系的z轴 = 参考坐标系的z轴 (0, 0, 1)
            tf2::Vector3 fake_z_axis(0.0, 0.0, 1.0);

            // 新坐标系的y轴 = z叉乘x（右手定则）
            tf2::Vector3 fake_y_axis = fake_z_axis.cross(fake_x_axis).normalized();

            // 重新计算x轴，确保正交性（y叉乘z）
            fake_x_axis = fake_y_axis.cross(fake_z_axis).normalized();

            // 构建旋转矩阵
            tf2::Matrix3x3 fake_rotation_matrix(
                fake_x_axis.x(),
                fake_y_axis.x(),
                fake_z_axis.x(),
                fake_x_axis.y(),
                fake_y_axis.y(),
                fake_z_axis.y(),
                fake_x_axis.z(),
                fake_y_axis.z(),
                fake_z_axis.z()
            );

            // 转换为四元数
            tf2::Quaternion fake_quat;
            fake_rotation_matrix.getRotation(fake_quat);
            fake_quat.normalize();

            // 设置旋转
            fake_trans.transform.rotation = tf2::toMsg(fake_quat);

            // 发布变换
            tf_broadcaster_->sendTransform(fake_trans);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    std::string gripper_frame_;
    std::string base_frame_;
    std::string fake_frame_;
    std::string reference_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeGripperTfPublisher>());
    rclcpp::shutdown();
    return 0;
}