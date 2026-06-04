#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

class MultiStaticTFPublisher: public rclcpp::Node {
public:
    MultiStaticTFPublisher(): Node("multi_static_tf_publisher") {
        // 声明参数
        this->declare_parameter("config_file", "");

        // 获取参数
        std::string config_file = this->get_parameter("config_file").as_string();

        // 创建静态TF广播器
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 如果提供了配置文件，从YAML读取变换参数
        if (!config_file.empty()) {
            RCLCPP_INFO(this->get_logger(), "尝试从配置文件加载参数: %s", config_file.c_str());
            try {
                loadTransformsFromYAML(config_file);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "读取YAML文件失败: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "未提供配置文件路径，请通过参数config_file指定");
        }
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    void loadTransformsFromYAML(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);

        // 确保transforms是一个序列/数组
        if (!config["transforms"] || !config["transforms"].IsSequence()) {
            RCLCPP_ERROR(this->get_logger(), "配置文件必须包含transforms数组");
            return;
        }

        // 遍历所有变换
        for (const auto& transform: config["transforms"]) {
            // 默认值
            std::string parent_frame = "base_link";
            std::string child_frame  = "child_frame";
            double x = 0.0, y = 0.0, z = 0.0;
            double roll = 0.0, pitch = 0.0, yaw = 0.0;

            // 读取坐标系名称
            if (transform["frames"]) {
                if (transform["frames"]["parent"])
                    parent_frame = transform["frames"]["parent"].as<std::string>();
                if (transform["frames"]["child"])
                    child_frame = transform["frames"]["child"].as<std::string>();
            }

            // 读取位置数据
            if (transform["translation"]) {
                if (transform["translation"]["x"])
                    x = transform["translation"]["x"].as<double>();
                if (transform["translation"]["y"])
                    y = transform["translation"]["y"].as<double>();
                if (transform["translation"]["z"])
                    z = transform["translation"]["z"].as<double>();
            }

            // 读取旋转数据（角度）并转换为弧度
            if (transform["rotation"]) {
                if (transform["rotation"]["roll"])
                    roll = transform["rotation"]["roll"].as<double>() * M_PI / 180.0;
                if (transform["rotation"]["pitch"])
                    pitch = transform["rotation"]["pitch"].as<double>() * M_PI / 180.0;
                if (transform["rotation"]["yaw"])
                    yaw = transform["rotation"]["yaw"].as<double>() * M_PI / 180.0;
            }

            // 创建并发布静态变换
            publishStaticTransform(parent_frame, child_frame, x, y, z, roll, pitch, yaw);
        }
    }

    void publishStaticTransform(
        const std::string& parent_frame,
        const std::string& child_frame,
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw
    ) {
        // 创建变换消息
        geometry_msgs::msg::TransformStamped static_transform_stamped;
        static_transform_stamped.header.stamp            = this->now();
        static_transform_stamped.header.frame_id         = parent_frame;
        static_transform_stamped.child_frame_id          = child_frame;
        static_transform_stamped.transform.translation.x = x;
        static_transform_stamped.transform.translation.y = y;
        static_transform_stamped.transform.translation.z = z;

        // 从RPY角度（弧度）转换为四元数
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        static_transform_stamped.transform.rotation.x = quat.x();
        static_transform_stamped.transform.rotation.y = quat.y();
        static_transform_stamped.transform.rotation.z = quat.z();
        static_transform_stamped.transform.rotation.w = quat.w();

        // 发布静态变换
        tf_static_broadcaster_->sendTransform(static_transform_stamped);

        RCLCPP_INFO(
            this->get_logger(),
            "已发布从 %s 到 %s 的静态TF变换: [%f, %f, %f] [%f°, %f°, %f°]",
            parent_frame.c_str(),
            child_frame.c_str(),
            x,
            y,
            z,
            roll * 180.0 / M_PI,
            pitch * 180.0 / M_PI,
            yaw * 180.0 / M_PI
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiStaticTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}