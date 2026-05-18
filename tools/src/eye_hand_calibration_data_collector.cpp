#include "tools/eye_hand_calibration_data_collector.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>

namespace utils {

EyeHandCalibrationDataCollector::EyeHandCalibrationDataCollector(const rclcpp::NodeOptions& options):
    Node("eye_hand_calibration_data_collector", options),
    sample_count_(0) {
    // 获取参数
    save_path_             = this->declare_parameter<std::string>("save_path", "/tmp/eye_hand_calibration_data");
    image_topic_           = this->declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    robot_state_topic_     = this->declare_parameter<std::string>("robot_state_topic", "L/robot_state");
    auto_capture_          = this->declare_parameter<bool>("auto_capture", false);
    auto_capture_interval_ = this->declare_parameter<int>("auto_capture_interval", 2000); // 默认2秒
    camera_frame_id_       = this->declare_parameter<std::string>("camera_frame_id", "camera_color_optical_frame");
    robot_base_frame_id_   = this->declare_parameter<std::string>("robot_base_frame_id", "robot_base_link");
    tcp_only_mode_         = this->declare_parameter<bool>("tcp_only_mode", true); // 默认true，保存图像和TCP

    robot_state_topic_ = this->get_parameter("robot_state_topic").as_string();
    RCLCPP_INFO(this->get_logger(),"Using RobotState topic: %s", robot_state_topic_.c_str());

    // 创建保存目录
    if (!std::filesystem::exists(save_path_)) {
        if (!std::filesystem::create_directories(save_path_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create save directory: %s", save_path_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_path_.c_str());
        }
    }

    // 根据模式创建相应的子目录
    if (!tcp_only_mode_) {
        // 创建图像子目录
        std::string image_dir = save_path_ + "/images";
        if (!std::filesystem::exists(image_dir)) {
            if (!std::filesystem::create_directories(image_dir)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create image directory: %s", image_dir.c_str());
            }
        }
    }

    // 创建位姿子目录
    std::string pose_dir = save_path_ + "/poses";
    if (!std::filesystem::exists(pose_dir)) {
        if (!std::filesystem::create_directories(pose_dir)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pose directory: %s", pose_dir.c_str());
        }
    }

    // 初始化TF监听器
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 根据模式设置订阅方式
    if (tcp_only_mode_) {
        RCLCPP_INFO(
            this->get_logger(),
            "TCP-only mode enabled. Only subscribing to RobotState topic: %s",
            robot_state_topic_.c_str()
        );

        // TCP-only模式：只订阅机器人状态话题
        robot_state_sub_.subscribe(this, robot_state_topic_);

        // 为robot_state_sub_设置回调函数
        robot_state_sub_.registerCallback(
            std::bind(&EyeHandCalibrationDataCollector::robotStateCallback, this, std::placeholders::_1)
        );
    } else {
        // 设置消息订阅和同步方式
        RCLCPP_INFO(
            this->get_logger(),
            "Using message synchronizer with Image topic: %s and RobotState topic: %s",
            image_topic_.c_str(),
            robot_state_topic_.c_str()
        );

        // 订阅图像话题和机器人状态话题
        image_sub_.subscribe(this, image_topic_);
        robot_state_sub_.subscribe(this, robot_state_topic_);

        // 配置同步器
        synchronizer_ =
            std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), image_sub_, robot_state_sub_);

        // 设置时间同步的容忍度和回调函数
        synchronizer_->setAgePenalty(0.3);
        synchronizer_->registerCallback(std::bind(
            &EyeHandCalibrationDataCollector::syncCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));
    }

    // 创建触发服务
    capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "capture_calibration_sample",
        std::bind(&EyeHandCalibrationDataCollector::captureCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 如果启用了自动采集，则创建定时器
    if (auto_capture_) {
        RCLCPP_INFO(this->get_logger(), "Auto capture enabled with interval %d ms", auto_capture_interval_);
        auto_capture_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(auto_capture_interval_),
            std::bind(&EyeHandCalibrationDataCollector::autoCaptureTick, this)
        );
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Eye-hand calibration data collector initialized in %s mode. Use '%s/capture_calibration_sample' service to trigger data capture.",
        tcp_only_mode_ ? "TCP-only" : "Image+TCP",
        this->get_fully_qualified_name()
    );
}

EyeHandCalibrationDataCollector::~EyeHandCalibrationDataCollector() {
    RCLCPP_INFO(
        this->get_logger(),
        "Eye-hand calibration data collector shutting down. Collected %d samples.",
        sample_count_
    );
}

void EyeHandCalibrationDataCollector::syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const robo_ctrl::msg::RobotState::ConstSharedPtr& robot_state_msg
) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_image_ = std::make_shared<sensor_msgs::msg::Image>(*image_msg);

    // 从RobotState中获取TCPPose，并转换为PoseStamped
    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(tcpPoseToPoseStamped(robot_state_msg->tcp_pose));

    RCLCPP_DEBUG(
        this->get_logger(),
        "Received synchronized image and robot state data. Image timestamp: %d.%09d, Robot state timestamp: %d.%09d",
        image_msg->header.stamp.sec,
        image_msg->header.stamp.nanosec,
        robot_state_msg->header.stamp.sec,
        robot_state_msg->header.stamp.nanosec
    );
}

void EyeHandCalibrationDataCollector::robotStateCallback(
    const robo_ctrl::msg::RobotState::ConstSharedPtr& robot_state_msg
) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // TCP-only模式：只处理机器人状态数据
    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(tcpPoseToPoseStamped(robot_state_msg->tcp_pose));
    // 在TCP-only模式下不需要图像数据
    current_image_ = nullptr;

    RCLCPP_DEBUG(
        this->get_logger(),
        "Received robot state data in TCP-only mode. Robot state timestamp: %d.%09d",
        robot_state_msg->header.stamp.sec,
        robot_state_msg->header.stamp.nanosec
    );
}

void EyeHandCalibrationDataCollector::captureCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    (void)request; // 未使用的参数

    bool success = saveCalibrationSample();

    if (success) {
        response->success = true;
        response->message = "Successfully saved calibration sample " + std::to_string(sample_count_ - 1);
        RCLCPP_INFO(this->get_logger(), "Saved calibration sample %d", sample_count_ - 1);
    } else {
        response->success = false;
        response->message = "Failed to save calibration sample. Check logs for details.";
        RCLCPP_ERROR(this->get_logger(), "Failed to save calibration sample");
    }
}

void EyeHandCalibrationDataCollector::autoCaptureTick() {
    if (saveCalibrationSample()) {
        RCLCPP_INFO(this->get_logger(), "Auto captured calibration sample %d", sample_count_ - 1);
    }
}

geometry_msgs::msg::PoseStamped EyeHandCalibrationDataCollector::tcpPoseToPoseStamped(
    const robo_ctrl::msg::TCPPose& tcp_pose
) {
    geometry_msgs::msg::PoseStamped pose_stamped;

    // 设置时间戳为当前时间
    pose_stamped.header.stamp = this->now();

    // 设置坐标系
    pose_stamped.header.frame_id = robot_base_frame_id_;

    // 设置位置（注意单位转换：毫米 -> 米）
    pose_stamped.pose.position.x = tcp_pose.x / 1000.0;
    pose_stamped.pose.position.y = tcp_pose.y / 1000.0;
    pose_stamped.pose.position.z = tcp_pose.z / 1000.0;

    // 从欧拉角（度）转换为四元数
    tf2::Quaternion q;
    q.setRPY(tcp_pose.rx * M_PI / 180.0, tcp_pose.ry * M_PI / 180.0, tcp_pose.rz * M_PI / 180.0);

    // 设置姿态四元数
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    return pose_stamped;
}

bool EyeHandCalibrationDataCollector::saveCalibrationSample() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 在TCP-only模式下，不需要检查图像数据
    if (!tcp_only_mode_) {
        // 检查是否有图像数据
        if (!current_image_) {
            RCLCPP_ERROR(this->get_logger(), "No image data available!");
            return false;
        }
    }

    // 检查是否有位姿数据
    if (!current_pose_) {
        RCLCPP_ERROR(this->get_logger(), "No robot TCP pose data available!");
        return false;
    }

    try {
        // 构造文件名
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(4) << sample_count_;
        std::string idx_str = ss.str();

        std::string image_path = save_path_ + "/images/image_" + idx_str + ".png";
        std::string pose_path  = save_path_ + "/poses/pose_" + idx_str + ".json";

        // 在TCP-only模式下跳过图像保存
        if (!tcp_only_mode_) {
            // 保存图像
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(current_image_);
            cv::Mat bgr_image;
            if (cv_ptr->encoding == "rgb8") {
                cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
            } else {
                bgr_image = cv_ptr->image;
            }

            if (!cv::imwrite(image_path, bgr_image)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image to %s", image_path.c_str());
                return false;
            }
        }

        // 保存位姿信息（JSON格式）
        nlohmann::json pose_data;
        pose_data["timestamp"]        = current_pose_->header.stamp.sec * 1e9 + current_pose_->header.stamp.nanosec;
        pose_data["frame_id"]         = current_pose_->header.frame_id;
        pose_data["tcp_only_mode"]    = tcp_only_mode_;
        pose_data["position"]["x"]    = current_pose_->pose.position.x;
        pose_data["position"]["y"]    = current_pose_->pose.position.y;
        pose_data["position"]["z"]    = current_pose_->pose.position.z;
        pose_data["orientation"]["x"] = current_pose_->pose.orientation.x;
        pose_data["orientation"]["y"] = current_pose_->pose.orientation.y;
        pose_data["orientation"]["z"] = current_pose_->pose.orientation.z;
        pose_data["orientation"]["w"] = current_pose_->pose.orientation.w;

        // 尝试从TF获取相机到机器人基座的变换
        try {
            if (tf_buffer_->canTransform(camera_frame_id_, robot_base_frame_id_, tf2::TimePointZero)) {
                geometry_msgs::msg::TransformStamped transform =
                    tf_buffer_->lookupTransform(camera_frame_id_, robot_base_frame_id_, tf2::TimePointZero);

                pose_data["camera_to_base"]["translation"]["x"] = transform.transform.translation.x;
                pose_data["camera_to_base"]["translation"]["y"] = transform.transform.translation.y;
                pose_data["camera_to_base"]["translation"]["z"] = transform.transform.translation.z;
                pose_data["camera_to_base"]["rotation"]["x"]    = transform.transform.rotation.x;
                pose_data["camera_to_base"]["rotation"]["y"]    = transform.transform.rotation.y;
                pose_data["camera_to_base"]["rotation"]["z"]    = transform.transform.rotation.z;
                pose_data["camera_to_base"]["rotation"]["w"]    = transform.transform.rotation.w;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                this->get_logger(),
                "Could not get transform from %s to %s: %s",
                camera_frame_id_.c_str(),
                robot_base_frame_id_.c_str(),
                ex.what()
            );
        }

        // 将位姿数据写入JSON文件
        std::ofstream pose_file(pose_path);
        if (!pose_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open pose file for writing: %s", pose_path.c_str());
            return false;
        }

        pose_file << pose_data.dump(2);
        pose_file.close();

        // 增加样本计数
        sample_count_++;

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during sample saving: %s", e.what());
        return false;
    }
}

} // namespace utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(utils::EyeHandCalibrationDataCollector)