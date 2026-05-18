#include "tools/dual_end_tf_collector.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace utils {

DualEndTfCollector::DualEndTfCollector(const rclcpp::NodeOptions& options):
    Node("dual_end_tf_collector", options),
    sample_count_(0) {
    // 获取参数
    save_path_             = this->declare_parameter<std::string>("save_path", "/tmp/dual_end_tf_data");
    auto_capture_          = this->declare_parameter<bool>("auto_capture", false);
    auto_capture_interval_ = this->declare_parameter<int>("auto_capture_interval", 2000); // 默认2秒
    tf_update_interval_    = this->declare_parameter<int>("tf_update_interval", 100);     // 默认100毫秒

    // 坐标系参数
    rend_frame_id_ = this->declare_parameter<std::string>("rend_frame_id", "Rend");
    lend_frame_id_ = this->declare_parameter<std::string>("lend_frame_id", "Lend");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "world");

    // 分别为左臂和右臂设置基座坐标系
    rend_base_frame_id_ = this->declare_parameter<std::string>("rend_base_frame_id", "Rrobot_base");
    lend_base_frame_id_ = this->declare_parameter<std::string>("lend_base_frame_id", "Lrobot_base");

    RCLCPP_INFO(this->get_logger(), "Dual End TF Collector initialized with:");
    RCLCPP_INFO(
        this->get_logger(),
        "  - Rend frame: %s (base: %s)",
        rend_frame_id_.c_str(),
        rend_base_frame_id_.c_str()
    );
    RCLCPP_INFO(
        this->get_logger(),
        "  - Lend frame: %s (base: %s)",
        lend_frame_id_.c_str(),
        lend_base_frame_id_.c_str()
    );
    RCLCPP_INFO(this->get_logger(), "  - Base frame: %s", base_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Save path: %s", save_path_.c_str());

    // 创建保存目录
    if (!std::filesystem::exists(save_path_)) {
        if (!std::filesystem::create_directories(save_path_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create save directory: %s", save_path_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_path_.c_str());
        }
    }

    // 创建双臂位姿子目录
    std::string poses_dir = save_path_ + "/dual_poses";
    if (!std::filesystem::exists(poses_dir)) {
        if (!std::filesystem::create_directories(poses_dir)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create poses directory: %s", poses_dir.c_str());
        }
    }

    // 初始化TF监听器
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建TF数据更新定时器
    tf_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tf_update_interval_),
        std::bind(&DualEndTfCollector::tfUpdateCallback, this)
    );

    // 创建触发服务
    capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "capture_dual_end_sample",
        std::bind(&DualEndTfCollector::captureCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 如果启用了自动采集，则创建定时器
    if (auto_capture_) {
        RCLCPP_INFO(this->get_logger(), "Auto capture enabled with interval %d ms", auto_capture_interval_);
        auto_capture_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(auto_capture_interval_),
            std::bind(&DualEndTfCollector::autoCaptureTick, this)
        );
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Dual End TF Collector initialized. Use '%s/capture_dual_end_sample' service to trigger data capture.",
        this->get_fully_qualified_name()
    );
}

DualEndTfCollector::~DualEndTfCollector() {
    RCLCPP_INFO(this->get_logger(), "Dual End TF Collector shutting down. Collected %d samples.", sample_count_);
}

void DualEndTfCollector::tfUpdateCallback() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 获取右臂末端位姿（从右臂基座坐标系到世界坐标系）
    geometry_msgs::msg::PoseStamped rend_pose;
    bool rend_success = getTransformAsPose(base_frame_id_, rend_frame_id_, rend_pose);

    // 获取左臂末端位姿（从左臂基座坐标系到世界坐标系）
    geometry_msgs::msg::PoseStamped lend_pose;
    bool lend_success = getTransformAsPose(base_frame_id_, lend_frame_id_, lend_pose);

    if (rend_success && lend_success) {
        current_rend_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(rend_pose);
        current_lend_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(lend_pose);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Updated dual end poses - Rend: [%.3f, %.3f, %.3f], Lend: [%.3f, %.3f, %.3f]",
            rend_pose.pose.position.x,
            rend_pose.pose.position.y,
            rend_pose.pose.position.z,
            lend_pose.pose.position.x,
            lend_pose.pose.position.y,
            lend_pose.pose.position.z
        );
    } else {
        if (!rend_success) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to get transform for Rend frame");
        }
        if (!lend_success) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to get transform for Lend frame");
        }
    }
}

void DualEndTfCollector::captureCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
) {
    (void)request; // 未使用的参数

    bool success = saveDualEndSample();

    if (success) {
        response->success = true;
        response->message = "Successfully saved dual end sample " + std::to_string(sample_count_ - 1);
        RCLCPP_INFO(this->get_logger(), "Saved dual end sample %d", sample_count_ - 1);
    } else {
        response->success = false;
        response->message = "Failed to save dual end sample. Check logs for details.";
        RCLCPP_ERROR(this->get_logger(), "Failed to save dual end sample");
    }
}

void DualEndTfCollector::autoCaptureTick() {
    if (saveDualEndSample()) {
        RCLCPP_INFO(this->get_logger(), "Auto captured dual end sample %d", sample_count_ - 1);
    }
}

bool DualEndTfCollector::saveDualEndSample() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 检查是否有双臂位姿数据
    if (!current_rend_pose_ || !current_lend_pose_) {
        RCLCPP_ERROR(
            this->get_logger(),
            "No dual end pose data available! Rend: %s, Lend: %s",
            current_rend_pose_ ? "OK" : "NULL",
            current_lend_pose_ ? "OK" : "NULL"
        );
        return false;
    }

    try {
        // 构造文件名
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(4) << sample_count_;
        std::string idx_str = ss.str();

        std::string dual_pose_path = save_path_ + "/dual_poses/dual_pose_" + idx_str + ".json";

        // 创建JSON数据
        nlohmann::json dual_pose_data;

        // 添加时间戳（使用当前时间）
        auto now                        = this->now();
        dual_pose_data["timestamp"]     = now.seconds() * 1e9 + now.nanoseconds();
        dual_pose_data["sample_index"]  = sample_count_;
        dual_pose_data["base_frame_id"] = base_frame_id_;

        // 保存右臂末端位姿
        dual_pose_data["rend_pose"]["frame_id"]         = current_rend_pose_->header.frame_id;
        dual_pose_data["rend_pose"]["position"]["x"]    = current_rend_pose_->pose.position.x;
        dual_pose_data["rend_pose"]["position"]["y"]    = current_rend_pose_->pose.position.y;
        dual_pose_data["rend_pose"]["position"]["z"]    = current_rend_pose_->pose.position.z;
        dual_pose_data["rend_pose"]["orientation"]["x"] = current_rend_pose_->pose.orientation.x;
        dual_pose_data["rend_pose"]["orientation"]["y"] = current_rend_pose_->pose.orientation.y;
        dual_pose_data["rend_pose"]["orientation"]["z"] = current_rend_pose_->pose.orientation.z;
        dual_pose_data["rend_pose"]["orientation"]["w"] = current_rend_pose_->pose.orientation.w;

        // 计算右臂末端的欧拉角
        tf2::Quaternion rend_q(
            current_rend_pose_->pose.orientation.x,
            current_rend_pose_->pose.orientation.y,
            current_rend_pose_->pose.orientation.z,
            current_rend_pose_->pose.orientation.w
        );
        tf2::Matrix3x3 rend_m(rend_q);
        double rend_roll, rend_pitch, rend_yaw;
        rend_m.getRPY(rend_roll, rend_pitch, rend_yaw);
        dual_pose_data["rend_pose"]["euler"]["roll"]  = rend_roll;
        dual_pose_data["rend_pose"]["euler"]["pitch"] = rend_pitch;
        dual_pose_data["rend_pose"]["euler"]["yaw"]   = rend_yaw;

        // 保存左臂末端位姿
        dual_pose_data["lend_pose"]["frame_id"]         = current_lend_pose_->header.frame_id;
        dual_pose_data["lend_pose"]["position"]["x"]    = current_lend_pose_->pose.position.x;
        dual_pose_data["lend_pose"]["position"]["y"]    = current_lend_pose_->pose.position.y;
        dual_pose_data["lend_pose"]["position"]["z"]    = current_lend_pose_->pose.position.z;
        dual_pose_data["lend_pose"]["orientation"]["x"] = current_lend_pose_->pose.orientation.x;
        dual_pose_data["lend_pose"]["orientation"]["y"] = current_lend_pose_->pose.orientation.y;
        dual_pose_data["lend_pose"]["orientation"]["z"] = current_lend_pose_->pose.orientation.z;
        dual_pose_data["lend_pose"]["orientation"]["w"] = current_lend_pose_->pose.orientation.w;

        // 计算左臂末端的欧拉角
        tf2::Quaternion lend_q(
            current_lend_pose_->pose.orientation.x,
            current_lend_pose_->pose.orientation.y,
            current_lend_pose_->pose.orientation.z,
            current_lend_pose_->pose.orientation.w
        );
        tf2::Matrix3x3 lend_m(lend_q);
        double lend_roll, lend_pitch, lend_yaw;
        lend_m.getRPY(lend_roll, lend_pitch, lend_yaw);
        dual_pose_data["lend_pose"]["euler"]["roll"]  = lend_roll;
        dual_pose_data["lend_pose"]["euler"]["pitch"] = lend_pitch;
        dual_pose_data["lend_pose"]["euler"]["yaw"]   = lend_yaw;

        // 计算双臂末端之间的相对位姿
        double relative_x = current_lend_pose_->pose.position.x - current_rend_pose_->pose.position.x;
        double relative_y = current_lend_pose_->pose.position.y - current_rend_pose_->pose.position.y;
        double relative_z = current_lend_pose_->pose.position.z - current_rend_pose_->pose.position.z;
        double distance   = std::sqrt(relative_x * relative_x + relative_y * relative_y + relative_z * relative_z);

        dual_pose_data["relative_info"]["distance"] = distance;
        dual_pose_data["relative_info"]["delta_x"]  = relative_x;
        dual_pose_data["relative_info"]["delta_y"]  = relative_y;
        dual_pose_data["relative_info"]["delta_z"]  = relative_z;

        // 将数据写入JSON文件
        std::ofstream pose_file(dual_pose_path);
        if (!pose_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open dual pose file for writing: %s", dual_pose_path.c_str());
            return false;
        }

        pose_file << dual_pose_data.dump(2);
        pose_file.close();

        // 增加样本计数
        sample_count_++;

        RCLCPP_INFO(
            this->get_logger(),
            "Saved dual end sample %d - Distance: %.3f m, Rend: [%.3f, %.3f, %.3f], Lend: [%.3f, %.3f, %.3f]",
            sample_count_ - 1,
            distance,
            current_rend_pose_->pose.position.x,
            current_rend_pose_->pose.position.y,
            current_rend_pose_->pose.position.z,
            current_lend_pose_->pose.position.x,
            current_lend_pose_->pose.position.y,
            current_lend_pose_->pose.position.z
        );

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during dual end sample saving: %s", e.what());
        return false;
    }
}

bool DualEndTfCollector::getTransformAsPose(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::msg::PoseStamped& pose
) {
    try {
        // 使用最新的可用时间而不是 tf2::TimePointZero
        rclcpp::Time now = this->get_clock()->now();

        // 检查是否能够获取变换，等待最多 50ms
        if (!tf_buffer_->canTransform(target_frame, source_frame, now, std::chrono::milliseconds(50))) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Cannot transform from %s to %s at time %f",
                source_frame.c_str(),
                target_frame.c_str(),
                now.seconds()
            );
            return false;
        }

        // 获取变换
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(target_frame, source_frame, now, std::chrono::milliseconds(50));

        // 转换为PoseStamped
        pose = transformToPoseStamped(transform);

        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "Could not get transform from %s to %s: %s",
            source_frame.c_str(),
            target_frame.c_str(),
            ex.what()
        );
        return false;
    }
}

geometry_msgs::msg::PoseStamped DualEndTfCollector::transformToPoseStamped(
    const geometry_msgs::msg::TransformStamped& transform
) {
    geometry_msgs::msg::PoseStamped pose;

    // 复制头部信息
    pose.header = transform.header;

    // 复制位置
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;

    // 复制姿态
    pose.pose.orientation = transform.transform.rotation;

    return pose;
}

} // namespace utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(utils::DualEndTfCollector)
