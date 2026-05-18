#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "detector/msg/bbox2d_array.hpp"
#include "detector/msg/bbox2d.hpp"
#include "depth_handler/msg/bbox3d.hpp"
#include "depth_handler/msg/bbox3d_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "depth_handler/cluster.hpp"
#include <mutex>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// 添加TF2相关头文件
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp> // 更新为推荐的头文件路径
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace depth_handler {
class depth_node: public rclcpp::Node {
public:
    depth_node(const rclcpp::NodeOptions& options);
    ~depth_node();

private:
    // ROS2 参数声明
    void declare_parameters();
    // 参数获取与更新
    void update_parameters();

    rclcpp::Publisher<depth_handler::msg::Bbox3dArray>::SharedPtr bbox3d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    message_filters::Subscriber<detector::msg::Bbox2dArray> bbox_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<detector::msg::Bbox2dArray, sensor_msgs::msg::Image>>>
        sync_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr temp_camera_info_sub_;
    sensor_msgs::msg::CameraInfo camera_info_;
    bool camera_info_received_ = false;
    std::vector<Eigen::Vector3f> pixel_directions_;
    std::mutex camera_info_mutex_;

    // TF2相关变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_ = "Lrobot_base";         // 目标坐标系
    std::string source_frame_ = "camera_depth_frame"; // 相机坐标系

    // 主题配置
    std::string camera_info_topic_         = "/camera/depth/camera_info";
    std::string depth_topic_               = "/camera/depth/image_raw";
    std::string image_topic_               = "/camera/color/image_raw";
    std::string bbox3d_topic_              = "/depth_handler/bbox3d";
    std::string result_visulization_topic_ = "/depth_handler/result_visualization";
    std::string bbox2d_topic_              = "/detector/detections";
    std::string pointcloud_topic_          = "/depth_handler/pointcloud";
    std::string visualization_topic_       = "/depth_handler/visualization";

    // 可视化配置
    bool enable_visualization_ = true;
    bool enable_pointcloud_    = true;
    double marker_lifetime_    = 0.3;  // 可视化标记存在时间（秒）
    double marker_scale_       = 0.05; // 可视化标记大小

    // 深度处理相关参数
    float depth_scale_       = 0.001f; // 深度图像缩放因子，默认为0.001（毫米转米）
    int min_points_          = 50;     // 最小点云数量，用于滤除噪声
    float outlier_threshold_ = 0.05f;  // 异常值阈值，用于点云过滤

    // 修改数组声明，使用 std::array 而不是 auto 初始化列表
    std::array<float, 9> r = { 0.9999948740005493,     0.0013504032976925373,  -0.002899251179769635,
                               -0.0013477675383910537, 0.9999986886978149,     0.0009108961676247418,
                               0.0029004772659391165,  -0.0009069840307347476, 0.9999954104423523 };
    std::array<float, 3> t = { -0.014382616996765137, -0.00026784500479698183, -0.0017295591831207274 };

    // 参数服务回调处理
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter>& parameters);

    void callback(
        const std::shared_ptr<const detector::msg::Bbox2dArray>& detectmsg,
        const std::shared_ptr<const sensor_msgs::msg::Image>& depthmsg
    );

    /*
     * @brief Callback function for camera info
     * @param msg CameraInfo message
     * @note This function is called once when the camera info is received.
     *       It precomputes pixel directions and stores the camera info.
     */
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received CameraInfo once");

        // 保存或处理 camera_info
        camera_info_          = *msg;
        camera_info_received_ = true;
        // 计算像素方向
        pixel_directions_ = precomputePixelDirections(camera_info_.width, camera_info_.height, camera_info_);
        RCLCPP_INFO(this->get_logger(), "CameraInfo received, pixel directions precomputed");

        // 收到一次后，取消订阅
        temp_camera_info_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "CameraInfo subscription destroyed");
    }

    /*
     * @brief Precompute pixel directions for the camera
     * @param width Image width
     * @param height Image height
     * @param camera_info CameraInfo message
     * @return Vector of pixel directions in camera coordinates
     * @note The pixel directions are computed based on the camera intrinsic parameters.
     */
    std::vector<Eigen::Vector3f>
    precomputePixelDirections(int width, int height, const sensor_msgs::msg::CameraInfo& camera_info) {
        float fx, fy, cx, cy;
        fx = camera_info.k[0];
        fy = camera_info.k[4];
        cx = camera_info.k[2];
        cy = camera_info.k[5];
        std::cout << "fx: " << fx << ", fy: " << fy << ", cx: " << cx << ", cy: " << cy << std::endl;
        std::vector<Eigen::Vector3f> directions(width * height);

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                float x                   = (u - cx) / fx;
                float y                   = (v - cy) / fy;
                directions[v * width + u] = Eigen::Vector3f(x, y, 1.0f);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Pixel directions precomputed");
        return directions;
    }

    /*
     * @brief Convert depth image to 3D points in camera coordinates
     * @param depth_img Depth image
     * @param pixel_directions Precomputed pixel directions
     * @param depth_scale Scale factor for depth values (e.g., 0.001 for mm to m)
     * @return Vector of 3D points in camera coordinates
     * @note The depth image is assumed to be in meters. If the depth image is in millimeters,
     *       you can use a depth_scale of 0.001 to convert it to meters.
     */

    std::vector<Eigen::Vector3f> depthToPoints(
        const cv::Mat& depth_img,
        const std::vector<Eigen::Vector3f>& pixel_directions,
        float depth_scale = 1.0f
    );
    /*
     * @brief Convert depth image to 3D points in camera coordinates
     * @param depth_img Depth image
     * @param pixel_directions Precomputed pixel directions
     * @param roi Region of interest in the image
     * @param depth_scale Scale factor for depth values (e.g., 0.001 for mm to m)
     * @return Vector of 3D points in camera coordinates
     */
    std::vector<Eigen::Vector3f> depthToPoints(
        const cv::Mat& depth_img,
        const std::vector<Eigen::Vector3f>& pixel_directions,
        cv::Rect roi,
        float depth_scale = 1.0f
    );

    /*
     * @brief Convert depth image to 3D points in camera coordinates
     * @param depth_img Depth image
     * @param pixel_directions Precomputed pixel directions
     * @param roi Region of interest in the image
     * @param current_points Current points to be updated
     * @param depth_scale Scale factor for depth values (e.g., 0.001 for mm to m)
     * @return Vector of 3D points in camera coordinates
     */
    std::vector<Eigen::Vector3f> depthToPoints(
        const cv::Mat& depth_img,
        const std::vector<Eigen::Vector3f>& pixel_directions,
        cv::Rect roi,
        std::vector<Eigen::Vector3f> current_points,
        float depth_scale = 1.0f
    );

    /*
     * @brief Scale a bounding box to the image size
     * @param bbox Bounding box to scale
     * @param width Width of the image
     * @param height Height of the image
     * @param img_width Original width of the image
     * @param img_height Original height of the image
     * @return Scaled bounding box
     */
    inline cv::Rect
    scale_bbox(const cv::Rect& bbox, const int width, const int height, const int img_width, const int img_height) {
        cv::Rect scaled_bbox;
        scaled_bbox.x      = bbox.x * width / img_width;
        scaled_bbox.y      = bbox.y * height / img_height;
        scaled_bbox.width  = bbox.width * width / img_width;
        scaled_bbox.height = bbox.height * height / img_height;
        return scaled_bbox;
    }

    /*
     * @brief Convert a vector of Eigen::Vector3f points to a PointCloud2 message
     * @param points Vector of 3D points
     * @param pointcloud_msg PointCloud2 message to fill
     * @note This function assumes that the points are in camera coordinates.
     *       The PointCloud2 message will be filled with the x, y, z fields.
     *       The point cloud is assumed to be dense and in little-endian format.
     */
    inline void
    eigenToPointCloud2(const std::vector<Eigen::Vector3f>& points, sensor_msgs::msg::PointCloud2& pointcloud_msg) {
        // Set up the header
        pointcloud_msg.height       = 1;
        pointcloud_msg.width        = static_cast<uint32_t>(points.size());
        pointcloud_msg.is_dense     = true;
        pointcloud_msg.is_bigendian = false;

        // Define fields: x, y, z
        pointcloud_msg.fields.clear();
        pointcloud_msg.fields.resize(3);

        pointcloud_msg.fields[0].name     = "x";
        pointcloud_msg.fields[0].offset   = 0;
        pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[0].count    = 1;

        pointcloud_msg.fields[1].name     = "y";
        pointcloud_msg.fields[1].offset   = 4;
        pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[1].count    = 1;

        pointcloud_msg.fields[2].name     = "z";
        pointcloud_msg.fields[2].offset   = 8;
        pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[2].count    = 1;

        pointcloud_msg.point_step = 12; // 3 * sizeof(float)
        pointcloud_msg.row_step   = pointcloud_msg.point_step * pointcloud_msg.width;

        // Resize the data buffer
        pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height);

        // Direct memory copy
        uint8_t* data_ptr = pointcloud_msg.data.data();
        for (const auto& point: points) {
            std::memcpy(data_ptr, point.data(), 12); // 12 bytes: x,y,z floats
            data_ptr += 12;
        }
    }

    // 点云坐标变换函数 - 将点云从相机坐标系转换到机器人基座坐标系
    bool transformPointCloud(
        std::vector<Eigen::Vector3f>& points,
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time
    );

    // 单个点的坐标变换
    bool transformPoint(
        Eigen::Vector3f& point,
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time
    );

    // 边界框的坐标变换
    bool transformBoundingBox(
        depth_handler::msg::Bbox3d& bbox,
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time
    );
};
} // namespace depth_handler