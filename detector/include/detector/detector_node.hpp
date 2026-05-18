#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP
// # define USING_YOLOV8
#define USING_LW_DETR

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <sstream>
#include <iomanip>
#include "detector/lw_detr.hpp"

#include "detector/yolov8.hpp"
#include "detector/msg/bbox2d_array.hpp"
#include "detector/msg/bbox2d.hpp"

namespace detector {

class detector_node: public rclcpp::Node {
public:
    explicit detector_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~detector_node();

private:
    // 回调函数处理接收到的图像
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

// YOLOv8推理对象
#ifdef USING_YOLOV8
    std::unique_ptr<YoloV8> yolov8_detector_;
#endif
// LwDetr推理对象
#ifdef USING_LW_DETR
    std::unique_ptr<LwDetr> yolov8_detector_;
#endif

    // 发布器、订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<detector::msg::Bbox2dArray>::SharedPtr detections_pub_;
    // 添加图像发布器，用于发布带有检测结果的图像
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_pub_;

    // 参数
    std::string model_path_;
    std::string plugin_path_;
    double confidence_threshold_;
    bool detect_bottle_; // true: 检测瓶子, false: 检测其他物品

    // 互斥锁保护推理过程
    std::mutex inference_mutex_;
};

} // namespace detector

#endif // DETECTOR_NODE_HPP