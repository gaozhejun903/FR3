#include "detector/detector_node.hpp"

namespace detector {

detector_node::detector_node(const rclcpp::NodeOptions& options): Node("detector_node", options) {
// 声明参数
#ifdef USING_YOLOV8
    this->declare_parameter("model_path", "/home/phoenix/roboarm/FairinoDualArm/src/detector/asserts/best_l.enging");
    this->declare_parameter("plugin_path", "/home/phoenix/tensorrtx/yolov8/build/libmyplugins.so");
#endif
#ifdef USING_LW_DETR
    this->declare_parameter(
        "model_path",
        "/home/phoenix/roboarm/FairinoDualArm/src/detector/asserts/inference_model.engine"
    );
    this->declare_parameter(
        "plugin_path",
        "/home/phoenix/roboarm/FairinoDualArm/src/detector/asserts/layernorm_plugin.so"
    );
#endif
    this->declare_parameter("confidence_threshold", 0.5);
    this->declare_parameter("image_topic", "/camera/color/image_raw");
    this->declare_parameter("detections_topic", "/detector/detections");

    // 获取参数
    model_path_                  = this->get_parameter("model_path").as_string();
    plugin_path_                 = this->get_parameter("plugin_path").as_string();
    confidence_threshold_        = this->get_parameter("confidence_threshold").as_double();
    std::string image_topic      = this->get_parameter("image_topic").as_string();
    std::string detections_topic = this->get_parameter("detections_topic").as_string();

// 初始化YOLOv8推理对象
#ifdef USING_YOLOV8
    yolov8_detector_ = std::make_unique<YoloV8>();
    int ret          = yolov8_detector_->init(model_path_, plugin_path_);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "YOLOv8初始化失败，错误码: %d", ret);
        throw std::runtime_error("YOLOv8初始化失败");
    }
    RCLCPP_INFO(this->get_logger(), "YOLOv8初始化成功，模型路径: %s", model_path_.c_str());
#endif
#ifdef USING_LW_DETR
    yolov8_detector_ = std::make_unique<LwDetr>();
    int ret          = yolov8_detector_->init(model_path_);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "LwDetr初始化失败，错误码: %d", ret);
        throw std::runtime_error("LwDetr初始化失败");
    }
    RCLCPP_INFO(this->get_logger(), "LwDetr初始化成功，模型路径: %s", model_path_.c_str());
#endif

    // 创建发布器和订阅器
    detections_pub_ = this->create_publisher<detector::msg::Bbox2dArray>(detections_topic, 10);
    // 添加图像结果发布器
    detection_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(detections_topic + "/image", 10);
    image_sub_           = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic,
        10,
        std::bind(&detector_node::image_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "检测器节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "置信度阈值: %.2f", confidence_threshold_);
    RCLCPP_INFO(this->get_logger(), "订阅图像话题: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "发布检测结果话题: %s", detections_topic.c_str());
}

detector_node::~detector_node() {
    RCLCPP_INFO(this->get_logger(), "检测器节点销毁");
}

void detector_node::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto start_time = std::chrono::high_resolution_clock::now();
    // 使用互斥锁保护推理过程
    std::lock_guard<std::mutex> lock(inference_mutex_);

    try {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image                = cv_ptr->image;
        // 创建一个图像副本用于绘制检测结果
        cv::Mat visualization_image = image.clone();

        // 准备输入图像向量
        std::vector<cv::Mat> input_images;
        input_images.push_back(image);

        // 执行YOLOv8推理
        auto [processed_images, inference_time, boxes, scores, class_ids, extra_info] =
            yolov8_detector_->infer(input_images);

        // 创建自定义边界框消息
        auto detections_msg = std::make_unique<detector::msg::Bbox2dArray>();

        // 只处理第一张图的结果（因为我们只输入了一张图）
        if (!boxes.empty()) {
            const auto& img_boxes     = boxes[0];
            const auto& img_scores    = scores[0];
            const auto& img_class_ids = class_ids[0];

            // 添加每个检测框到结果消息
            for (size_t i = 0; i < img_boxes.size() / 4; ++i) {
                // 检查置信度阈值
                if (img_scores[i] < confidence_threshold_) {
                    continue;
                }

                // 创建单个Bbox2d消息
                detector::msg::Bbox2d bbox;

                // 设置边界框
                float xmin = img_boxes[i * 4];
                float ymin = img_boxes[i * 4 + 1];
                float xmax = img_boxes[i * 4 + 2];
                float ymax = img_boxes[i * 4 + 3];

                // 计算中心点和尺寸
                bbox.x        = (xmin + xmax) / 2.0f;
                bbox.y        = (ymin + ymax) / 2.0f;
                bbox.width    = xmax - xmin;
                bbox.height   = ymax - ymin;
                bbox.class_id = img_class_ids[i];
                bbox.score    = img_scores[i];
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "检测到目标: 类别ID: %d, 置信度: %.2f, 边界框: [%.2f, %.2f, %.2f, %.2f]",
                    img_class_ids[i],
                    img_scores[i],
                    xmin,
                    ymin,
                    xmax,
                    ymax
                );

                // 添加到检测结果数组
                detections_msg->results.push_back(bbox);
                detections_msg->header.stamp    = msg->header.stamp;
                detections_msg->header.frame_id = "camera_color_frame";

                // 在可视化图像上绘制检测框
                cv::Rect rect(
                    static_cast<int>(xmin),
                    static_cast<int>(ymin),
                    static_cast<int>(xmax - xmin),
                    static_cast<int>(ymax - ymin)
                );

                // 为不同类别选择不同颜色
                cv::Scalar color;
                switch (img_class_ids[i] % 6) {
                    case 0:
                        color = cv::Scalar(255, 0, 0);
                        break; // 蓝色
                    case 1:
                        color = cv::Scalar(0, 255, 0);
                        break; // 绿色
                    case 2:
                        color = cv::Scalar(0, 0, 255);
                        break; // 红色
                    case 3:
                        color = cv::Scalar(255, 255, 0);
                        break; // 青色
                    case 4:
                        color = cv::Scalar(255, 0, 255);
                        break; // 洋红色
                    case 5:
                        color = cv::Scalar(0, 255, 255);
                        break; // 黄色
                    default:
                        color = cv::Scalar(255, 255, 255); // 白色
                }

                // 绘制矩形框
                cv::rectangle(visualization_image, rect, color, 2);

                // 准备标签文本（类别ID和置信度）
                std::ostringstream oss;
                oss << "Class: " << img_class_ids[i] << " (" << std::fixed << std::setprecision(2) << img_scores[i]
                    << ")";
                std::string label = oss.str();

                // 绘制文本背景
                int baseline       = 0;
                cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                cv::rectangle(
                    visualization_image,
                    cv::Point(rect.x, rect.y - text_size.height - 5),
                    cv::Point(rect.x + text_size.width, rect.y),
                    color,
                    -1
                );

                // 绘制文本标签
                cv::putText(
                    visualization_image,
                    label,
                    cv::Point(rect.x, rect.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 0, 0),
                    1
                );
            }
        }

        // 发布检测结果
        if (detections_msg->results.empty()) {
        } else {
            RCLCPP_DEBUG(this->get_logger(), "检测到 %zu 个目标", detections_msg->results.size());
            detections_pub_->publish(std::move(detections_msg));
        }
        // 发布带有检测结果的图像
        if (detection_image_pub_->get_subscription_count() > 0) {
            // 只有当有订阅者时才发布图像，以节省带宽
            sensor_msgs::msg::Image::SharedPtr detection_image_msg =
                cv_bridge::CvImage(msg->header, "bgr8", visualization_image).toImageMsg();
            detection_image_pub_->publish(*detection_image_msg);
        }
        auto end_time                          = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        // 输出检测统计信息
        RCLCPP_DEBUG(
            this->get_logger(),
            "检测完成: %zu个目标，推理时间: %.2f ms",
            detections_msg->results.size(),
            inference_time
        );
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV桥接异常: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理图像时发生异常: %s", e.what());
    }
}

} // namespace detector

// 注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector::detector_node)