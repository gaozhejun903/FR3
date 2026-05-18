#ifndef ROBOARM_DETECT_detr
#define ROBOARM_DETECT_detr
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>
#include <dlfcn.h>
#include <NvInferPlugin.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stl_numeric.h>

namespace detector {

// 定义LwDetr类
class LwDetr {
public:
    LwDetr();
    ~LwDetr();

    /*
     * @brief 初始化模型
     * @param engine_path 模型路径
     * @param plugin_path 插件路径
     * @return 0: 成功, -1: 失败
     */
    int init(std::string engine_path, std::string plugin_path);
    int init(std::string engine_path);

    /*
     * @brief 推理
     * @param raw_image_generator 输入图像
     * @return 返回推理结果 ( 图像, 推理时间, 检测框, 置信度, 类别ID, TBD )
     */
    std::tuple<
        std::vector<cv::Mat>,
        double,
        std::vector<std::vector<float>>,
        std::vector<std::vector<float>>,
        std::vector<std::vector<int>>,
        std::vector<std::vector<float>>>
    infer(std::vector<cv::Mat>& raw_image_generator);

private:
    /*
     * @brief 图像预处理
     * @param raw_bgr_image 输入图像
     * @return 预处理后的图像
     */
    cv::Mat preprocess_image(const cv::Mat& raw_bgr_image);

    /*
     * @brief 非极大值抑制
     * @param boxes 检测框 xyxy格式
     * @param scores 置信度
     * @param classID 类别ID
     * @param threshold 阈值
     * @return 返回非极大值抑制结果 ( 检测框, 置信度, 类别ID )
     */
    std::tuple<std::vector<float>, std::vector<float>, std::vector<int>> non_max_suppression(
        std::vector<float>& boxes,
        std::vector<float> scores,
        std::vector<int> classID,
        float threshold
    );

    /*
     * @brief 后处理
     * @param dets DETR检测结果 (bbox坐标)
     * @param labels DETR标签结果 (类别概率)
     * @param origin_w 原图宽度
     * @param origin_h 原图高度
     * @return 返回后处理结果 ( 检测框, 置信度, 类别ID )
     */
    std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
    post_process(float* dets, float* labels, int origin_w, int origin_h, int num_boxes, int num_classes);

    /*
     * @brief 裁剪检测框
     * @param box 检测框
     * @param origin_h 原图高度
     * @param origin_w 原图宽度
     * @return 裁剪后的检测框
     */
    cv::Rect clip_box(const cv::Rect& box, int origin_h, int origin_w);

    /*
     * @brief 计算IoU
     * @param box1 检测框1
     * @param box2 检测框2
     * @return IoU值
     */
    float bbox_iou(const cv::Rect& box1, const cv::Rect& box2);

    nvinfer1::IExecutionContext* context;
    cudaStream_t stream;
    void** bindings;
    float* host_inputs;
    float* cuda_inputs;
    float* host_dets;   // 检测框输出
    float* host_labels; // 标签输出
    float* cuda_dets;   // GPU检测框
    float* cuda_labels; // GPU标签

    int batch_size      = 1;
    int input_h         = 640;
    int input_w         = 640;
    int num_boxes       = 100; // DETR输出的最大检测框数量
    int num_classes     = 91;  // DETR支持的类别数量
    float THRESHOLD     = 0.5; // 置信度阈值
    float NMS_THRESHOLD = 0.4; // NMS阈值
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IRuntime* runtime;
};

} // namespace detector

inline float sigmoid(float x) {
    return 1.0f / (1.0f + exp(-x));
}
#endif