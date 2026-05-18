#ifndef ROBOARM_DETECT
#define ROBOARM_DETECT
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

// Define YoloV8
class YoloV8 {
public:
    YoloV8();
    ~YoloV8();

    /*
     * @brief 初始化模型
     * @param engine_path 模型路径
     * @param plugin_path 插件路径
     * @return 0: 成功, -1: 失败
     */
    int init(std::string engine_path, std::string plugin_path);

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
     * @param output 模型输出
     * @param origin_w 原图宽度
     * @param origin_h 原图高度
     * @param output_size 输出大小
     * @return 返回后处理结果 ( 检测框, 置信度, 类别ID )
     */
    std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
    post_process(float* output, int origin_w, int origin_h, int output_size);

    /*
     * @brief 裁剪检测框
     * @param box 检测框
     * @param origin_h 原图高度
     * @param origin_w 原图宽度
     * @return 裁剪后的检测框
     */
    cv::Rect clip_box(const cv::Rect& box, int origin_h, int origin_w);

    /*
     * @brief 计算两个检测框的IOU
     * @param box1 检测框1
     * @param box2 检测框2
     * @return IOU
     */
    float bbox_iou(const cv::Rect& box1, const cv::Rect& box2);

    nvinfer1::IExecutionContext* context;
    cudaStream_t stream;
    void** bindings;
    float* host_inputs;
    float* cuda_inputs;
    float* host_outputs;
    float* cuda_outputs;
    int batch_size      = 1;
    int input_h         = 640;
    int input_w         = 640;
    int output_size     = 38001; // Adjust based on output size
    float THRESHOLD     = 0.5;
    float NMS_THRESHOLD = 0.4;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IRuntime* runtime;
    std::vector<float> old_boxes;
    // TODO: 移动到配置文件
};
} // namespace detector
#endif
