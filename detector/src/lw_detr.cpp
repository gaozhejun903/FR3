#include "detector/lw_detr.hpp"

// 定义Logger
static class Logger: public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // 只显示非INFO级别的消息
        if (severity != Severity::kINFO) {
            std::cout << msg << std::endl;
        }
    }
} gLogger;

detector::LwDetr::LwDetr() {
    // 分配模型输入内存
    host_inputs = new float[batch_size * 3 * input_h * input_w];

    // 分配模型输出内存 - DETR输出包括检测框和类别标签两部分
    host_dets   = new float[batch_size * num_boxes * 4]; // 检测框: [batch, num_boxes, 4]，每框包含[x1,y1,x2,y2]
    host_labels = new float[batch_size * num_boxes * num_classes]; // 标签: [batch, num_boxes, num_classes]
}

detector::LwDetr::~LwDetr() {
    // 释放CPU和GPU内存
    delete[] host_inputs;
    delete[] host_dets;
    delete[] host_labels;

    cudaFree(cuda_inputs);
    cudaFree(cuda_dets);
    cudaFree(cuda_labels);

    // 销毁CUDA流
    cudaStreamDestroy(stream);

    // 释放TensorRT资源
    context->destroy();
    engine->destroy();
    runtime->destroy();

    // 释放绑定
    delete[] bindings;
}

int detector::LwDetr::init(std::string engine_path, std::string plugin_path) {
    // 加载插件库
#ifndef PLUGIN_REGIST
    #define PLUGIN_REGIST
    void* handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
    if (!handle) {
        std::cerr << "错误: 加载插件库失败: " << dlerror() << std::endl;
        return -1;
    }
#endif

    // 初始化NvInfer插件
    initLibNvInferPlugins(&gLogger, "");

    // 加载引擎文件
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
        std::cerr << "错误: 无法打开引擎文件: " << engine_path << std::endl;
        return -1;
    }

    // 加载和反序列化引擎
    runtime = nvinfer1::createInferRuntime(gLogger);
    std::vector<char> engine_data((std::istreambuf_iterator<char>(engine_file)), std::istreambuf_iterator<char>());
    engine  = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
    context = engine->createExecutionContext();

    // 验证引擎输入输出
    if (engine->getNbBindings() != 3) { // 输入+2个输出(检测框和类别)
        std::cerr << "错误: LwDetr模型应有3个绑定点 (1个输入, 2个输出), 但找到 " << engine->getNbBindings()
                  << std::endl;
        return -1;
    }

    // 分配GPU内存
    cudaMalloc((void**)&cuda_inputs, batch_size * 3 * input_h * input_w * sizeof(float));
    cudaMalloc((void**)&cuda_dets, batch_size * num_boxes * 4 * sizeof(float));
    cudaMalloc((void**)&cuda_labels, batch_size * num_boxes * num_classes * sizeof(float));

    // 创建CUDA流
    cudaStreamCreate(&stream);

    // 设置输入输出绑定
    bindings    = new void*[3]; // 1个输入 + 2个输出
    bindings[0] = cuda_inputs;  // 输入图像
    bindings[1] = cuda_dets;    // 输出检测框
    bindings[2] = cuda_labels;  // 输出类别概率

    return 0;
}
int detector::LwDetr::init(std::string engine_path) {
    // 初始化NvInfer插件
    initLibNvInferPlugins(&gLogger, "");

    // 加载引擎文件
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
        std::cerr << "错误: 无法打开引擎文件: " << engine_path << std::endl;
        return -1;
    }

    // 加载和反序列化引擎
    runtime = nvinfer1::createInferRuntime(gLogger);
    std::vector<char> engine_data((std::istreambuf_iterator<char>(engine_file)), std::istreambuf_iterator<char>());
    engine  = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
    context = engine->createExecutionContext();

    // 验证引擎输入输出
    if (engine->getNbBindings() != 3) { // 输入+2个输出(检测框和类别)
        std::cerr << "错误: LwDetr模型应有3个绑定点 (1个输入, 2个输出), 但找到 " << engine->getNbBindings()
                  << std::endl;
        return -1;
    }

    // 分配GPU内存
    cudaMalloc((void**)&cuda_inputs, batch_size * 3 * input_h * input_w * sizeof(float));
    cudaMalloc((void**)&cuda_dets, batch_size * num_boxes * 4 * sizeof(float));
    cudaMalloc((void**)&cuda_labels, batch_size * num_boxes * num_classes * sizeof(float));

    // 创建CUDA流
    cudaStreamCreate(&stream);

    // 设置输入输出绑定
    bindings    = new void*[3]; // 1个输入 + 2个输出
    bindings[0] = cuda_inputs;  // 输入图像
    bindings[1] = cuda_dets;    // 输出检测框
    bindings[2] = cuda_labels;  // 输出类别概率

    return 0;
}

std::tuple<
    std::vector<cv::Mat>,
    double,
    std::vector<std::vector<float>>,
    std::vector<std::vector<float>>,
    std::vector<std::vector<int>>,
    std::vector<std::vector<float>>>
detector::LwDetr::infer(std::vector<cv::Mat>& raw_image_generator) {
    // 准备批处理输入
    std::vector<cv::Mat> batch_image_raw = {};
    std::vector<int> batch_origin_h      = {};
    std::vector<int> batch_origin_w      = {};
    std::vector<float> batch_input_image(batch_size * 3 * input_h * input_w);

    // 处理每张输入图像
    for (size_t i = 0; i < raw_image_generator.size() && i < batch_size; ++i) {
        cv::Mat image_raw = raw_image_generator[i];
        int origin_h      = image_raw.rows;
        int origin_w      = image_raw.cols;

        // 图像预处理
        cv::Mat input_image = preprocess_image(image_raw);

        // 保存原始图像和尺寸信息
        batch_image_raw.push_back(image_raw);
        batch_origin_h.push_back(origin_h);
        batch_origin_w.push_back(origin_w);

        // 复制处理后的图像数据到批处理缓冲区
        std::memcpy(
            batch_input_image.data() + i * 3 * input_h * input_w,
            input_image.data,
            3 * input_h * input_w * sizeof(float)
        );
    }
    // 将输入数据传输到GPU
    cudaMemcpyAsync(
        cuda_inputs,
        batch_input_image.data(),
        batch_size * 3 * input_h * input_w * sizeof(float),
        cudaMemcpyHostToDevice,
        stream
    );

    // 设置输入张量的维度
    nvinfer1::Dims4 inputDims { batch_size, 3, input_h, input_w };
    context->setBindingDimensions(0, inputDims);

    // 检查维度是否有效
    if (!context->allInputDimensionsSpecified()) {
        std::cerr << "错误: 未能正确指定所有输入维度" << std::endl;
        return std::make_tuple(
            batch_image_raw,
            0.0,
            std::vector<std::vector<float>> {},
            std::vector<std::vector<float>> {},
            std::vector<std::vector<int>> {},
            std::vector<std::vector<float>> {}
        );
    }

    // 执行推理
    auto start = std::chrono::high_resolution_clock::now();
    // 使用enqueueV2替代enqueue
    auto input_dims = context->getBindingDimensions(0);

    bool status = context->enqueueV2(bindings, stream, nullptr);
    if (!status) {
        std::cerr << "错误: 推理执行失败" << std::endl;
    }

    // 将结果从GPU复制回主机
    cudaMemcpyAsync(host_dets, cuda_dets, batch_size * num_boxes * 4 * sizeof(float), cudaMemcpyDeviceToHost, stream);

    cudaMemcpyAsync(
        host_labels,
        cuda_labels,
        batch_size * num_boxes * num_classes * sizeof(float),
        cudaMemcpyDeviceToHost,
        stream
    );

    // 同步等待推理和数据传输完成
    cudaStreamSynchronize(stream);
    auto err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "CUDA 错误: " << cudaGetErrorString(err) << std::endl;
    }
    auto end                               = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    // 准备结果容器
    std::vector<std::vector<float>> result_boxes  = {};
    std::vector<std::vector<float>> result_scores = {};
    std::vector<std::vector<int>> result_classID  = {};
    std::vector<std::vector<float>> det           = {};

    // 对每个批次图像进行后处理
    for (int i = 0; i < batch_size && i < raw_image_generator.size(); ++i) {
        float* dets_offset   = host_dets + i * num_boxes * 4;
        float* labels_offset = host_labels + i * num_boxes * num_classes;

        // 后处理获取检测结果
        auto post_result =
            post_process(dets_offset, labels_offset, batch_origin_w[i], batch_origin_h[i], num_boxes, num_classes);

        // 存储检测结果
        result_boxes.push_back(std::get<0>(post_result));
        result_scores.push_back(std::get<1>(post_result));
        result_classID.push_back(std::get<2>(post_result));
    }

    return std::make_tuple(batch_image_raw, duration.count(), result_boxes, result_scores, result_classID, det);
}

// cv::Mat detector::LwDetr::preprocess_image(const cv::Mat& raw_bgr_image) {
//     cv::Mat image_raw = raw_bgr_image.clone();
//     int h             = image_raw.rows;
//     int w             = image_raw.cols;

//     // 将BGR转换为RGB (DETR通常使用RGB输入)
//     cv::Mat image;
//     cv::cvtColor(image_raw, image, cv::COLOR_BGR2RGB);

//     // 计算缩放比例和填充 (DETR通常使用固定大小的输入，同时保持长宽比)
//     float scale = std::min((float)input_w / w, (float)input_h / h);
//     int new_w   = (int)(w * scale);
//     int new_h   = (int)(h * scale);

//     // 缩放图像
//     cv::Mat resized_image;
//     cv::resize(image, resized_image, cv::Size(new_w, new_h));

//     // 创建填充后的图像 (DETR通常填充为正方形)
//     cv::Mat padded_image = cv::Mat::zeros(cv::Size(input_w, input_h), CV_8UC3);

//     // 将缩放后的图像放在填充图像的左上角
//     resized_image.copyTo(padded_image(cv::Rect(0, 0, new_w, new_h)));

//     // 归一化为[0,1]范围
//     padded_image.convertTo(padded_image, CV_32FC3, 1.0 / 255.0);

//     // 标准化图像 (使用ImageNet的均值和标准差)
//     // DETR通常使用ImageNet标准化
//     std::vector<cv::Mat> channels(3);
//     cv::split(padded_image, channels);

//     // 对每个通道应用均值和标准差的标准化
//     channels[0] = (channels[0] - 0.485) / 0.229; // R通道
//     channels[1] = (channels[1] - 0.456) / 0.224; // G通道
//     channels[2] = (channels[2] - 0.406) / 0.225; // B通道

//     cv::Mat normalized_image;
//     cv::merge(channels, normalized_image);

//     // 将图像格式从HWC转换为NCHW
//     cv::Mat chw_image;
//     std::vector<cv::Mat> chw_channels(3);
//     cv::split(normalized_image, chw_channels);
//     cv::vconcat(chw_channels, chw_image);

//     // 返回预处理后的图像
//     return chw_image.reshape(1, { 1, 3, input_h, input_w });
// }

cv::Mat detector::LwDetr::preprocess_image(const cv::Mat& img) {
    cv::Mat resized, float_img, rgb_img;
    cv::resize(img, resized, cv::Size(input_w, input_h));
    cv::cvtColor(resized, rgb_img, cv::COLOR_BGR2RGB); // BGR -> RGB

    rgb_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0); // 转 float 并归一化

    // 拆分通道
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);

    // 归一化 (mean/std)
    const float mean[3] = { 0.485, 0.456, 0.406 };
    const float std[3]  = { 0.229, 0.224, 0.225 };

    for (int i = 0; i < 3; ++i) {
        channels[i] = (channels[i] - mean[i]) / std[i];
    }

    // 合并成连续 CHW 格式
    std::vector<float> chw_data(3 * input_h * input_w);
    for (int i = 0; i < 3; ++i) {
        std::memcpy(chw_data.data() + i * input_h * input_w, channels[i].data, input_h * input_w * sizeof(float));
    }

    return cv::Mat(1, 3 * input_h * input_w, CV_32FC1, chw_data.data()).clone();
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
detector::LwDetr::post_process(float* dets, float* labels, int origin_w, int origin_h, int num_boxes, int num_classes) {
    std::vector<float> boxes;
    std::vector<float> scores;
    std::vector<int> classID;

    // 处理每个检测框
    for (int i = 0; i < num_boxes; i++) {
        // 获取检测框坐标 (DETR通常输出归一化的坐标[0,1])
        float x_center = dets[i * 4];
        float y_center = dets[i * 4 + 1];
        float width    = dets[i * 4 + 2];
        float height   = dets[i * 4 + 3];

        // 如果是归一化坐标，转换为实际像素坐标
        if (x_center <= 1 && y_center <= 1 && width <= 1 && height <= 1) {
            x_center *= input_w;
            y_center *= input_h;
            width *= input_w;
            height *= input_h;
        }

        // 转换为左上角和右下角坐标 (XYXY格式)
        float x1 = x_center - width / 2.0;
        float y1 = y_center - height / 2.0;
        float x2 = x_center + width / 2.0;
        float y2 = y_center + height / 2.0;

        // 将坐标映射回原始图像尺寸 - 直接采用等比例缩放
        // 预处理中使用 cv::resize 进行等比例缩放，这里采用相同方式进行逆变换
        float scale_w = (float)input_w / (float)origin_w;
        float scale_h = (float)input_h / (float)origin_h;

        // 计算实际缩放后的尺寸（与预处理一致）
        int scaled_w = origin_w * scale_w;
        int scaled_h = origin_h * scale_h;

        // 将坐标从模型输入尺寸映射回原始图像尺寸
        x1 = x1 / scale_w;
        y1 = y1 / scale_h;
        x2 = x2 / scale_w;
        y2 = y2 / scale_h;

        // 限制坐标在图像边界内
        x1 = std::max(0.0f, std::min(x1, (float)origin_w));
        y1 = std::max(0.0f, std::min(y1, (float)origin_h));
        x2 = std::max(0.0f, std::min(x2, (float)origin_w));
        y2 = std::max(0.0f, std::min(y2, (float)origin_h));

        // 找出最高置信度的类别
        int best_class   = -1;
        float best_score = 0;
        for (int j = 0; j < num_classes; j++) {
            float score = sigmoid(labels[i * num_classes + j]);
            if (score > best_score && score > THRESHOLD) {
                best_score = score;
                best_class = j;
            }
        }

        // 如果有有效预测，添加到结果中
        if (best_class >= 0) {
            boxes.push_back(x1);
            boxes.push_back(y1);
            boxes.push_back(x2);
            boxes.push_back(y2);
            scores.push_back(best_score);
            classID.push_back(best_class);
        }
    }

    // 执行非极大值抑制
    return non_max_suppression(boxes, scores, classID, NMS_THRESHOLD);
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>> detector::LwDetr::non_max_suppression(
    std::vector<float>& boxes,
    std::vector<float> scores,
    std::vector<int> classID,
    float threshold
) {
    std::vector<float> keep_boxes, keep_scores;
    std::vector<int> keep_IDs;

    // 如果没有检测框，直接返回空结果
    if (boxes.size() == 0) {
        return std::make_tuple(keep_boxes, keep_scores, keep_IDs);
    }

    // 执行非极大值抑制
    for (size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] < threshold) {
            scores[i] = 0;
            continue;
        }

        // 与后续所有框计算IoU，抑制重叠框
        for (size_t j = i + 1; j < scores.size(); ++j) {
            if (scores[j] < threshold || classID[i] != classID[j]) {
                continue; // 跳过低于阈值的框或不同类别的框
            }

            // 计算两个框的IoU
            cv::Rect box1(
                boxes[i * 4],
                boxes[i * 4 + 1],
                boxes[i * 4 + 2] - boxes[i * 4],
                boxes[i * 4 + 3] - boxes[i * 4 + 1]
            );

            cv::Rect box2(
                boxes[j * 4],
                boxes[j * 4 + 1],
                boxes[j * 4 + 2] - boxes[j * 4],
                boxes[j * 4 + 3] - boxes[j * 4 + 1]
            );

            float iou = bbox_iou(box1, box2);

            // 如果IoU大于阈值，保留置信度更高的框
            if (iou > NMS_THRESHOLD) {
                if (scores[i] > scores[j]) {
                    scores[j] = 0;
                } else {
                    scores[i] = 0;
                    break; // 当前框已被抑制，不需要继续比较
                }
            }
        }
    }

    // 保留有效的检测框
    for (size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] > threshold) {
            keep_boxes.push_back(boxes[i * 4]);
            keep_boxes.push_back(boxes[i * 4 + 1]);
            keep_boxes.push_back(boxes[i * 4 + 2]);
            keep_boxes.push_back(boxes[i * 4 + 3]);
            keep_scores.push_back(scores[i]);
            keep_IDs.push_back(classID[i]);
        }
    }

    return std::make_tuple(keep_boxes, keep_scores, keep_IDs);
}

cv::Rect detector::LwDetr::clip_box(const cv::Rect& box, int origin_h, int origin_w) {
    int x1 = std::max(0, std::min(box.x, origin_w - 1));
    int y1 = std::max(0, std::min(box.y, origin_h - 1));
    int x2 = std::max(0, std::min(box.x + box.width, origin_w - 1));
    int y2 = std::max(0, std::min(box.y + box.height, origin_h - 1));

    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

// 辅助函数：计算两个矩形的IoU
float detector::LwDetr::bbox_iou(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    int inter_area = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int box1_area  = box1.width * box1.height;
    int box2_area  = box2.width * box2.height;

    float iou = static_cast<float>(inter_area) / (box1_area + box2_area - inter_area);
    return iou;
}