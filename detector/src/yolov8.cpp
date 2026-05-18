#include "detector/yolov8.hpp"

// Define gLogger
class Logger: public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // suppress info-level messages
        if (severity != Severity::kINFO) {
            std::cout << msg << std::endl;
        }
    }
} gLogger;

detector::YoloV8::YoloV8() {
    host_inputs  = new float[batch_size * 3 * input_h * input_w]; // 分配 host_inputs
    host_outputs = new float[38001];
}

detector::YoloV8::~YoloV8() {
    // Free CUDA memory
    cudaFree(cuda_inputs);
    cudaFree(cuda_outputs);
    // Destroy stream
    cudaStreamDestroy(stream);
    // Free bindings
    free(bindings);
}

int detector::YoloV8::init(std::string engine_path, std::string plugin_path) {
#ifndef PLUGIN_REGIST
    #define PLUGIN_REGIST
    void* handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
    if (!handle) {
        std::cerr << "Error loading plugin library: " << dlerror() << std::endl;
        return -1;
    }
#endif

    // Load engine
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
        std::cerr << "Error: Unable to open engine file" << std::endl;
        return -1;
    }

    initLibNvInferPlugins(&gLogger, "");

    // Deserialize the engine and create context
    runtime = nvinfer1::createInferRuntime(gLogger);
    std::ifstream file(engine_path, std::ios::binary);
    std::vector<char> engine_data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    engine  = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
    context = engine->createExecutionContext();
    // Allocate GPU memory
    cudaMalloc((void**)&cuda_inputs, batch_size * 3 * input_h * input_w * sizeof(float));
    cudaMalloc((void**)&cuda_outputs, 38001 * sizeof(float)); // Output size based on model

    cudaStreamCreate(&stream);

    bindings    = new void*[2];
    bindings[0] = cuda_inputs;
    bindings[1] = cuda_outputs;
    return 0;
}

std::tuple<
    std::vector<cv::Mat>,
    double,
    std::vector<std::vector<float>>,
    std::vector<std::vector<float>>,
    std::vector<std::vector<int>>,
    std::vector<std::vector<float>>>
detector::YoloV8::infer(std::vector<cv::Mat>& raw_image_generator) {
    // Prepare batch
    std::vector<cv::Mat> batch_image_raw = {};
    std::vector<int> batch_origin_h      = {};
    std::vector<int> batch_origin_w      = {};
    std::vector<float> batch_input_image(batch_size * 3 * input_h * input_w);

    for (size_t i = 0; i < raw_image_generator.size(); ++i) {
        int origin_h = 640, origin_w = 640;
        cv::Mat image_raw   = raw_image_generator[i];
        cv::Mat input_image = preprocess_image(image_raw);
        // std::cout<<"perprocess done"<<std::endl;
        batch_image_raw.push_back(image_raw);
        batch_origin_h.push_back(origin_h);
        batch_origin_w.push_back(origin_w);
        // std::cout<<"memcpy"<<std::endl;
        std::memcpy(
            batch_input_image.data() + i * 3 * input_h * input_w,
            input_image.data,
            3 * input_h * input_w * sizeof(float)
        );
    }

    // Transfer input to GPU
    cudaMemcpyAsync(
        cuda_inputs,
        batch_input_image.data(),
        batch_size * 3 * input_h * input_w * sizeof(float),
        cudaMemcpyHostToDevice,
        stream
    );

    // Execute the inference
    auto start = std::chrono::high_resolution_clock::now();
    context->enqueue(batch_size, bindings, stream, nullptr);
    cudaMemcpyAsync(host_outputs, cuda_outputs, output_size * sizeof(float), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    auto end                               = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    std::vector<std::vector<float>> result_boxes = {}, result_scores = {};
    std::vector<std::vector<int>> result_classID = {};
    std::vector<std::vector<float>> det          = {};

    for (int i = 0; i < batch_size; ++i) {
        auto output_offset = host_outputs + i * 38001; // Adjust based on output size
        auto post_result =
            YoloV8::post_process(output_offset, raw_image_generator[i].cols, raw_image_generator[i].rows, 38001);
        result_boxes.push_back(std::get<0>(post_result));
        result_scores.push_back(std::get<1>(post_result));
        result_classID.push_back(std::get<2>(post_result));
    }

    return std::make_tuple(batch_image_raw, duration.count(), result_boxes, result_scores, result_classID, det);
}

cv::Mat detector::YoloV8::preprocess_image(const cv::Mat& raw_bgr_image) {
    cv::Mat image_raw = raw_bgr_image.clone();
    int h             = image_raw.rows;
    int w             = image_raw.cols;
    int input_w_      = 640;
    int input_h_      = 640;

    // 将BGR转换为RGB
    cv::Mat image;
    cv::cvtColor(image_raw, image, cv::COLOR_BGR2RGB);

    // 计算缩放比例
    float r_w = static_cast<float>(input_w_) / w;
    float r_h = static_cast<float>(input_h_) / h;

    int tw, th, tx1, tx2, ty1, ty2;

    // 根据长宽比进行调整
    if (r_h > r_w) {
        tw  = input_w_;
        th  = static_cast<int>(r_w * h);
        tx1 = tx2 = 0;
        ty1       = (input_h_ - th) / 2;
        ty2       = input_h_ - th - ty1;
    } else {
        tw  = static_cast<int>(r_h * w);
        th  = input_h_;
        tx1 = (input_w_ - tw) / 2;
        tx2 = input_w_ - tw - tx1;
        ty1 = ty2 = 0;
    }

    // 调整图像大小
    cv::resize(image, image, cv::Size(tw, th));

    // 填充图像
    cv::copyMakeBorder(image, image, ty1, ty2, tx1, tx2, cv::BORDER_CONSTANT, cv::Scalar(128, 128, 128));

    // 转换为float32，并归一化到 [0, 1]
    image.convertTo(image, CV_32FC3, 1.0 / 255.0);

    // 转置为NCHW格式
    cv::Mat chw_image;
    std::vector<cv::Mat> channels(3);
    cv::split(image, channels);
    cv::vconcat(channels, chw_image);

    // 扩展维度以适应批量输入
    chw_image = chw_image.reshape(1, { 1, 3, input_h_, input_w_ });
    // std::cout<<"perprocess image shape: "<<chw_image.size()<<std::endl;

    return chw_image;
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>> detector::YoloV8::non_max_suppression(
    std::vector<float>& boxes,
    std::vector<float> scores,
    std::vector<int> classID,
    float threshold
) {
    std::vector<float> keep_boxes, keep_scores;
    std::vector<int> keep_IDs;
    if (boxes.size() == 0) {
        return std::make_tuple(keep_boxes, keep_scores, keep_IDs);
    }
    for (size_t i = 0; i < scores.size(); ++i) {
        // nms
        for (size_t j = i + 1; j < scores.size(); ++j) {
            if (scores[j] < threshold) {
                scores[j] = 0;
                continue;
            }
            cv::Rect box1(boxes[i * 4], boxes[i * 4 + 1], boxes[i * 4 + 2], boxes[i * 4 + 3]);
            cv::Rect box2(boxes[j * 4], boxes[j * 4 + 1], boxes[j * 4 + 2], boxes[j * 4 + 3]);
            float iou = bbox_iou(box1, box2);
            if (iou > 0.5) {
                if (scores[i] > scores[j]) {
                    scores[j] = 0;
                } else {
                    scores[i] = 0;
                }
            }
        }
    }
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

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
detector::YoloV8::post_process(float* output, int origin_w, int origin_h, int output_size) {
    std::vector<float> boxes  = {};
    std::vector<float> scores = {};
    std::vector<float> det    = {};
    std::vector<int> classID  = {};

    float r_w = static_cast<float>(640) / origin_w;
    float r_h = static_cast<float>(640) / origin_h;

    // 通过缩放比例 r_w 和 r_h 判断是否基于宽或高进行缩放
    float scale = (r_h > r_w) ? r_w : r_h;
    float dw    = (640 - scale * origin_w) / 2; // 水平填充的宽度
    float dh    = (640 - scale * origin_h) / 2; // 垂直填充的高度

    int result_count = output[0];

    for (int i = 1; i < result_count * 38; i += 38) {
        float score  = output[i + 4]; // 第5个元素是置信度
        int classID_ = output[i + 5]; // 第6个元素是类别ID
        if (score < 0.1)
            continue;

        // 获取缩放后的坐标
        float x1 = output[i];
        float y1 = output[i + 1];
        float x2 = output[i + 2];
        float y2 = output[i + 3];

        // 去除填充并按比例还原坐标到原图
        x1 = (x1 - dw) / scale;
        y1 = (y1 - dh) / scale;
        x2 = (x2 - dw) / scale;
        y2 = (y2 - dh) / scale;

        // 将坐标限制在图像边界内
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(origin_w)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(origin_h)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(origin_w)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(origin_h)));

        boxes.push_back(x1);
        boxes.push_back(y1);
        boxes.push_back(x2);
        boxes.push_back(y2);
        scores.push_back(score);
        classID.push_back(classID_); // 类别ID
    }
    std::tuple<std::vector<float>, std::vector<float>, std::vector<int>> nms_result =
        detector::YoloV8::non_max_suppression(boxes, scores, classID, 0.3);
    return nms_result;
}

cv::Rect detector::YoloV8::clip_box(const cv::Rect& box, int origin_h, int origin_w) {
    int x1 = std::max(0, std::min(box.x, origin_w - 1));
    int y1 = std::max(0, std::min(box.y, origin_h - 1));
    int x2 = std::max(0, std::min(box.x + box.width, origin_w - 1));
    int y2 = std::max(0, std::min(box.y + box.height, origin_h - 1));

    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

// 辅助函数：计算两个矩形的IoU
float detector::YoloV8::bbox_iou(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    int inter_area = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int box1_area  = box1.width * box1.height;
    int box2_area  = box2.width * box2.height;
    float iou      = static_cast<float>(inter_area) / (box1_area + box2_area - inter_area);
    return iou;
}