# Detector 模块

这个模块是飞利诺双臂机器人系统（Fairino Dual Arm）的目标检测组件，基于YOLOv8深度学习模型实现，提供实时物体识别功能。

## 功能特点

- 使用YOLOv8深度学习模型进行实时目标检测
- 支持TensorRT加速，优化推理性能
- 集成ROS2框架，提供标准ROS2接口
- 支持自定义消息类型，传递检测结果
- 提供可视化功能，在图像上绘制检测框和目标类别
- 支持多类别物体检测（如瓶子等）

## 安装依赖

确保系统已安装以下依赖：

- ROS2（推荐Humble或更高版本）
- OpenCV 4.x
- CUDA 11.x 或更高版本
- TensorRT 8.x 或更高版本
- NVIDIA驱动（支持所用CUDA版本）

## 配置参数

在启动detector节点时，可以通过以下参数进行配置：

| 参数名称 | 类型 | 默认值 | 描述 |
| ------- | ---- | ------ | ---- |
| model_path | string | "/home/phoenix/roboarm/FairinoDualArm/src/detector/asserts/best0423.enging" | YOLOv8模型路径 |
| plugin_path | string | "/home/phoenix/tensorrtx/yolov8/build/libmyplugins.so" | TensorRT插件库路径 |
| confidence_threshold | double | 0.5 | 检测结果置信度阈值 |
| image_topic | string | "/camera/color/image_raw" | 输入图像话题 |
| detections_topic | string | "/detector/detections" | 输出检测结果话题 |

## 使用方法

### 构建

在工作空间根目录执行以下命令构建detector模块：

```bash
colcon build --packages-select detector
source install/setup.bash
```

### 运行

启动detector节点：

```bash
ros2 launch detector detector.launch.py
```

或者通过命令行直接运行并指定参数：

```bash
ros2 run detector detector_node --ros-args -p model_path:=/path/to/model.engine -p confidence_threshold:=0.6
```

### 接口说明

#### 订阅话题

- `/camera/color/image_raw` (sensor_msgs/msg/Image)：输入RGB图像

#### 发布话题

- `/detector/detections` (detector/msg/Bbox2dArray)：检测结果
- `/detector/detections/image` (sensor_msgs/msg/Image)：带有检测框的可视化图像

#### 自定义消息

- `Bbox2d.msg`：单个目标检测结果，包含：
  - `float32 x, y`：边界框中心点坐标
  - `float32 width, height`：边界框宽高
  - `int32 class_id`：类别ID
  - `float32 score`：置信度分数

- `Bbox2dArray.msg`：包含多个Bbox2d的检测结果数组

## 模型转换与部署

模型部署流程：

1. 准备YOLOv8训练好的`.pt`模型
2. 使用ONNX转换为`.onnx`格式
3. 使用TensorRT将`.onnx`转换为`.engine`格式用于加速推理

转换示例：

```bash
# PT转ONNX
python3 export.py --weights yolov8n.pt --format onnx

# ONNX转TensorRT引擎
/usr/src/tensorrt/bin/trtexec --onnx=yolov8n.onnx --saveEngine=yolov8n.engine --workspace=4096
```

## 常见问题

1. **模型加载失败**：确认模型路径和插件路径是否正确，并确保模型是兼容的TensorRT格式
2. **CUDA内存不足**：尝试减小batch_size或输入分辨率
3. **检测性能不佳**：调整confidence_threshold参数，或使用更适合目标场景的预训练模型

## 故障排除

如果遇到问题，请检查：

- CUDA和TensorRT是否正确安装
- 模型格式是否正确
- 相机是否正常工作并发布图像
- 硬件是否满足性能要求

## 贡献与开发

开发新功能时，请遵循以下最佳实践：

- 添加适当的注释
- 确保代码符合项目编码规范
- 编写单元测试
- 更新文档以反映变更

## 许可证

[请指定许可证]