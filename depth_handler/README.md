# depth_handler

## 概述
`depth_handler` 包是一个基于 ROS2 Humble 的深度图像处理模块，主要功能是将 2D 检测结果与深度图像同步，提取对应区域的点云，计算并发布 3D 边界框和可视化工具。

## 主要功能
- 使用 `message_filters` 同步 2D 检测框 (`detector/msg/Bbox2dArray`) 与深度图像 (`sensor_msgs/Image`)
- 将深度图像转为点云，并在 ROI 范围内提取有效点
- 支持 TF2 坐标变换，将点云从相机坐标系转换到目标坐标系
- 计算 3D 边界框（`depth_handler/msg/Bbox3dArray`）并移除地面点
- 可选发布点云 (`sensor_msgs/PointCloud2`) 和可视化 Marker (`visualization_msgs/MarkerArray`)

## 消息定义
- `msg/Bbox3d.msg`
- `msg/Bbox3dArray.msg`

## 依赖关系
```bash
ament_cmake
rclcpp
std_msgs
sensor_msgs
geometry_msgs
detector
message_filters
cv_bridge
OpenCV
tf2 tf2_ros tf2_eigen tf2_geometry_msgs
visualization_msgs
rosidl_default_generators
```

## 安装与构建
```bash
# 在工作空间根目录
colcon build --packages-select depth_handler
source install/setup.bash
```

## 参数
| 参数名称                    | 类型    | 默认值                 | 描述                         |
| --------------------------- | ------- | ---------------------- | ---------------------------- |
| camera_info_topic           | string  | `/camera_info`         | 输入 CameraInfo 主题         |
| depth_topic                 | string  | `/depth/image_raw`     | 输入深度图像主题             |
| bbox2d_topic                | string  | `/detector/bbox2d`     | 输入 2D 检测框主题           |
| bbox3d_topic                | string  | `/depth_handler/bbox3d`| 输出 3D 边界框主题           |
| pointcloud_topic            | string  | `/depth_handler/points`| 输出点云主题                 |
| image_topic                 | string  | `/depth_handler/image` | 输出结果可视化图像           |
| visualization_topic         | string  | `/depth_handler/vis`   | 输出可视化 Marker 主题       |
| enable_visualization        | bool    | `true`                 | 是否发布可视化 Marker        |
| enable_pointcloud           | bool    | `false`                | 是否发布点云                 |
| marker_lifetime             | double  | `1.0`                  | Marker 存在时间（秒）         |
| marker_scale                | double  | `1.0`                  | Marker 缩放比例               |
| depth_scale                 | float   | `0.001`                | 深度值缩放因子（单位 m）      |
| min_points                  | int     | `50`                   | 最小有效点数量               |
| outlier_threshold           | float   | `0.1`                  | 异常值过滤阈值（单位 m）      |

## 启动示例
```bash
# 直接运行节点
ros2 run depth_handler depth_processor_node

# 使用 launch（若已编写 launch 脚本）
ros2 launch depth_handler depth_processor.launch.py
```

