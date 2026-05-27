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

# 使用 launch（整合相机、TF发布、检测器、深度处理）
ros2 launch depth_handler depth_processor.launch.py

# 轻量 launch（仅 TF 发布 + 深度处理）
ros2 launch depth_handler depth_tf.launch.py
```

## 坐标系与 TF 链路

### 核心链路

```
depthToPoints 输出          TF lookupTransform              最终输出
camera_depth_optical_frame ──→  ...  ──→  Lrobot_base      /depth_handler/bbox3d
       ↑                                                    frame_id: Lrobot_base
  光学坐标系                                           左臂基座坐标系
 (X右 Y下 Z前)
```

### source_frame_ 的选择

`sourcerce_frame_` 设为 `camera_link`，而非 `camera_depth_optical_frame`。

**原因**：手眼标定 `Ltcp → camera_link` 的结果本身是 OpenCV 光学约定（`solvePnP` 输出），与 `depthToPoints` 输出的光学坐标系一致。若使用 `camera_depth_optical_frame`，TF 链路会经过 URDF 中 `camera_depth_frame → camera_depth_optical_frame` 的 `rpy="-π/2, 0, -π/2"` 旋转，导致**光学坐标被重复旋转一次**。

### 完整 TF 树

```
Lrobot_base ← Ltcp ← camera_link ← camera_depth_frame ← camera_depth_optical_frame
  (robot     (手眼    (标定锚点,              ↑                ↑
  driver)    标定)   与depthToPoints    相机URDF        相机URDF
                     光学约定一致)    (identity)    (rpy=-π/2,0,-π/2)
```

### 关键约定

| 坐标系 | 用途 | 约定 |
|--------|------|------|
| `camera_depth_optical_frame` | 深度图 frame_id, depthToPoints 输出 | X右 Y下 Z前(深度) |
| `camera_depth_frame` | 纯 TF 中间节点 | X前 Y左 Z上 (机体) |
| `camera_link` | 标定锚点, source_frame_ | 同光学约定(标定结果) |
| `Ltcp` | 法兰盘 | 机器人坐标系 |
| `Lrobot_base` | 左臂基座, 最终输出 | 机器人坐标系 |

## 常见问题

### bbox3d 坐标偏差大

1. 确认机器人控制器在跑（`Lrobot_base` 存在于 TF 树）
2. 确认 `static_transforms.yaml` 已同步到 `install/` 目录
3. 确认 `source_frame_` = `camera_link`（不是 `camera_depth_optical_frame`）
4. 确认代码中**没有** `p.z() = -p.z()` 这类硬编码点云翻转

### 验证 TF 链路

```bash
# 查看 camera_link 在 Lrobot_base 下的位姿
ros2 run tf2_ros tf2_echo Lrobot_base camera_link

# 查看完整 TF 树
ros2 run tf2_ros tf2_monitor
```

## 手眼标定数据流

```
标定脚本(eye_in_hand_calibration.py)
  ↓ solvePnP (OpenCV光学约定)
camera_to_ltcp (camera_color_optical_frame → Ltcp)
  ↓ 取逆
ltcp_to_camera (Ltcp → camera_link)
  ↓ 写入 static_transforms.yaml
TF 发布节点(static_transforms_publisher.py)
  ↓ sendTransform
深度节点 lookupTransform 可查
```

> **注意**：标定使用彩色相机 (`camera_color_optical_frame`)，深度处理使用深度相机 (`camera_depth_optical_frame`)。两者之间有 ~1.4cm 的物理偏移（由相机出厂外参给出），对抓取精度影响可忽略。严格场景下可改为从 TF 动态读取 color→depth 外参。

