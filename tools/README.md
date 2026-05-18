# tools

## 概述
`tools` 包是一个基于 ROS2 Humble 的工具集合，提供了手眼标定数据收集、坐标变换处理、TF工具等功能，辅助机器人系统的开发、调试和标定。

## 主要功能
- **手眼标定工具**
  - 标定数据采集节点：同步采集相机图像和机器人位姿
  - 标定算法脚本：支持棋盘格检测与手眼标定计算
- **坐标变换工具**
  - TF点查询服务：实时查询坐标系之间的点位转换
  - 静态TF发布器：基于配置发布静态坐标变换
  - 虚拟夹爪TF发布：模拟夹爪坐标系发布
- **实用工具脚本**
  - ArUco标记生成工具
  - TF查询命令行客户端

## 组件和服务

### 1. 手眼标定数据采集器
提供服务接口控制数据采集，支持自动和手动触发，用于收集标定用图像和位姿对。

```bash
# 启动节点
ros2 launch tools eye_hand_calibration_data_collector.launch.py

# 手动触发采集
ros2 service call /eye_hand_calibration_data_collector/capture_calibration_sample std_srvs/srv/Trigger
```

### 2. TF点坐标变换服务
提供点位坐标系变换服务，支持查询任意坐标系之间的点位变换关系。

```bash
# 启动服务
ros2 launch tools tf_point_query.launch.py

# 查询变换 (使用Python客户端)
ros2 run tools tf_query_client.py base_link camera_link 1.0 2.0 3.0

# 直接调用服务
ros2 service call /tf_point_query/transform_point tools/srv/TransformPoint "{
  source_frame: 'base_link',
  target_frame: 'camera_link',
  point: {x: 1.0, y: 2.0, z: 3.0},
  timeout: 1.0
}"
```

### 3. 静态TF发布器
从配置文件加载并发布静态坐标变换关系，支持多个TF配置同时加载。

```bash
# 单个TF变换发布
ros2 launch tools static_tf_publisher.launch.py config_file:=tcp_to_gripper_tf_config.yaml

# 多个TF变换发布
ros2 launch tools static_tf_multiple.launch.py config_file:=static_transforms.yaml
```

配置文件示例 (static_transforms.yaml):
```yaml
transforms:
  - parent: robot_base_link
    child: camera_base_link
    x: 0.5
    y: -0.1
    z: 0.2
    roll: 0.0
    pitch: 0.0
    yaw: 3.14159
  - parent: tool0
    child: camera_link
    x: 0.0
    y: 0.0
    z: 0.05
    roll: 0.0
    pitch: -1.57079
    yaw: 0.0
```

### 4. 手眼标定脚本
处理采集的标定数据，计算相机与机器人末端之间的变换关系。

```bash
# 执行标定
python3 $(ros2 pkg prefix tools)/share/tools/scripts/eye_in_hand_calibration.py \
  --data_path /path/to/calibration/data \
  --pattern_size 7 6 \
  --square_size 0.025
```

### 5. TransformPoint 服务接口
服务接口用于请求点坐标从一个坐标系转换到另一个坐标系。

**请求参数:**
- `source_frame`: 源坐标系名称
- `target_frame`: 目标坐标系名称
- `point`: 源坐标系中的点位坐标 (geometry_msgs/Point)
- `timeout`: 等待变换可用的超时时间 (秒)

**响应参数:**
- `transformed_point`: 转换后的点位 (geometry_msgs/PointStamped)
- `success`: 转换是否成功
- `error_msg`: 如果失败，错误信息

## 配置文件
包中包含以下默认配置文件:

- `static_tf_config.yaml`: 通用静态TF配置
- `tcp_to_gripper_tf_config.yaml`: 从TCP到夹爪的TF配置
- `static_transforms.yaml`: 包含多个TF变换的综合配置

## 依赖关系
```
- ROS2 Humble
- OpenCV
- rclcpp
- tf2/tf2_ros
- geometry_msgs
- sensor_msgs
- cv_bridge
- image_transport
- nlohmann_json
- yaml_cpp_vendor
```

## 安装与构建
```bash
cd ~/roboarm/FairinoDualArm
colcon build --symlink-install --packages-select tools
source install/setup.bash
```

## 维护与贡献
- 维护者: phoenix (<lanlanlllma@gmail.com>)
- 许可证: 待定
- 问题反馈请提交至项目Issue或联系维护者

## 组件说明

### 眼手标定数据收集器 (EyeHandCalibrationDataCollector)

该组件是一个ROS2节点，用于同步采集相机图像和机器人TCP位姿数据，为后续的眼手标定提供必要的输入数据。

#### 主要特性

- 支持消息同步：使用message filters同步订阅图像和机器人状态消息
- 支持自动采集：可配置定时自动采集标定数据
- 数据可视化：在保存的图像上标记检测到的特征点
- TF集成：支持记录相关坐标系之间的变换关系

#### 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|-------|------|
| save_path | string | /tmp/eye_hand_calibration_data | 数据保存路径 |
| image_topic | string | /camera/color/image_raw | 相机图像话题 |
| robot_state_topic | string | /robot_state | 机器人状态话题 |
| auto_capture | bool | false | 是否启用自动采集 |
| auto_capture_interval | int | 2000 | 自动采集间隔(毫秒) |
| camera_frame_id | string | camera_color_optical_frame | 相机坐标系ID |
| robot_base_frame_id | string | robot_base_link | 机器人基座坐标系ID |

#### 使用方法

1. 启动节点：
```bash
ros2 run tools eye_hand_calibration_data_collector --ros-args -p image_topic:=/camera/color/image_raw -p robot_state_topic:=/robot_state
```

2. 触发数据采集：
```bash
ros2 service call /eye_hand_calibration_data_collector/capture_calibration_sample std_srvs/srv/Trigger
```

### 眼在手上标定脚本 (eye_in_hand_calibration.py)

该Python脚本用于处理采集的标定数据，计算相机相对于机器人末端执行器的变换关系。

#### 主要特性

- 棋盘格检测：自动检测标定板上的棋盘格角点
- 相机内参标定：同时进行相机内参标定或使用现有内参
- 多种标定算法：支持TSAI等经典手眼标定算法
- 标定评估：计算重投影误差，评估标定质量
- 结果可视化：保存角点检测和标定结果的可视化图像

#### 使用方法

从头进行完整标定（包括相机内参）：
```bash
python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --pattern_size 7 6 --square_size 0.025
```

使用现有相机内参仅进行手眼标定：
```bash
python3 eye_in_hand_calibration.py --data_path /path/to/calibration/data --camera_matrix_file /path/to/camera_matrix.json
```

参数说明：
- `--data_path`: 标定数据路径，包含images和poses子目录
- `--pattern_size`: 棋盘格内部角点的行列数
- `--square_size`: 棋盘格方格的实际尺寸(米)
- `--camera_matrix_file`: 可选，包含相机内参的JSON文件路径

#### 相机内参JSON文件格式

脚本支持两种常见的相机内参JSON格式：

1. 'k'/'d' 格式 (常见于RealSense等相机的标定文件)：
```json
{
    "k": [fx, 0, cx, 0, fy, cy, 0, 0, 1],
    "d": [k1, k2, p1, p2, k3, ...]
}
```

2. 'camera_matrix'/'distortion_coefficients' 格式：
```json
{
    "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    "distortion_coefficients": [k1, k2, p1, p2, k3]
}
```

#### 输出结果

脚本执行后会在data_path目录下生成以下文件：
- `calibration_result.json`: 包含相机内参和手眼变换关系的结果文件
- `calibration_vis/`: 包含角点检测可视化的图像目录

## 依赖项

- ROS2 humble
- OpenCV 4.x
- NumPy
- SciPy
- cv_bridge
- message_filters
- nlohmann_json

## 构建与安装

```bash
cd /path/to/FairinoDualArm
colcon build --packages-select tools
source install/setup.bash
```

## 常见问题

1. **消息同步失败**
   - 检查两个话题的时间戳是否匹配
   - 确认机器人状态消息中包含正确格式的TCP位姿信息

2. **标定精度不高**
   - 尝试采集更多不同位姿下的图像
   - 检查棋盘格模式是否正确识别
   - 确认棋盘格尺寸参数与实际一致

3. **相机内参文件格式不兼容**
   - 确认JSON文件包含'k'和'd'字段或'camera_matrix'和'distortion_coefficients'字段
   - 检查矩阵维度是否正确

## 示例

标定结果格式示例:

```json
{
  "camera_matrix": [
    [615.2, 0.0, 320.1],
    [0.0, 616.8, 240.3],
    [0.0, 0.0, 1.0]
  ],
  "distortion_coefficients": [0.02, -0.05, 0.001, 0.001, 0.0],
  "camera_to_gripper": {
    "translation": {
      "x": 0.05,
      "y": 0.02,
      "z": 0.12
    },
    "rotation_matrix": [
      [0.0, -1.0, 0.0],
      [-1.0, 0.0, 0.0],
      [0.0, 0.0, -1.0]
    ],
    "euler_angles_xyz_degrees": [180.0, 0.0, 90.0]
  },
  "reprojection_error": 0.32
}
```
