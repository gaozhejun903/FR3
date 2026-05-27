# FairinoDualArm

## 当前工作进度（vision 分支）
- **项目初始化**：整理仓库结构，所有功能包统一归入 `src/` 目录
- **detector 检测模块**：修改 detector 功能包，检测节点编译通过并测试成功，可检测到目标物体
- **相机数据**：解决彩色图与深度图像素对齐问题（待测试）；修改相机内参以适配当前设备
- **robo_ctrl**：修复编译问题（调整 CMake）、适配当前相机内参
- **手眼标定**：修改手眼标定相关文件，完成标定数据拍摄与结果保存。标定数据位于 `calibration_data/`（结果: `calibration_result_aruco_20260520_113046.json`，原始图像与位姿在 `images/` 和 `poses/` 子目录下）
- **其他**：编写时间戳测试文件（编译通过，待实践验证）

## 概述
FairinoDualArm 是一个基于 ROS2 Humble 的双臂协作机器人控制包，集成了感知、运动规划与执行功能，可用于实现复杂的抓取、搬运与协作任务。

## 主要功能
- 传感器接入：支持深度相机、激光雷达输入
- 运动规划：基于 MoveIt2 实现双臂协同规划
- 抓取与放置：自定义抓取策略与末端工具
- 可视化调试：RViz 工具链支持

## 软件结构
```plaintext
src/
├── camera_info_interceptor    # 相机信息拦截与转换节点
├── depth_handler              # 深度图像处理节点
├── detector                   # 目标检测与定位节点
├── dualarm                    # 双臂运动规划与执行核心
├── epg50_gripper_ros          # EPG50 电磁夹爪驱动
├── robo_ctrl                  # 机器人整体控制管理节点
├── tools                      # 辅助脚本与工具
└── tf_node                    # TF 坐标变换广播节点
```

## 安装与构建
```bash
# 安装依赖
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-moveit2 ros-humble-rviz2

# 克隆仓库
cd ~/ros2_ws/src
git clone <仓库地址> FairinoDualArm

# 构建
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 启动方式

### 一体化 Launch 文件（推荐）

`depth_handler` 包提供了两个预配置的 launch 文件，将多个节点整合到一起，简化启动流程。

#### 全量启动 — `depth_processor.launch.py`

**用途**：一键启动相机、机械臂控制、目标检测、静态TF、深度处理等所有核心节点。

| 集成的节点 | 来源包 | 说明 |
|---|---|---|
| Gemini 330 系列相机 | `orbbec_camera` | RGB-D 图像流 + 点云 |
| 静态 TF 发布 (camera + gripper) | `tools` | `Ltcp → camera_link`、`Ltcp → Lgripper` |
| 机械臂控制 | `robo_ctrl` (`robo_ctrl_node`) | 机器人状态发布、运动控制服务 |
| 目标检测 | `detector` (`detector_node_exe`) | YOLOv8 推理，发布 2D 检测框 |
| 深度处理 | `depth_handler` (`depth_processor_node`) | 2D 检测框 + 深度图 → 3D 坐标 |

```bash
# 使用默认参数（robot_ip: 192.168.58.2, robot_name: L, confidence: 0.5）
ros2 launch depth_handler depth_processor.launch.py

# 覆盖参数
ros2 launch depth_handler depth_processor.launch.py \
  robot_ip:=192.168.58.2 \
  robot_name:=L \
  confidence_threshold:=0.7 \
  model_path:="/home/mihu/FR3_again/src/detector/best2.engine"
```

#### 轻量启动 — `depth_tf.launch.py`

**用途**：仅启动静态 TF 和深度处理节点，适用于相机和机械臂已在其他终端手动启动的场景。

| 集成的节点 | 来源包 | 说明 |
|---|---|---|
| 静态 TF 发布 (camera + gripper) | `tools` | `Ltcp → camera_link`、`Ltcp → Lgripper` |
| 深度处理 | `depth_handler` (`depth_processor_node`) | 2D 检测框 + 深度图 → 3D 坐标 |

```bash
ros2 launch depth_handler depth_tf.launch.py
```

### 手动启动（逐个节点，调试用）

```bash
# 相机
ros2 launch orbbec_camera gemini_330_series.launch.py
# 机械臂
ros2 run robo_ctrl robo_ctrl_node --ros-args -p robot_ip:=192.168.58.2 -p robot_name:=L
# 目标检测
ros2 run detector detector_node_exe --ros-args -p confidence_threshold:=0.5 -p model_path:="/home/mihu/FR3_again/src/detector/best2.engine"
# 静态TF
ros2 launch tools static_tf_multiple.launch.py
# 深度处理
ros2 run depth_handler depth_processor_node
# 高层规划（按需）
ros2 run robo_ctrl high_level_node
```

foxglove观察用（可选）
```bash
ros2 run camera_info_interceptor camera_info_interceptor_node
```

记得检查下夹爪的供电情况，大疆电池有休眠

## 需要的service和话题
机器人控制
```
/robot_state
/depth_handler/visualization 的 center_point
/transform_point
/epg50_gripper/command
/robot_move_cart
/robot_act
```
