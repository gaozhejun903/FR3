# FR3 双臂协作系统

## 🚀 快速启动

```bash
# 一键启动全部节点 (双臂 + 夹爪 + 相机 + 检测 + 深度 + TF + 状态检查)
ros2 launch depth_handler depth_full.launch.py

# 不带状态检查的版本
ros2 launch depth_handler depth_processor.launch.py

# 仅 TF + 深度处理 (相机/机械臂已手动启动时)
ros2 launch depth_handler depth_tf.launch.py
```

### 可覆盖参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `L_robot_ip` | `192.168.58.2` | 左臂控制器 IP |
| `R_robot_ip` | `192.168.58.3` | 右臂控制器 IP |
| `L_gripper_port` | `...usb-0:2.2.4:1.0-port0` | 左夹爪 by-path |
| `R_gripper_port` | `...usb-0:2.2.1:1.0-port0` | 右夹爪 by-path |
| `confidence_threshold` | `0.5` | 检测置信度阈值 |
| `model_path` | `.../detector/best2.engine` | YOLO 模型路径 |

```bash
# 覆盖示例
ros2 launch depth_handler depth_full.launch.py \
  confidence_threshold:=0.7 \
  model_path:=/path/to/other/model.engine
```

### 启动后自动检查

`depth_full.launch.py` 会在 15 秒后自动检查以下组件：

| 检查项 | 节点/话题 |
|--------|----------|
| 左臂 | `Lrobo_ctrl`, `Lhigh_level`, `/L/joint_states` |
| 右臂 | `Rrobo_ctrl`, `Rhigh_level`, `/R/joint_states` |
| 左夹爪 | `L_gripper_node`, `/L_gripper_node/status_stream` |
| 右夹爪 | `R_gripper_node`, `/R_gripper_node/status_stream` |
| 目标检测 | `detector_node`, `/detector/detections` |
| 深度处理 | `depth_handler_node`, `/depth_handler/bbox3d` |
| TF 发布 | `camera_static_tf_publisher`, `gripper_static_tf_publisher`, `Lfake_gripper_tf_publisher`, `Rfake_gripper_tf_publisher` |
| 相机 | `ob_camera_node`, `/camera/color/image_raw`, `/camera/depth/image_raw` |

也可单独运行检查脚本：
```bash
bash src/depth_handler/scripts/status_checker.sh
```

---

## 📐 标定

### TF 树结构

```
world
├── Lrobot_base                    ← robo_ctrl_node 动态发布 (L=原点)
│   ├── Ltcp                       ← robo_ctrl_node 动态发布 (法兰)
│   │   ├── camera_link            ← static_tf (手眼标定结果)
│   │   └── Lgripper_tip           ← static_tf (TCP四点法标定结果)
│   └── Lfake_gripper_frame        ← fake_gripper_tf_publisher (位置=Lgripper_tip, Z=世界朝上)
└── Rrobot_base                    ← robo_ctrl_node 动态发布 (双臂基座标定结果)
    ├── Rtcp                       ← robo_ctrl_node 动态发布 (法兰)
    │   └── Rgripper_tip           ← static_tf (TCP四点法标定结果)
    └── Rfake_gripper_frame        ← fake_gripper_tf_publisher (位置=Rgripper_tip, Z=世界朝上)
```

### 当前标定值

| 变换 | 来源 | 日期 | 值 |
|------|------|------|-----|
| `Ltcp → camera_link` | 手眼标定 (TSAI, ArUco) | 2026-05-27 | t=[0.0115, -0.0859, -0.0779], RPY=[-0.109, -0.859, 90.740] |
| `Ltcp → Lgripper_tip` | 四点法 TCP | 2026-06-04 | t=[-0.00166, 0.00182, 0.18564], 纯平移 |
| `Rtcp → Rgripper_tip` | 四点法 TCP | 2026-06-04 | t=[0.00389, 0.00306, 0.19254], 纯平移 |
| `world → Rrobot_base` | 双臂基座标定 (Arun SVD) | 2026-06-04 | t=[-0.09815, 1.05515, -0.01051], RPY=[0.869, -1.130, 34.878] |

> 配置文件: `src/tools/config/static_transforms.yaml`
> 双臂基座: 由 `robo_ctrl_node.cpp` 的 `publish_tf_transforms()` 动态发布

### 标定工具

| 脚本 | 用途 | 用法 |
|------|------|------|
| `4point_interactive.py` | TCP 四点法标定 (交互式, 断点续采) | `python3 4point_interactive.py --robot_state_topic /L/robot_state --save_path ./tcp_data/L` |
| `dualarm_points.py` | 双臂基座标定 (顺序碰点法) | `python3 dualarm_points.py --save_path ./dual_calib_data` |
| `eye_in_hand_calibration.py` | 手眼标定 (ArUco/棋盘格) | `python3 eye_in_hand_calibration.py --data_path ... --aruco_dict DICT_6X6_250 --aruco_size 0.1 --camera_matrix_file ...` |
| `dualarm.py` | 双臂标定 (AX=XB 法, 需同时采集成对姿态) | `python3 dualarm.py --data_dir /tmp/dual_end_tf_data/dual_poses/` |

---

## 📦 软件结构

```
src/
├── camera_info_interceptor/     # 相机信息拦截 (Foxglove 兼容)
├── depth_handler/               # 深度处理 + 一体化 launch
│   ├── launch/
│   │   ├── depth_full.launch.py       # ★ 推荐: 全量启动 + 状态检查
│   │   ├── depth_processor.launch.py  # 全量启动 (无状态检查)
│   │   └── depth_tf.launch.py         # 轻量: 仅 TF + 深度
│   └── scripts/
│       └── status_checker.sh          # 系统状态检查脚本
├── detector/                    # YOLOv8 目标检测 (TensorRT)
├── dualarm/                     # 双臂运动规划与执行
│   └── config/config.yaml       # 任务关节姿态配置
├── epg50_gripper_ros/           # EPG50 夹爪 ROS2 驱动
├── robo_ctrl/                   # 机器人控制 (状态发布 + MoveCart + Servo + TF)
│   ├── src/
│   │   ├── robo_ctrl_node.cpp   # 核心: 状态线程 + TF广播
│   │   ├── high_level.cpp       # 高层: 圆弧/伺服轨迹规划
│   │   └── main.cpp             # 任务入口
│   ├── include/libfairino/      # 法奥机器人 SDK
│   └── launch/                  # 旧版独立 launch (已废弃, 请用 depth_handler)
├── tools/                       # 标定工具 & 静态TF
│   ├── config/
│   │   └── static_transforms.yaml  # ★ 静态 TF 配置 (手眼+TCP 标定结果)
│   ├── scripts/                 # 标定脚本
│   └── src/                     # TF 发布 & 数据采集节点
├── tf_node/                     # (已废弃) 旧 TF 节点
└── fairino3_v6_*/               # MoveIt2 配置 (moveit2_config / planner)
```

---

## 🔧 安装与构建

```bash
# 依赖
sudo apt install -y ros-humble-ros-base ros-humble-moveit2 ros-humble-rviz2

# 构建
cd ~/ros2_ws  # 或你的实际工作空间路径
# 本仓库位于 /home/mihu/FR3_again，已配置为独立工作空间
cd /home/mihu/FR3_again
colcon build --symlink-install
source install/setup.bash
```

---

## ⚠️ 注意事项

- **夹爪供电**：大疆电池有休眠机制，启动前确认夹爪已经上电
- **设备权限**：launch 文件会自动 `chmod 777` 串口设备
- **相机内参**：由相机驱动自动发布 `/camera/color/camera_info` 和 `/camera/depth/camera_info`
- **深度处理器** 使用外参 (`R_c2d`, `t_c2d`) 将彩色图检测框像素级对齐到深度图
