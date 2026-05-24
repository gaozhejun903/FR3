# my_arm_control

双臂控制与虚拟视觉节点包，支持仿真 (MoveIt) 和实机 (robo_ctrl) 双后端。

## 节点

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| arm_task_manager | arm_task_node | 核心任务管理器，状态机驱动 |
| virtual_vision_node | virtual_vision_node | 仿真用虚拟视觉，圆弧轨迹发布 target_pose |
| fake_vision_node | fake_vision_node | 实机测试用，发布固定 Bbox3dArray 坐标 |

## 状态机

```
IDLE -> OBSERVATION -> APPROACHING -> GRABBING -> RETREATING -> PLACING -> IDLE
```

## 关键参数

### 高度参数 (mm)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| desk_height | 360.0 | 机器人基座到桌面的高度 |
| object_height | 89.0 | 目标物体高度 (cola=89, cestbon=83) |

z 轴抓取计算公式: `dz = desk_height + object_height - current_tcp_z`

与 `dualarm/config/config.yaml` 保持一致。

### 运动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| velocity | 50.0 | 实机运动速度 (mm/s) |
| acceleration | 50.0 | 实机加速度 |
| velocity_scale | 0.4 | MoveIt 速度比例 |
| acceleration_scale | 0.4 | MoveIt 加速度比例 |

### 夹爪参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| gripper_port | /dev/ttyACM0 | 串口 (实机为 /dev/ttyUSB0) |
| gripper_slave_id | 9 | Modbus 从站 ID (左臂=9, 右臂=10) |
| approach_offset_z | 150.0 | 接近偏移 z (mm) |
| retreat_z | 80.0 | 撤离高度 (mm) |

### 检测滤波参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| distance_threshold | 150.0 | 同一目标合并距离 (mm) |
| age_threshold | 5 | 目标过期帧数 (超过则丢弃) |
| valid_threshold | 1 | 最低有效检测次数 |
| kalman_process_noise | 0.01 | Kalman 过程噪声 |
| kalman_measurement_noise | 0.05 | Kalman 测量噪声 |

### 观测/放置位姿

| 参数 | 默认值 (实机) | 说明 |
|------|--------------|------|
| observe_x/y/z | 99.917, -144.210, 542.554 | 观测位姿 (mm) |
| observe_rx/ry/rz | -125.357, 0, -100.476 | 观测姿态 (度) |
| place_x/y/z | 200, -300, 200 | 放置位姿 (mm) |
| place_rx/ry/rz | -90, 0, -90 | 放置姿态 (度) |

## 启动

### 仿真

```bash
ros2 launch my_arm_control simulation_test.launch.py
```

### 实机完整启动顺序

**必须严格按以下顺序启动，每步间隔 3-5 秒:**

#### 第 1 步: 摄像头驱动 (Orbbec Gemini 335)

```bash
# 终端 1: 启动 Orbbec 摄像头驱动
#   发布话题: /camera/color/image_raw, /camera/depth/image_raw, /camera/camera_info
#   驱动包位置: /home/gzj/ros2_ws/src/OrbbecSDK_ROS2
ros2 launch orbbec_camera gemini_330_series.launch.py
```

**验证:** `ros2 topic hz /camera/color/image_raw` 应显示 ~30fps

#### 第 2 步: 左臂驱动 + 视觉管线 (IP: 192.168.58.2)

```bash
# 终端 2: 启动左臂全套
#   包含: robo_ctrl_node(L), high_level_node(L), detector_node,
#         depth_handler_node, camera_info_interceptor, static_tf
ros2 launch robo_ctrl robo_ctrl_L.launch.py
```

**验证:** `ros2 service list | grep L/robot_move_cart` 应显示服务

#### 第 3 步: 右臂驱动 (IP: 192.168.58.3)

```bash
# 终端 3: 启动右臂
#   包含: robo_ctrl_node(R), high_level_node(R)
ros2 launch robo_ctrl robo_ctrl_R.launch.py
```

**验证:** `ros2 service list | grep R/robot_move_cart` 应显示服务

#### 第 4 步: 夹爪节点 (双夹爪)

```bash
# 终端 4a: 启动左臂夹爪 (ttyUSB0, slave_id=9)
#   服务名: /gripper_command
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p default_slave_id:=9 \
  -r __node:=gripper_node_L

# 终端 4b: 启动右臂夹爪 (ttyUSB1, slave_id=10)
#   服务名: /R_gripper_command (通过 service_prefix 区分)
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p default_slave_id:=10 \
  -p service_prefix:=R_ \
  -r __node:=gripper_node_R
```

**验证:**
- `ros2 service call /gripper_command epg50_gripper_ros/srv/GripperCommand '{command: 1, slave_id: 9}'` → 左臂使能
- `ros2 service call /R_gripper_command epg50_gripper_ros/srv/GripperCommand '{command: 1, slave_id: 10}'` → 右臂使能

#### 第 5 步: 任务节点

```bash
# 终端 5: 启动任务管理器 (等待 5s 让所有服务就绪)
# 单臂抓取:
ros2 launch my_arm_control real_hardware.launch.py

# 拧瓶盖任务:
ros2 launch my_arm_control real_hardware.launch.py task_mode:=opencap

# 接球任务:
ros2 launch my_arm_control real_hardware.launch.py task_mode:=ball
```

### 快速启动 (单终端)

```bash
# 一行命令启动全部 (后台运行)
ros2 launch orbbec_camera gemini_330_series.launch.py &
sleep 5
ros2 launch robo_ctrl robo_ctrl_L.launch.py &
sleep 5
ros2 launch robo_ctrl robo_ctrl_R.launch.py &
sleep 5
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args -p port:=/dev/ttyUSB0 -p default_slave_id:=10 &
sleep 3
ros2 launch my_arm_control real_hardware.launch.py task_mode:=opencap
```

### 停止所有节点

```bash
# 停止所有 ROS2 节点
pkill -f ros2
# 或逐个停止
pkill -f arm_task_node
pkill -f robo_ctrl
pkill -f detector
pkill -f depth_handler
pkill -f camera_info_interceptor
pkill -f epg50_gripper
pkill -f orbbec_camera
```

### 节点依赖关系

```
摄像头驱动 (orbbec_camera)
    ↓ /camera/color/image_raw, /camera/depth/image_raw
视觉管线 (detector_node + depth_handler_node + camera_info_interceptor)
    ↓ /depth_handler/bbox3d
左臂驱动 (robo_ctrl_L: robo_ctrl_node + high_level_node)
    ↓ /L/robot_state, /L/robot_move_cart, /L/robot_act, /L/robot_act_j
右臂驱动 (robo_ctrl_R: robo_ctrl_node + high_level_node)
    ↓ /R/robot_state, /R/robot_move_cart, /R/robot_act, /R/robot_act_j
夹爪节点 (epg50_gripper_node)
    ↓ /gripper_command, /gripper_status
任务管理器 (arm_task_node)
    ↑ 订阅以上所有话题和服务
```

### 常见问题排查

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 服务超时 (MoveCart 30s) | 未启动 ServoMoveStart | 代码已自动处理 (Step 0.5) |
| 夹爪服务不可用 | 夹爪节点未启动或串口错误 | 检查 `/dev/ttyUSB0` 权限，手动启动夹爪节点 |
| 未检测到目标 | 摄像头未启动或 detector 未启动 | 确认 `/camera/color/image_raw` 有数据 |
| MoveCart 错误码 112 | 机器人未就绪 | 检查机器人 IP 连接，重启 robo_ctrl_node |
| MoveCart 错误码 14 | 运动执行失败 | 检查目标位置是否在工作空间内 |
| ros2 命令无响应 | ROS2 daemon 卡住 | `ros2 daemon stop && ros2 daemon start` |

### 虚拟坐标测试

```bash
# 发布单个假目标坐标 (用于测试实机运动)
ros2 run my_arm_control fake_vision_node --ros-args \
  -p target_x:=0.4 -p target_y:=-0.15 -p target_z:=0.3
```

## 双后端架构

- `use_moveit=True`: MoveIt Action Client，用于 fairino3_v6_moveit2_config demo 仿真
- `use_moveit=False`: robo_ctrl 服务接口 (RobotMoveCart, RobotAct, RobotActJ)，用于实机

launch 文件通过 `PythonExpression` 根据 `use_fake_hardware` 自动切换。

## 可用服务接口

| 方法 | 服务 | 说明 |
|------|------|------|
| `_robo_ctrl_move_cart(pose, incremental, side)` | RobotMoveCart | 笛卡尔点到点运动 |
| `_robo_ctrl_act_incremental(dx, dy, dz)` | RobotAct | 笛卡尔增量直线运动 |
| `_robo_ctrl_act_j(joints, incremental, side)` | RobotActJ | 关节空间运动 |
| `_robo_ctrl_arc(center, radian, side)` | RobotAct (plan_type=1) | 圆弧运动 |

所有方法均支持 `side="L"` (左臂) 和 `side="R"` (右臂)。

## 任务模式

通过 `task_mode` 参数选择:

| 模式 | 说明 |
|------|------|
| `grab` | 单臂抓取 (默认) |
| `opencap` | 拧瓶盖 + 倒可乐 (双臂协调) |
| `ball` | 接球 (TODO) |

```bash
# 拧瓶盖任务
ros2 launch my_arm_control real_hardware.launch.py task_mode:=opencap
```

## 比赛进度

---

# Task 1: 双臂倒水服务 (pour_service)

**2026 中国高校智能机器人创意大赛 — 双臂组任务一**

双臂机器人自主识别饮品（怡宝/可乐）和水杯，一手持瓶一手持杯倒水，最后将瓶和杯放回原位。

## 架构

```
视觉节点 (virtual_vision_task1 / 真实视觉)
    ↓ 4× PoseStamped (/vision/bottle_water, /vision/bottle_cola, /vision/cup_1, /vision/cup_2)
    ↓ frame_id: camera_depth_frame
TF 坐标变换 (static_transform_publisher → tf2_ros Buffer)
    ↓ camera_depth_frame → base_link
task1_pour_service (状态机)
    ↓ RobotMoveCart / RobotAct / RobotActJ / GripperCommand
左臂 mock_robo_ctrl_L (或者真实 robo_ctrl_L)    右臂 mock_robo_ctrl_R (或者真实 robo_ctrl_R)
```

## 状态机流程

```
IDLE → OBSERVE → WAIT_VISION → GRASP_BOTTLE → GRASP_CUP
  → OPEN_CAP → POUR → PLACE_BOTTLE → PLACE_CUP
  → NEXT_DRINK → (重复) → DONE
```

| Phase | 名称 | 操作 | 臂 |
|-------|------|------|-----|
| 1 | OBSERVE | 左臂移到观测位，看清台面 | L |
| 2 | GRASP_BOTTLE | 增量运动到瓶上方 → 夹紧 → 垂直撤离 | L |
| 3 | GRASP_CUP | 增量运动到杯上方 → 夹紧 → 垂直撤离 | R |
| 4 | OPEN_CAP | 左臂移拧盖位夹瓶盖 → J6旋转2圈 → 松开 → 回退 | L |
| 5 | POUR | 左臂J6倾斜倒水 → 回正 | L |
| 6 | PLACE_BOTTLE | 增量回到瓶原始位置 → 释放 → 撤退 | L |
| 7 | PLACE_CUP | 增量回到杯原始位置 → 释放 → 撤退 | R |
| 8 | NEXT_DRINK | 处理下一饮品 (water → cola) | — |

## 关键参数

所有参数在 `launch/task1_pour_service.launch.py` 中配置：

### 高度参数 (mm)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `desk_height` | 360.0 | 机器人基座到桌面高度 (需示教) |
| `bottle_height` | 180.0 | 瓶子高度 (瓶底到瓶盖顶部) |
| `cup_height` | 120.0 | 水杯高度 (杯底到杯口) |
| `approach_offset_z` | 150.0 | 接近物体时 TCP 高出物体顶部的距离 |
| `retreat_z` | 80.0 | 抓取后垂直撤离高度 (mm) |

### 运动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `velocity` | 50.0 | 笛卡尔运动速度 (%) |
| `acceleration` | 50.0 | 笛卡尔运动加速度 (%) |
| `observe_pose` | [99.9, -144.2, 542.6, -125.4, 0, -100.5] | 观测位姿 [x,y,z,rx,ry,rz] (需示教) |
| `collision_safety_distance` | 100.0 | 两臂TCP最小安全距离 (mm)，低于此值阻止运动 |

### 拧瓶盖参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cap_open_joints_l` | [-55, -90, -120, 30, 81.3, 0] | 左臂拧盖抓瓶关节角 [j1~j6] (需示教) |
| `cap_open_joints_r` | [45.4, -124.6, 128.4, -184.3, 19.2, 0] | 右臂拧盖抓盖关节角 [j1~j6] (需示教) |
| `pour_j6_angle` | 60.0 | 倒水时 J6 倾斜角度 (度, 需示教) |

### 夹爪参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gripper_id_L` | 9 | 左臂夹爪 Modbus ID |
| `gripper_id_R` | 10 | 右臂夹爪 Modbus ID |

### 饮品顺序

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `drink_order` | ["water", "cola"] | 处理顺序 |

### TF 坐标变换

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_frame` | "base_link" | 控制用坐标系 |
| `vision_frame` | "camera_depth_frame" | 视觉数据原始坐标系 |
| `tf_timeout` | 1.0 | TF 查询超时 (秒) |

## 需要示教的点 (实机部署前)

在使用实机前，需要在机器人上示教以下 5 个点，填入 launch 参数：

| # | 示教点 | 操作方法 | 记录内容 | 对应参数 |
|---|--------|---------|---------|---------|
| ① | **桌面高度** | 左臂 TCP 碰桌面 | tcp_pose.z 值 | `desk_height` |
| ② | **左臂观测位** | 左臂拖到能看清台面的位置 | TCP [x,y,z,rx,ry,rz] | `observe_pose` |
| ③ | **左臂拧盖握瓶** | 左臂末端握住桌上直立的水瓶 | 6个关节角 [j1~j6] | `cap_open_joints_l` |
| ④ | **右臂拧盖握盖** | 右臂从上往下抓住瓶盖 | 6个关节角 [j1~j6] | `cap_open_joints_r` |
| ⑤ | **倒水倾斜角** | 手动倾斜瓶子到水能流出 | J6 增量角度 | `pour_j6_angle` |

### 示教方法

在实机上手动拖动机器人到目标位置，从终端读取关节角 / TCP：

```bash
# 查看左臂当前关节角
ros2 topic echo /L/robot_state --field joint_position --once

# 查看左臂当前 TCP
ros2 topic echo /L/robot_state --field tcp_pose --once

# 查看右臂当前关节角
ros2 topic echo /R/robot_state --field joint_position --once
```

### 示教后参数配置示例

将示教得到的数据填入 `task1_pour_service.launch.py`：

```python
parameters=[{
    'desk_height': 360.0,           # ① 桌面高度
    'observe_pose': [99.917, -144.210, 542.554, -125.357, 0.0, -100.476],  # ② 观测位
    'cap_open_joints_l': [-55.0, -90.0, -120.0, 30.0, 81.272, 0.0],  # ③ 左握瓶
    'cap_open_joints_r': [45.434, -124.551, 128.388, -184.270, 19.218, 0.0],  # ④ 右握盖
    'pour_j6_angle': 60.0,           # ⑤ 倒水角度
}]
```

## 启动方式

### 1. 模拟测试 (无硬件)

一键启动全套模拟 (mock 左右臂 + 虚拟视觉 + TF + 倒水服务):

```bash
ros2 launch my_arm_control task1_pour_service.launch.py
```

启动的节点:
| 节点 | 数量 | 说明 |
|------|------|------|
| mock_robo_ctrl_node | 2 | 模拟 L/R 臂，发布假 robot_state |
| virtual_vision_task1 | 1 | 5Hz 发布 4 个物体的虚拟坐标 |
| static_transform_publisher | 1 | camera_depth_frame → base_link |
| task1_pour_service | 1 | 倒水服务状态机 (延迟 3s 启动) |

### 2. 实机测试

#### 前置启动 (同原实机流程)

```bash
# 终端 1: 摄像头驱动
ros2 launch orbbec_camera gemini_330_series.launch.py

# 终端 2: 左臂驱动 + 视觉管线
ros2 launch robo_ctrl robo_ctrl_L.launch.py

# 终端 3: 右臂驱动
ros2 launch robo_ctrl robo_ctrl_R.launch.py

# 终端 4: 夹爪节点 (左右各一个)
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyUSB0 -p default_slave_id:=9 \
  -r __node:=gripper_node_L

ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyUSB1 -p default_slave_id:=10 \
  -p service_prefix:=R_ -r __node:=gripper_node_R
```

#### 启动倒水服务

```bash
# 终端 5: 启动倒水服务 (需先修改 launch 文件中的示教参数)
ros2 launch my_arm_control task1_pour_service.launch.py
```

> **注意:** 实机运行时，launch 文件中的 `mock_robo_ctrl_L` 和 `mock_robo_ctrl_R` 需要替换为真实 robo_ctrl 节点 (或注释掉)，因为真实驱动已由 robo_ctrl_L/R.launch.py 提供。

## 运动等待机制

代码使用 `_wait_motion_done(side, timeout)` 替代固定 `sleep`：

- 监听 `robot_state.motion_done` 信号
- 运动完成立刻返回 (毫秒级响应)
- 超时 5~15s 报错 (防止死等)
- 支持左右臂独立等待

夹爪操作 (开/合) 仍保留 0.3~0.5s 短延时，因为夹爪执行后需要机械动作完成。

## 碰撞防护机制

每次发运动前自动检查两臂碰撞风险：

| 运动类型 | 检查方式 |
|---------|---------|
| `_move_cart` (绝对) | 检查目标 TCP 与静止臂 TCP 的距离 |
| `_move_cart` (增量) | 当前 TCP + 增量 → 算出目标位置，再与静止臂 TCP 比对 |
| `_act_incremental` | 同上 (增量 → 目标 → 比对) |
| `_act_j` (关节) | 检查运动臂当前 TCP 与静止臂 TCP 的距离 |

**触发条件：** 两臂 TCP 距离 < `collision_safety_distance` (默认 100mm)

**触发行为：** 日志打印 `碰撞风险: L臂目标距R臂TCP仅 XX.Xmm (阈值 100.0mm) — 已阻止`，返回 `False`，任务进入 ERROR 状态。

**局限性：**
- TCP 距离 ≠ 手臂本体距离 (手臂比 TCP 粗，且姿态不同时本体位置不同)
- 示教拧盖位时仍需用人眼确认两臂有足够间距 (建议 ≥ 200mm)

## 视觉坐标变换流程

```
视觉节点发布 PoseStamped(frame_id=camera_depth_frame)
    ↓
task1_pour_service 收到后:
    ├─ 如果 frame_id != base_link:
    │   └─ tf_buffer.lookup_transform(base_link, camera_depth_frame)
    │   └─ do_transform_pose_stamped(pose, transform)
    │   └─ 得到 base_link 坐标系下的 x/y/z
    └─ 存入 self._objects
```

相机外参标定完成后修改 launch 文件中的 `static_transform_publisher` 参数:

```python
camera_to_base = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_camera_to_base',
    arguments=[
        '--x', '标定结果x',      # ← 修改这里
        '--y', '标定结果y',
        '--z', '标定结果z',
        '--roll', '标定结果roll',
        '--pitch', '标定结果pitch',
        '--yaw', '标定结果yaw',
        '--frame-id', 'base_link',
        '--child-frame-id', 'camera_depth_frame',
    ],
)
```

## 常见问题 (Task 1 专项)

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| `_wait_motion_done` 超时 | 实机 `motion_done` 一直为 False | 检查 robo_ctrl 节点是否正常 |
| 夹爪服务超时 | 夹爪节点未启动或串口权限 | `sudo chmod 666 /dev/ttyUSB*` |
| 视觉数据一直 0/4 | 视觉节点未启动或话题名不匹配 | `ros2 topic list \| grep /vision/` |
| TF 变换失败 | camera_depth_frame 不存在 | `ros2 run tf2_ros tf2_echo base_link camera_depth_frame` |
| 增量运动方向错误 | 视觉坐标在 camera_frame 未变换 | 检查 static_transform_publisher 参数 |

---

已完成的功能清单:

- [x] RobotActJ 服务封装 (关节空间运动)
- [x] 右臂 service client (/R/robot_move_cart, /R/robot_act_j)
- [x] 圆弧运动 (RobotAct plan_type=1, circle_center + radian)
- [x] 右臂状态订阅 (/R/robot_state)
- [x] 姿态修正方法 (_fix_orientation)
- [x] 预设关节角度 (config.yaml 全部导入)
- [x] 夹爪 ID 参数化 (支持左臂=9, 右臂=10)
- [x] 拧瓶盖任务编排 (13步完整流程)
- [x] 拧瓶盖循环 (3轮: 夹→转60度→松→回)
- [x] 倒可乐动作 (关节 J6 轴旋转)
- [x] 双臂协调状态机
- [x] Kalman 滤波集成
- [x] ServoMoveStart 伺服模式启动
- [x] 碰撞防护机制
- [ ] 接球任务流程

## 变更日志

### 2026-05-24 — 碰撞检测与运动规划逻辑

#### 新增: 碰撞防护机制

- 新增 `_check_collision_risk(target_xyz, moving_side)` 方法
- 在 `_move_cart`、`_act_incremental`、`_act_j` 中集成碰撞检查
- 新增参数 `collision_safety_distance` (默认 100mm)
- 三种运动类型的检查方式: 绝对笛卡尔检查目标位置、增量笛卡尔推算目标位置、关节运动检查当前 TCP 距离
- 触发时打印具体距离并返回 False，阻止危险运动

#### 其他修改

- 更新 README.md: 添加碰撞防护、运动规划逻辑章节

### 2026-05-05 (实机调试完整记录)

#### 问题 1: 夹爪服务名称不匹配
- **现象:** `夹爪服务不可用, 跳过夹爪控制`
- **原因:** 代码中使用 `/epg50_gripper/command`，但 epg50_gripper_ros 实际服务名是 `gripper_command`
- **修复:** 修改 `arm_task_manager.py` 第 357 行，服务名改为 `gripper_command`

#### 问题 2: MoveCart 错误码 112 (机器人未就绪)
- **现象:** `服务 L/robot_move_cart 失败: 错误码 112`
- **原因:** 机器人上电后需要先发送 ServoMoveStart 命令才能接受运动指令
- **修复:** 添加 `_servo_move_start()` 方法 (command_type=0, RobotAct)，在 Step 0.5 调用

#### 问题 3: MoveCart 错误码 14 (执行失败)
- **现象:** `服务 L/robot_move_cart 失败: 错误码 14`
- **原因:** ServoMoveStart 后笛卡尔运动仍失败，可能因为目标位置超出工作空间或姿态奇异
- **修复:** Step 1 改用关节运动 (`_robo_ctrl_act_j`) 替代笛卡尔运动 (`_robo_ctrl_move_cart`)

#### 问题 4: Python 类型错误 (INTEGER vs DOUBLE)
- **现象:** `InvalidParameterTypeException: Trying to set parameter 'velocity' to '50' of type 'INTEGER', expecting type 'DOUBLE'`
- **原因:** launch 文件传入整数 `velocity` 而非浮点数
- **修复:** launch 文件中 `"velocity": velocity` 改为 `"velocity": 50.0`

#### 问题 5: TCPPose 字段类型错误
- **现象:** `The 'x' field must be of type 'float'`
- **原因:** Python int 0 传给 ROS2 float 字段
- **修复:** `_fix_orientation()` 和 `_robo_ctrl_move_cart()` 中所有 TCPPose 字段加 `float()` 转换

#### 问题 6: 姿态修正超时 (30s)
- **现象:** `服务 L/robot_move_cart 超时 (30.0s)`，姿态差异 516.6°
- **原因:** `_fix_orientation()` 计算的姿态差过大，机器人无法在 30s 内完成
- **修复:** 跳过 Step 3 姿态修正，关节运动已到达正确位置

#### 问题 7: 左臂 robo_ctrl_node 缺失
- **现象:** `ps aux` 中看不到 Lrobo_ctrl_node 进程
- **原因:** robo_ctrl_L.launch.py 启动时 robo_ctrl_node 崩溃 (可能 IP 连接问题)
- **修复:** 重启 robo_ctrl_L.launch.py，确认 Lrobo_ctrl_node 和 Lhigh_level 都在运行

#### 问题 8: 重复启动导致冲突
- **现象:** 两个 robo_ctrl_L 实例同时运行
- **原因:** 多次启动 launch 文件未先清理旧进程
- **修复:** `kill` 旧进程后再启动新实例

#### 问题 9: ROS2 daemon 卡住
- **现象:** `ros2 topic list` 等命令无响应，报 `TimeoutError: [Errno 110] Connection timed out`
- **原因:** ROS2 daemon 进程异常
- **修复:** `pkill -f 'ros2 daemon'`，然后用 `--no-daemon` 参数执行命令

#### 问题 10: 摄像头未启动
- **现象:** 未检测到目标 (class_id=1)，`/camera/color/image_raw` 无数据
- **原因:** Orbbec Gemini 335 摄像头驱动未启动
- **修复:** `ros2 launch orbbec_camera gemini_330_series.launch.py`

#### 问题 11: 夹爪 slave_id 配置
- **现象:** 夹爪使能失败
- **原因:** 左臂夹爪 slave_id=9，右臂夹爪 slave_id=10，需要正确配置
- **修复:** 启动夹爪时指定 `default_slave_id:=10`，代码中按任务使用不同 ID

#### 问题 12: 双夹爪节点服务名冲突
- **现象:** 两个夹爪节点都注册 `/gripper_command`，ROS2 随机路由导致一个夹爪不可控
- **原因:** 左臂夹爪在 ttyUSB0 (slave_id=9)，右臂夹爪在 ttyUSB1 (slave_id=10)，需要两个节点但服务名相同
- **修复:**
  1. `epg50_gripper_node.cpp` 添加 `service_prefix` 参数
  2. 左臂节点: 无前缀 → 服务名 `gripper_command`
  3. 右臂节点: `service_prefix:=R_` → 服务名 `R_gripper_command`
  4. `arm_task_manager.py` 添加 `_R_gripper_client`，`_get_gripper_client()` 按 gripper_id 选择客户端

#### 当前状态
- [x] 双臂服务就绪 (L/R robot_move_cart, robot_act, robot_act_j)
- [x] 摄像头驱动就绪 (/camera/color/image_raw)
- [x] 视觉管线就绪 (detector + depth_handler)
- [x] 夹爪服务就绪 (/gripper_command)
- [ ] 完整 opencap 任务端到端验证

#### 代码修改汇总
- `arm_task_manager.py`: 服务名修复、添加 ServoMoveStart、float 转换、跳过姿态修正、双夹爪服务支持
- `real_hardware.launch.py`: velocity 类型修复、desk_height 更新、添加关节参数
- `epg50_gripper_node.cpp`: 添加 `service_prefix` 参数，支持多夹爪节点不同服务名
- `setup.py`: 清理无效 data_files
- `README.md`: 更新启动顺序文档、变更日志

### 2026-05-01 (6)
- 添加 Kalman 滤波 + 目标跟踪 (`_Kalman1D`, `TrackedObject`)
- 距离关联 (150mm 阈值) 合并同一目标
- 帧计数过期 (5帧) 丢弃消失目标
- 有效检测次数过滤 (≥1 次才有效)
- 新增 5 个滤波参数可配置

### 2026-05-01 (5)
- 修复左臂 IP: → 192.168.58.2
- 修复右臂 IP: → 192.168.58.3
- 修复左臂夹爪串口: /dev/ttyACM0 → /dev/ttyUSB0
- real_hardware.launch.py 移除重复的视觉节点，只保留 arm_task_node
- 修正夹爪 ID: 左臂=9, 右臂=10

### 2026-05-01 (4)
- 添加夹爪使能步骤 `_enable_gripper()` — command=1 (GRIPPER_ENABLE)
- opencap 任务启动时自动使能左右夹爪 (ID=9, 10)
- 添加检测暂停标志 `_detection_paused` — 抓取过程中暂停 bbox 回调
- opencap 任务自动设置 `target_class_id=1` (可乐), 完成后恢复
- 放回杯子时先移到 `_place_pose` 再松手 (之前是原地松手)

### 2026-05-01 (3)
- 添加右臂状态订阅 `/R/robot_state` → `_R_current_tcp`
- 添加 `_fix_orientation()` 方法 — 姿态修正到目标 rx/ry/rz
- 夹爪命令 `_open_gripper` / `_close_gripper` 支持 `gripper_id` 参数 (左臂=9, 右臂=10)
- 导入全部预设关节角度: cap_open_joints_l/r, ball_*_joint_pose (7组)
- 添加 `task_mode` 参数: "grab"(单臂), "opencap"(拧瓶盖), "ball"(接球)
- 实现拧瓶盖完整流程 `_task_opencap()` (13步, 双臂协调)
- 添加 cola_offset_x/y 参数 (-132, +45) 用于接近可乐校准

### 2026-05-01 (2)
- 添加 RobotActJ 服务封装 `_robo_ctrl_act_j()` — 关节空间运动，支持绝对/增量模式
- 添加圆弧运动封装 `_robo_ctrl_arc()` — RobotAct plan_type=1
- 添加右臂全套 service client (/R/robot_move_cart, /R/robot_act, /R/robot_act_j)
- 右臂服务为可选 (5s 超时)，不可用时 `_right_arm_ready=False`
- `_robo_ctrl_move_cart` 新增 `side` 参数支持双臂
- 更新 README.md 接口文档

### 2026-05-01 (1)
- 添加高度参数 `desk_height` (177mm) 和 `object_height` (89mm)
- 接近目标 z 轴使用 `desk_height + object_height - current_tcp_z` 计算，与 dualarm 一致
- 更新 real_hardware.launch.py 传入高度参数
- 创建 README.md 文档
