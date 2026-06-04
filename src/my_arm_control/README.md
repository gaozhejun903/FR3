# my_arm_control

双臂控制与虚拟视觉节点包，支持**三后端**运动控制模式：

| 模式 | 参数 | 说明 | 适用场景 |
|------|------|------|---------|
| MoveIt (仿真) | `use_moveit:=true` | MoveGroup Action 规划+执行 | Gazebo/仿真 |
| MoveIt+robo_ctrl (混合) | `use_moveit_robo:=true` | MoveIt 规划 + robo_ctrl RobotServoJoint 执行 | **实机（推荐）** |
| robo_ctrl (实机) | (默认) | robo_ctrl 线性插值直接发送 | 实机（旧方式） |

## 节点

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| arm_task_manager | `arm_task_node` | 核心任务管理器，状态机驱动 |
| task1_pour_service | `task1_pour_service` | 任务一: 倒水服务状态机 |
| virtual_vision_node | `virtual_vision_node` | 仿真用虚拟视觉 |
| fake_vision_node | `fake_vision_node` | 实机测试用，发布固定坐标 |
| virtual_vision_task1 | `virtual_vision_task1` | 仿真任务一 |
| mock_robo_ctrl_node | `mock_robo_ctrl_node` | 实机测试用 mock 服务 |

## 控制链路

### 1. MoveIt 仿真模式 (`use_moveit:=true`)

```
PoseStamped → MoveGroup Action (fairino3_v6_moveit2_config) → 实机/仿真
```

### 2. MoveIt+robo_ctrl 混合模式 (`use_moveit_robo:=true`) — 推荐

```
目标位姿 → fairino3_v6_planner (OMPL 规划) → JointState[] 轨迹点
         → robo_ctrl RobotServoJoint (ServoMoveStart + 轨迹点 + ServoMoveEnd) → 实机
```

核心模块: [moveit_controller.py](my_arm_control/moveit_controller.py)

### 3. robo_ctrl 线性插值模式 (默认, `use_moveit_robo:=false`)

```
目标位姿 → robo_ctrl RobotMoveCart/RobotActJ (内部线性插值) → 实机
```

## 运行方式

### 混合模式 (实机推荐)

```bash
# 终端1: 启动硬件驱动 + robo_ctrl
ros2 launch fairino3_v6_bringup fairino_bringup.launch.py

# 终端2: 启动 MoveIt 规划器
ros2 launch fairino3_v6_planner planner.launch.py

# 终端3: 启动视觉检测
ros2 launch depth_handler depth_full.launch.py service_prefix:=/vision

# 终端4: 启动双臂任务 (混合模式)
ros2 launch my_arm_control arm_task.launch.py use_moveit_robo:=true

# 或启动倒水服务 (混合模式)
ros2 run my_arm_control task1_pour_service --ros-args -p use_moveit_robo:=true
```

### robo_ctrl 模式 (实机传统方式)

```bash
# 终端1: 启动硬件驱动 + robo_ctrl
ros2 launch fairino3_v6_bringup fairino_bringup.launch.py

# 终端2: 启动视觉检测
ros2 launch depth_handler depth_full.launch.py service_prefix:=/vision

# 终端3: 启动双臂任务
ros2 launch my_arm_control arm_task.launch.py
```

### 仿真模式 (MoveIt demo)

```bash
ros2 launch fairino3_v6_moveit2_config demo.launch.py
ros2 launch my_arm_control arm_task.launch.py use_moveit:=true
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_moveit` | false | 启用 MoveIt 仿真模式 |
| `use_moveit_robo` | false | 启用混合模式 (MoveIt 规划 + robo_ctrl 执行) |
| `use_virtual_vision` | false | 使用虚拟视觉 |
| `desk_height` | 360.0 | 桌面高度 (mm) |
| `object_height` | 89.0 | 物体高度 (mm) |
| `robot_prefix` | "/L" | 主臂前缀 |
| `velocity` | 50.0 | 运动速度 |
| `acceleration` | 50.0 | 运动加速度 |

## 状态机

```
IDLE -> OBSERVATION -> APPROACHING -> GRABBING -> RETREATING -> PLACING -> IDLE
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `arm_task_manager.py` | 核心任务管理器，完整状态机逻辑 |
| `task1_pour_service.py` | 倒水任务状态机 (OBSERVE→GRASP→POUR→PLACE) |
| `moveit_controller.py` | MoveIt 规划 + robo_ctrl 执行的混合控制器 |
| `gripper_controller.py` | EPG50 夹爪串口控制器 |
| `virtual_vision_node.py` | 仿真用虚拟视觉节点 |
| `fake_vision_node.py` | 实机测试用固定视觉节点 |
