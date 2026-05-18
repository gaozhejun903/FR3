# Robo Ctrl

这个ROS 2包提供了控制Movecat机器人移动的服务接口。

## 功能特性

- 提供RobotMove服务用于控制机器人关节和笛卡尔空间的移动
- 提供RobotMoveCart服务用于更精确的笛卡尔空间移动控制
- 提供RobotServo服务用于伺服控制，支持ServoJ、ServoCart、ServoMoveStart和ServoMoveEnd
- 支持配置机器人IP和端口
- 包含示例客户端代码以演示如何调用服务
- 实时发布机器人状态信息
- 支持TF变换广播，便于可视化和坐标转换
- 自动错误检测和恢复机制

## 依赖项

- ROS 2 (Foxy或更高版本)
- libfairino (机器人控制库)
- tf2_ros (TF变换广播)

## 编译

```bash
cd ~/roboarm/FairinoDualArm
colcon build --symlink-install --packages-select robo_ctrl
source install/setup.bash
```

## 使用方法

### 启动机器人控制节点

```bash
# 使用默认参数启动
ros2 launch robo_ctrl robo_ctrl.launch.py

# 指定机器人IP和端口
ros2 launch robo_ctrl robo_ctrl.launch.py robot_ip:=192.168.1.100 robot_port:=8080

# 指定状态查询间隔(秒)
ros2 launch robo_ctrl robo_ctrl.launch.py state_query_interval:=0.2
```

### 运行示例客户端

```bash
ros2 run robo_ctrl robot_move_client
```

### 通过服务接口控制机器人

可以使用ROS 2服务直接调用机器人移动服务：

```bash
# 关节空间移动
ros2 service call /robot_move robo_ctrl/srv/RobotMove "{move_type: 0, joint_positions: [4.365596771240234,-77.84477233886719,-16.82822608947754,-92.0924072265625,82.69158935546875,-60.655428171157837], velocity: 50.0, acceleration: 30.0}"

# 笛卡尔空间移动
ros2 service call /robot_move robo_ctrl/srv/RobotMove "{move_type: 1, cartesian_pose: [400.0, 0.0, 500.0, 180.0, 0.0, 180.0], velocity: 30.0, acceleration: 20.0}"

# 更精确的笛卡尔空间移动
ros2 service call /robot_move_cart robo_ctrl/srv/RobotMoveCart "{
  tcp_pose: {x: 400.0, y: 0.0, z: 500.0, rx: 180.0, ry: 0.0, rz: 180.0}, 
  velocity: 30.0, 
  acceleration: 20.0, 
  ovl: 100.0, 
  blend_time: -1.0, 
  tool: 0, 
  user: 0, 
  config: -1
  use_increment: false
}"
```

## 服务接口说明

### RobotMove服务

**请求字段:**
- `move_type`: 移动类型 (0=关节移动, 1=笛卡尔移动)
- `joint_positions`: 关节位置数组 [joint1, joint2, ...]，单位为度
- `cartesian_pose`: 笛卡尔位姿数组 [x, y, z, rx, ry, rz]，位置单位为毫米，角度单位为度
- `velocity`: 速度百分比 (0-100)
- `acceleration`: 加速度百分比 (0-100)

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

### RobotMoveCart服务

**请求字段:**
- `tcp_pose`: TCP位姿结构体
  - `x`, `y`, `z`: 位置坐标，单位为毫米
  - `rx`, `ry`, `rz`: 旋转角度，单位为度
- `velocity`: 速度百分比 (0-100)
- `acceleration`: 加速度百分比 (0-100)
- `ovl`: 速度缩放因子 (0-100)
- `blend_time`: 平滑时间，-1表示阻塞运动
- `tool`: 工具坐标系号 (-0-9)
- `user`: 工件坐标系号 (-0-9)
- `config`: 关节空间构型，-1表示使用当前构型
- `use_increment`: 是否使用增量模式 (true/false)`

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

### RobotServo服务

RobotServo服务提供了机器人伺服控制功能，支持四种命令类型：ServoMoveStart、ServoMoveEnd、ServoJ和ServoCart。

**请求字段:**
- `command_type`: 命令类型
  - `0`: ServoMoveStart - 启动伺服控制模式
  - `1`: ServoMoveEnd - 结束伺服控制模式
  - `2`: ServoJ - 关节空间伺服运动
  - `3`: ServoCart - 笛卡尔空间伺服运动
- `joint_positions`: 关节位置数组 [joint1, joint2, ...]，单位为度，用于ServoJ命令
- `acc`: 加速度百分比 (0-100)
- `vel`: 速度百分比 (0-100)
- `cmd_time`: 指令传递周期，单位为秒，推荐范围 [0.001-0.0016]
- `filter_time`: 滤波时间常数，单位为秒
- `gain`: 目标位置比例放大器，默认为0
- `mode`: ServoCart专用，运动模式
  - `0`: 绝对运动(基座标系)
  - `1`: 增量运动(基座标系)
  - `2`: 增量运动(工具坐标系)
- `cartesian_pose`: 笛卡尔位姿数组 [x, y, z, rx, ry, rz]，用于ServoCart命令
- `pos_gain`: 位姿增量比例系数数组，仅对增量运动有效，范围 [0-1]，6个元素分别对应[x, y, z, rx, ry, rz]

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

### RobotAct服务
**请求字段:**
- `command_type`: 命令类型 (0=ServoMoveStart, 1=ServoMoveEnd)
- `tcp_pose`: TCP位姿结构体 (`robo_ctrl/msg/TCPPose`)
- `point_count`: 轨迹点数
- `message_time`: 单步运动时间 (秒)
- `plan_type`: 轨迹规划类型 (0=直线, 1=圆弧)
- `use_incremental`: 是否使用增量运动模式
- `circle_center`: 圆弧中点坐标 (`geometry_msgs/Point`)

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

**使用示例:**
```bash
ros2 service call /robot_act robo_ctrl/srv/RobotAct "{
  command_type: 0,
  tcp_pose: {x: 400.0, y: 0.0, z: 500.0, rx: 0.0, ry: 0.0, rz: 0.0},
  point_count: 3,
  message_time: 0.02,
  plan_type: 0,
  use_incremental: false
}"
```

### RobotServoJoint服务
**请求字段:**
- `command_type`: 命令类型 (0=ServoMoveStart, 1=ServoMoveEnd)
- `joint_positions`: 关节位置数组 (`sensor_msgs/JointState[]`)
- `acc`: 加速度百分比 (0-100)
- `vel`: 速度百分比 (0-100)
- `cmd_time`: 指令传递周期 (秒)
- `filter_time`: 滤波时间常数 (秒)
- `gain`: 目标位置比例放大器
- `use_incremental`: 是否使用增量运动模式

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

**使用示例:**
```bash
ros2 service call /robot_servo_joint robo_ctrl/srv/RobotServoJoint "{
  command_type: 2,
  joint_positions: [{name: 'joint1', position: 10.0}, {name: 'joint2', position: 20.0}],
  acc: 30.0,
  vel: 50.0,
  cmd_time: 0.001,
  filter_time: 0.0,
  gain: 0.0,
  use_incremental: true
}"
```

### RobotServoLine服务
**请求字段:**
- `command_type`: 命令类型 (0=ServoMoveStart, 1=ServoMoveEnd)
- `cartesian_pose`: 笛卡尔位姿数组 (`geometry_msgs/PoseArray`)
- `acc`: 加速度百分比 (0-100)
- `vel`: 速度百分比 (0-100)
- `cmd_time`: 指令传递周期 (秒)
- `filter_time`: 滤波时间常数 (秒)
- `gain`: 目标位置比例放大器
- `point_count`: 目标点数量
- `use_incremental`: 是否使用增量运动模式

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

**使用示例:**
```bash
ros2 service call /robot_servo_line robo_ctrl/srv/RobotServoLine "{
  command_type: 2,
  cartesian_pose: [{position: {x: 100.0, y: 0.0, z: 100.0}, orientation: {x:0.0,y:0.0,z:0.0,w:1.0}}],
  acc: 25.0,
  vel: 40.0,
  cmd_time: 0.001,
  filter_time: 0.0,
  gain: 0.0,
  point_count: 1,
  use_incremental: false
}"
```

### RobotSetSpeed服务
**请求字段:**
- `speed`: 全局速度百分比 (0-100)

**响应字段:**
- `success`: 操作是否成功
- `message`: 返回消息或错误描述

**使用示例:**
```bash
ros2 service call /robot_set_speed robo_ctrl/srv/RobotSetSpeed "{speed: 60}"
```

### 伺服控制使用示例

伺服控制通常用于需要频繁小幅度调整机器人位置的场景，如视觉伺服、力控制等。使用伺服控制的一般步骤是：

1. 调用ServoMoveStart启动伺服控制模式
2. 以一定频率循环调用ServoJ或ServoCart发送伺服命令
3. 完成任务后调用ServoMoveEnd结束伺服控制模式

```bash
# 启动伺服控制模式
ros2 service call /robot_servo robo_ctrl/srv/RobotServo "{command_type: 0}"

# 关节空间伺服运动 - ServoJ
ros2 service call /robot_servo robo_ctrl/srv/RobotServo "{
  command_type: 2, 
  joint_positions: [10.0, 20.0, 30.0, 40.0, 50.0, 60.0], 
  acc: 30.0, 
  vel: 50.0, 
  cmd_time: 0.001, 
  filter_time: 0.0, 
  gain: 0.0
}"

# 笛卡尔空间伺服运动 - ServoCart (绝对运动模式)
ros2 service call /robot_servo robo_ctrl/srv/RobotServo "{
  command_type: 3, 
  mode: 0, 
  cartesian_pose: [400.0, 0.0, 500.0, 180.0, 0.0, 180.0], 
  pos_gain: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 
  acc: 30.0, 
  vel: 50.0, 
  cmd_time: 0.001, 
  filter_time: 0.0, 
  gain: 0.0
}"

# 笛卡尔空间伺服运动 - ServoCart (增量运动模式，基座标系)
ros2 service call /robot_servo robo_ctrl/srv/RobotServo "{
  command_type: 3, 
  mode: 1, 
  cartesian_pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
  pos_gain: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 
  acc: 30.0, 
  vel: 50.0, 
  cmd_time: 0.001, 
  filter_time: 0.0, 
  gain: 0.0
}"

# 笛卡尔空间伺服运动 - ServoCart (增量运动模式，工具坐标系)
ros2 service call /robot_servo robo_ctrl/srv/RobotServo "{
  command_type: 3, 
  mode: 2, 
  cartesian_pose: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 
  pos_gain: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 
  acc: 30.0, 
  vel: 50.0, 
  cmd_time: 0.001, 
  filter_time: 0.0, 
  gain: 0.0
}"

# 结束伺服控制模式
ros2 service call /robot_servo robo_ctrl/srv/RobotServo "{command_type: 1}"
```

### 伺服控制参数说明

**ServoJ 和 ServoCart 共同参数:**
- `acc`: 加速度百分比，影响伺服运动的加速性能，推荐范围 [10-100]
- `vel`: 速度百分比，影响伺服运动的最大速度，推荐范围 [10-100]
- `cmd_time`: 指令周期，是控制命令的时间间隔，必须与实际发送命令的频率匹配。推荐值为0.001秒(1kHz)到0.0016秒(625Hz)
- `filter_time`: 滤波时间常数，用于平滑运动轨迹，值越大，运动越平滑但响应越慢
- `gain`: 目标位置比例放大器，可用于增加或减小伺服命令的作用力度，默认为0

**ServoCart 特有参数:**
- `mode`: 运动模式，决定了位置坐标的参考系
- `pos_gain`: 位姿增量比例系数，仅在增量模式有效，用于控制每个自由度的运动比例

## 伺服控制最佳实践

1. 在启动伺服控制前，确保机器人处于安全且稳定的位置
2. 保持伺服命令的发送频率与`cmd_time`参数匹配，过快或过慢都会影响控制精度
3. 初始使用时，建议将`acc`和`vel`设置为较低值，如20%和30%
4. 对于精细操作，尝试使用较小的`filter_time`值和较大的`gain`值
5. 完成伺服操作后，务必调用ServoMoveEnd结束伺服模式
6. 在增量模式下，小的增量值能提供更平滑的运动

## 话题接口说明

### 机器人状态发布

节点实时发布机器人状态信息到 `/robot_state` 话题：

```
# 关节位置信息
robo_ctrl/msg/JointPosition joint_position
  double j1, j2, j3, j4, j5, j6

# TCP位姿信息
robo_ctrl/msg/TCPPose tcp_pose
  double x, y, z, rx, ry, rz

# 运动状态
bool motion_done

# 错误代码
int32 error_code
```

## TF变换

节点会广播以下TF变换：
- `world` → `robot_base`: 世界坐标系到机器人基坐标系
- `robot_base` → `tcp`: 机器人基坐标系到末端执行器
- `robot_base` → `target_pose`: 目标位姿（仅在执行MoveCart命令时）

## 错误处理

节点内置了自动错误检测和恢复机制，可以处理以下常见错误：
- 无法到达的目标位姿
- 奇异点和构型问题
- 通信中断
- 运动规划失败

当出现错误时，系统会尝试：
1. 消除机器人错误
2. 返回到安全位置
3. 提供详细的错误信息
