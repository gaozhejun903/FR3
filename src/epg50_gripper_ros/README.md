# EPG50_Serial 夹爪串行通信库

## AI-Deep: 双夹爪独立总线部署指南

### 硬件拓扑

```
电脑 USB-HUB ─┬─ PL2303#1 (USB口3-1.1) ─ RS485 ─ 右爪 (ID=9)
              ├─ PL2303#2 (USB口3-1.4) ─ RS485 ─ 左爪 (ID=9)
              └─ 相机
```

两个夹爪使用**独立 RS-485 总线**，因此从站 ID 可以相同（均为 9），不冲突。

### 找出哪个端口对应哪个爪子

```bash
# 查看当前 USB 串口设备
ls /dev/ttyUSB*

# 逐个裸测 Modbus 命令
for p in /dev/ttyUSB*; do
    python3 -c "
import serial
s = serial.Serial('$p', 115200, timeout=0.3)
cmd = bytes([0x09, 0x03, 0x07, 0xD0, 0x00, 0x04, 0x45, 0xCC])
s.write(cmd); s.flush(); r = s.read(64)
print('$p:', r.hex()[:30] if r else '无响应')
s.close()
" 2>/dev/null
done
```

> **注意：** 端口编号 `/dev/ttyUSB0`、`/dev/ttyUSB1` 等会随 USB 插拔顺序变化，不建议硬编码端口号。可创建 udev 规则固定别名，或每次启动前先用上述方法确认。

### 启动双夹爪节点

服务名已改为**相对命名**（`~/command`），通过节点名区分：

```bash
# 启动右爪节点
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyUSB0 -r __node:=R_gripper_node &

# 启动左爪节点
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyUSB2 -r __node:=L_gripper_node &
```

验证服务互不冲突：
```bash
ros2 service list | grep command
# 应看到 /L_gripper_node/command 和 /R_gripper_node/command
```

### service call 控制

```bash
# === 左爪 ===
# 使能
ros2 service call /L_gripper_node/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 9, command: 1, position: 0, speed: 255, torque: 255}"
# 打开
ros2 service call /L_gripper_node/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 9, command: 2, position: 0, speed: 255, torque: 255}"
# 闭合
ros2 service call /L_gripper_node/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 9, command: 2, position: 255, speed: 255, torque: 255}"

# === 右爪 ===
# 使能
ros2 service call /R_gripper_node/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 9, command: 1, position: 0, speed: 255, torque: 255}"
# 打开
ros2 service call /R_gripper_node/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 9, command: 2, position: 0, speed: 255, torque: 255}"
# 闭合
ros2 service call /R_gripper_node/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 9, command: 2, position: 255, speed: 255, torque: 255}"
```

### 在 C++ 代码中使用

```cpp
// 左爪
auto L_req      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
L_req->slave_id = 9;
L_req->command  = 2;     // SET
L_req->position = 255;   // CLOSE
L_req->speed    = 255;
L_req->torque   = 255;
ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    node->gripper_command_client_,       // → /L_gripper_node/command
    L_req, node, std::chrono::seconds(5), "gripper_L");

// 右爪
auto R_req      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
R_req->slave_id = 9;
R_req->command  = 2;
R_req->position = 0;    // OPEN
R_req->speed    = 255;
R_req->torque   = 255;
ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    node->R_gripper_command_client_,     // → /R_gripper_node/command
    R_req, node, std::chrono::seconds(5), "gripper_R");
```

### 命令参数速查

| 参数 | 含义 | 值 |
|---|---|---|
| `slave_id` | 从站ID | 当前双总线均为 9 |
| `command` | 指令类型 | 0=失能, 1=使能, 2=设置参数 |
| `position` | 夹爪位置 | 0=全开, 255=全闭 |
| `speed` | 速度 | 0-255 |
| `torque` | 力矩 | 0-255 |

---

## 简介

EPG50_Serial 是一个用于与EPG50机械夹爪进行串口通信的C++库。该库实现了基于Modbus RTU协议的命令发送和接收功能，可以控制夹爪的开合、设置夹爪参数（位置、速度、力矩）以及读取夹爪状态。

## 功能特性

- 夹爪使能/禁用控制
- 夹爪参数设置（位置、速度、力矩）
- 夹爪状态读取
- 故障检测和诊断
- 支持调试模式

## 要求

- Linux操作系统
- 串口设备（默认为/dev/ttyACM0）
- C++11或更高版本

## 快速开始

### service call

为ros2 epg50_serial/EPG50_Serial 发送service call

```service
# 夹爪控制命令
uint8 slave_id # 从站ID
uint8 command  # 命令类型: 0=禁用, 1=使能, 2=设置参数

# 仅在设置参数时使用
uint8 position  # 位置参数 0-255
uint8 speed     # 速度参数 0-255
uint8 torque    # 力矩参数 0-255
---
bool success    # 操作是否成功
string message  # 返回信息
```

```
# 获取夹爪状态
uint8 slave_id # 从站ID
---
bool success         # 操作是否成功
uint16 status        # 状态
uint16 mode          # 模式
uint16 error         # 错误代码
uint16 position      # 当前位置
uint16 speed         # 当前速度
uint16 force         # 当前力
string error_message # 错误信息
```

### 初始化夹爪

```cpp
#include <serial/serial.hpp>

int main() {
    // 使用默认端口和从站ID初始化夹爪
    EPG50_Serial gripper;
    
    // 或者指定端口和从站ID
    // EPG50_Serial gripper("/dev/ttyUSB0", 0x09);
    
    // 启用调试输出（可选）
    gripper.debug = true;
    
    // 使能夹爪
    if (gripper.enable()) {
        std::cout << "夹爪已使能" << std::endl;
    } else {
        std::cout << "夹爪使能失败" << std::endl;
        return -1;
    }
    
    return 0;
}
```

### 控制夹爪

```cpp
// 设置夹爪参数：位置、速度、力矩
// 位置: 0x00-0xFF (0为完全打开，255为完全闭合)
// 速度: 0x00-0xFF (0为最慢，255为最快)
// 力矩: 0x00-0xFF (0为最小，255为最大)
gripper.set_parameters(0x80, 0xA0, 0x60);

// 完全打开夹爪
gripper.full_open();

// 禁用夹爪
gripper.disable();
```

### 读取夹爪状态

```cpp
std::vector<uint16_t> status = gripper.read_status();
if (!status.empty()) {
    std::cout << "夹爪状态: " << status[0] << std::endl;
    std::cout << "工作模式: " << status[1] << std::endl;
    std::cout << "错误码: " << gripper.check_errors(status[2]) << std::endl;
    std::cout << "当前位置: " << status[3] << std::endl;
    std::cout << "当前速度: " << status[4] << std::endl;
    std::cout << "当前力矩: " << status[5] << std::endl;
}
```

## API参考

### 构造函数

```cpp
EPG50_Serial(const std::string& port = "/dev/ttyACM0", const uint8_t slave_id = 0x09)
```

- `port`: 串口设备路径，默认为"/dev/ttyACM0"
- `slave_id`: 从站ID，默认为0x09

### 夹爪控制

```cpp
bool enable()              // 使能夹爪
bool disable()             // 禁用夹爪
bool full_open()           // 完全打开夹爪

// 设置夹爪参数
bool set_parameters(uint8_t position, uint8_t speed, uint8_t torque)
```

### 状态读取

```cpp
std::vector<uint16_t> read_status()  // 读取夹爪状态
```
返回值是一个包含6个寄存器值的向量：
1. 夹爪状态
2. 工作模式
3. 错误码
4. 当前位置
5. 当前速度
6. 当前力矩

### 故障诊断

```cpp
std::string check_errors(uint8_t error_status)  // 根据错误码返回故障描述
```

## 通信协议

该库使用Modbus RTU协议与夹爪通信。默认通信参数：
- 波特率：115200
- 数据位：8
- 停止位：1
- 校验位：无

## 错误处理

库中实现了错误检测和异常处理：
- 通信超时检测
- CRC16校验
- 响应完整性验证
- 错误状态解析

## 故障排查

### 现象：节点初始化成功，但所有命令均返回失败

```
[INFO] [epg50_gripper]: 初始化EPG50夹爪, 端口: /dev/ttyUSB0, 默认从站ID: 0x09
[INFO] [epg50_gripper]: EPG50夹爪节点已启动
发送命令: 9 3 7 d0 0 4 45 cc
等待响应...
响应超时
[WARN] [epg50_gripper]: 获取夹爪状态失败
```

此时 `ros2 service call` 调用 enable / set_parameters / status 全部返回 `success=False`。

**原因：** 串口设备文件能正常 `open()`，所以节点初始化成功。但 Modbus RTU 命令发出后夹爪硬件无任何回复，说明物理通信链路不通。

**排查顺序（按可能性从高到低）：**

1. **夹爪没供电** — RS-485 转 USB 模块由 USB 口供电，但夹爪本体需要独立 24V 电源。检查夹爪电源指示灯是否亮起。
2. **从站 ID 不匹配** — 默认从站 ID 为 `0x09`，如果夹爪此前被改过 ID（如通过 `/epg50_gripper/rename` 服务），发送给 `0x09` 的命令不会被响应。检查夹爪本体上是否有 ID 拨码开关或标签。
3. **波特率不匹配** — 代码写死 `B115200`，如果夹爪固件使用其他波特率则无法通信。
4. **RS-485 接线问题** — A+/B- 线可能接反，或 GND 未共地。
5. **串口设备不对** — `/dev/ttyUSB0` 可能对应其他设备（如机械臂），而不是夹爪的 RS-485 转换器。插拔夹爪 USB 后用 `dmesg | tail` 或 `ls /dev/tty*` 确认实际设备名。
6. **串口权限不足** — 非 root 用户需加入 `dialout` 组：`sudo usermod -aG dialout $USER`，重新登录后生效。

### 测试命令

```bash
# 使能夹爪
ros2 service call /epg50_gripper/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 0, command: 1, position: 0, speed: 0, torque: 0}"

# 设置参数（位置、速度、力矩）
ros2 service call /epg50_gripper/command epg50_gripper_ros/srv/GripperCommand \
  "{slave_id: 0, command: 2, position: 128, speed: 160, torque: 100}"

# 查询状态
ros2 service call /epg50_gripper/status epg50_gripper_ros/srv/GripperStatus \
  "{slave_id: 0}"
```

### 启动命令

```bash
# 默认端口 /dev/ttyACM0
ros2 launch epg50_gripper_ros launch.py

# 指定端口
ros2 launch epg50_gripper_ros launch.py port:=/dev/ttyUSB0
```

## 注意事项

1. 使用前请确保串口路径正确且有访问权限
2. 不同型号的夹爪可能需要调整寄存器地址
3. 在开始其他操作前，请先调用`enable()`使能夹爪
4. 串口能打开 ≠ 通信正常，需观察 debug 输出中是否有"响应超时"

