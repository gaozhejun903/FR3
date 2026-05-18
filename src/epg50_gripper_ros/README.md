# EPG50_Serial 夹爪串行通信库

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

## 注意事项

1. 使用前请确保串口路径正确且有访问权限
2. 不同型号的夹爪可能需要调整寄存器地址
3. 在开始其他操作前，请先调用`enable()`使能夹爪

