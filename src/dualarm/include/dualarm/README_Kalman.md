# 自适应帧率卡尔曼滤波器 - XYZ位置估计

这是一个专门用于xyz位置估计的标准卡尔曼滤波器类，采用匀速运动模型，**支持变化的传入帧率**。

## 🆕 新特性：自适应帧率支持

- **自动时间间隔检测**: 滤波器能够自动计算连续测量之间的实际时间间隔
- **动态矩阵更新**: 状态转移矩阵F和过程噪声矩阵Q会根据实际时间间隔动态调整
- **灵活模式切换**: 支持自适应模式和固定帧率模式之间的切换

## 特性

- **输入**: `std::vector<double>` 格式的xyz位置观测值 `[x, y, z]`
- **输出**: `std::vector<double>` 格式的滤波后位置 `[x, y, z]`
- **状态向量**: 6维 `[x, y, z, vx, vy, vz]` (位置 + 速度)
- **观测向量**: 3维 `[x, y, z]` (仅位置)
- **运动模型**: 匀速运动模型
- **帧率适应**: 自动适应5ms-1000ms的时间间隔 (1-200Hz)

## 文件结构

```
include/dualarm/
├── kalman.h                    # 头文件
├── kalman.cpp                  # 实现文件
├── kalman_example.cpp          # 基本使用示例
├── kalman_usage_example.cpp    # ROS应用示例
├── kalman_adaptive_test.cpp    # 自适应帧率测试
├── kalman_real_test.cpp        # 真实场景测试
└── README_Kalman.md           # 详细文档
```

## 快速开始

### 1. 自适应帧率模式（推荐）

```cpp
#include "kalman.h"

// 创建滤波器实例
KalmanFilter filter;

// 设置参数
filter.setProcessNoise(0.01);      // 过程噪声
filter.setMeasurementNoise(0.1);   // 观测噪声
filter.enableAdaptiveTiming(true); // 启用自适应帧率

// 使用滤波器（无需手动设置时间间隔）
std::vector<double> measurement = {x, y, z};
std::vector<double> filtered = filter.update(measurement);

// 获取当前帧率信息
double current_fps = filter.getCurrentFrameRate();
double time_interval = filter.getCurrentDeltaTime();
```

### 2. 固定帧率模式

```cpp
KalmanFilter filter;
filter.setDeltaTime(0.033);        // 固定30Hz频率
filter.enableAdaptiveTiming(false); // 禁用自适应帧率
filter.setProcessNoise(0.01);
filter.setMeasurementNoise(0.1);

// 正常使用...
```

### 3. ROS机器人应用（自适应模式）

```cpp
class RobotKalmanFilter {
private:
    KalmanFilter tcp_pose_filter_;
    
public:
    RobotKalmanFilter() {
        tcp_pose_filter_.setProcessNoise(0.005);
        tcp_pose_filter_.setMeasurementNoise(0.1);
        tcp_pose_filter_.enableAdaptiveTiming(true); // 自适应帧率
    }
    
    std::vector<double> filterTCPPose(const std::vector<double>& raw_pose) {
        return tcp_pose_filter_.update(raw_pose); // 自动处理时间间隔
    }
    
    double getCurrentFPS() {
        return tcp_pose_filter_.getCurrentFrameRate();
    }
};
```

## API参考

### 构造函数
- `KalmanFilter()`: 使用默认参数初始化滤波器

### 主要方法
- `predict()`: 执行预测步骤，返回预测位置 `[x, y, z]`
- `update(measurement)`: 执行更新步骤，返回滤波位置 `[x, y, z]`
- `getPosition()`: 获取当前位置估计 `[x, y, z]`
- `getVelocity()`: 获取当前速度估计 `[vx, vy, vz]`
- `getState()`: 获取完整状态 `[x, y, z, vx, vy, vz]`

### 参数设置
- `setDeltaTime(dt)`: 设置固定时间间隔(秒)，同时禁用自适应时间
- `enableAdaptiveTiming(enable)`: 启用/禁用自适应时间间隔
- `setProcessNoise(noise)`: 设置过程噪声方差
- `setMeasurementNoise(noise)`: 设置观测噪声方差
- `reset()`: 重置滤波器状态

### 状态查询
- `getCurrentDeltaTime()`: 获取当前时间间隔(秒)
- `getCurrentFrameRate()`: 获取当前帧率(Hz)

## 编译和测试

### 直接编译
```bash
cd /path/to/include/dualarm/
g++ -I/usr/include/eigen3 -o kalman_test kalman.cpp kalman_example.cpp
./kalman_test

# 测试自适应帧率功能
g++ -I/usr/include/eigen3 -o adaptive_test kalman.cpp kalman_adaptive_test.cpp
./adaptive_test
```

### 在ROS包中使用
1. 确保CMakeLists.txt包含Eigen3依赖：
```cmake
find_package(Eigen3 REQUIRED)
add_library(kalman_filter include/dualarm/kalman.cpp)
target_link_libraries(kalman_filter Eigen3::Eigen)
```

2. 在你的节点中链接库：
```cmake
target_link_libraries(your_node kalman_filter)
```

## 自适应帧率算法说明

### 时间间隔计算
滤波器使用高精度时间戳自动计算连续测量之间的时间间隔：
- 最小时间间隔：5ms (200Hz最大帧率)
- 最大时间间隔：1000ms (1Hz最小帧率)
- 自动处理异常时间间隔

### 动态矩阵调整
1. **状态转移矩阵F**: 根据实际时间间隔更新位置-速度关系
2. **过程噪声矩阵Q**: 根据时间间隔缩放噪声，保持滤波性能一致

### 何时使用自适应模式
- ✅ **推荐使用**：传感器帧率不稳定或变化的场景
- ✅ **推荐使用**：多传感器融合，不同传感器有不同更新频率
- ✅ **推荐使用**：实时系统中CPU负载变化导致的处理延迟
- ❌ **不推荐**：已知固定帧率且对性能要求极高的场景

## 参数调优指南

### 自适应模式调优
由于滤波器会自动调整时间相关参数，主要需要调整：
### 过程噪声 (Process Noise)
- **小值 (0.001-0.01)**: 滤波器相信运动模型，输出更平滑但响应较慢
- **大值 (0.1-1.0)**: 滤波器允许更多变化，响应更快但可能有更多噪声
- **自适应模式建议**: 0.005-0.02 (系统会自动根据时间间隔缩放)

### 观测噪声 (Measurement Noise)
- **小值 (0.01-0.1)**: 滤波器更信任观测值
- **大值 (1.0-10.0)**: 滤波器更依赖预测值
- **自适应模式建议**: 根据传感器精度设置，不受帧率影响

### 时间间隔设置 (仅固定模式)
- 应该设置为实际的采样周期
- 例如：30Hz → dt=0.033, 50Hz → dt=0.02
- **自适应模式**: 无需手动设置，自动计算

## 应用场景

1. **机器人TCP位置滤波**: 减少位置传感器噪声，自适应控制系统的变化帧率
2. **视觉目标跟踪**: 平滑目标位置估计，处理视觉处理延迟的变化
3. **传感器数据融合**: 结合多个不同更新频率的位置传感器
4. **运动预测**: 基于历史数据预测未来位置，适应实际运动节奏
5. **实时系统**: 处理由于系统负载变化导致的不规律采样

## 依赖项

- Eigen3: 线性代数库
- C++17或更高版本
- chrono: 高精度时间测量（C++标准库）

## 注意事项

1. 首次调用`update()`会自动初始化滤波器状态和时间戳
2. 滤波器假设匀速运动模型，适用于连续运动场景
3. **自适应模式**：首次测量后才能计算时间间隔
4. **固定模式**：参数需要根据具体应用场景进行调优
5. 如果目标运动模式发生显著变化，建议调用`reset()`重新初始化
6. 自适应模式会有轻微的计算开销，但通常可以忽略
