# 使用ServiceServerTemplate简化main.cpp重构说明

本文档说明了如何使用 `ServiceServerTemplate` 模板类来简化 `main.cpp` 中的服务调用逻辑。

## 重构概览

### 1. 原始问题
原始的 `main.cpp` 文件存在以下问题：
- 大量重复的服务调用代码
- 错误处理逻辑分散
- 缺乏统一的服务管理
- 代码冗长，难以维护

### 2. 重构方案

#### A. 创建了ServiceCaller模板类
```cpp
template<typename ServiceType>
class ServiceCaller {
    static std::shared_ptr<Response> callServiceSync(
        std::shared_ptr<Client> client,
        std::shared_ptr<Request> request,
        rclcpp::Node::SharedPtr node,
        std::chrono::seconds timeout = std::chrono::seconds(5),
        const std::string& service_name = "unknown");
};
```

**优点：**
- 统一的错误处理
- 自动的日志记录
- 超时控制
- 类型安全

#### B. 增强的等待服务函数
```cpp
template<typename ClientType>
bool waitForServiceEnhanced(std::shared_ptr<ClientType> client, 
                           rclcpp::Node::SharedPtr node,
                           const std::string& service_name,
                           std::chrono::seconds timeout = std::chrono::seconds(30));
```

**优点：**
- 统一的等待逻辑
- 更好的日志输出
- 超时控制
- 错误处理

#### C. 完整的ServiceManager和ServiceClientManager类
在 `main_refactored.cpp` 中提供了完整的重构示例，包括：
- `ServiceManager`: 管理服务服务器
- `ServiceClientManager`: 管理服务客户端
- `RobotMainRefactored`: 重构后的主控类

## 重构前后对比

### 重构前（原始代码）
```cpp
// 等待服务
while (!node->L_robot_move_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for robot_move service");
        return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for robot_move service to appear...");
}

// 调用服务
auto gripper_future = node->gripper_command_client_->async_send_request(gripper_request);
if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper enable");
    return 1;
}
auto gripper_response = gripper_future.get();
if (!gripper_response->success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to enable gripper %d", gripper_id);
    return 1;
}
```

### 重构后（简化代码）
```cpp
// 等待服务
if (!waitForServiceEnhanced(node->L_robot_move_client_, node, "L/robot_move")) {
    return 1;
}

// 调用服务
auto gripper_response = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
    node->gripper_command_client_, gripper_request, node, 
    std::chrono::seconds(5), "gripper_enable");
    
if (!gripper_response->success) {
    return 1; // 错误已经在ServiceCaller中记录
}
```

## 使用方法

### 1. 编译重构后的代码
```bash
cd /home/phoenix/roboarm/FairinoDualArm
colcon build --packages-select dualarm
```

### 2. 运行不同版本
```bash
# 运行原始版本（部分简化）
ros2 run dualarm robot_main

# 运行完全重构版本
ros2 run dualarm robot_main_refactored
```

### 3. 在新项目中使用ServiceServerTemplate
```cpp
#include "robo_ctrl/include/robo_ctrl/service_server_template.hpp"

// 创建服务服务器
auto move_server = std::make_shared<ServiceServerTemplate<robo_ctrl::srv::RobotMove>>(
    "robot_move_server",
    "/robot_move",
    [](const auto& request, auto& response) {
        // 处理逻辑
        response->success = true;
        response->message = "Move completed";
    },
    rmw_qos_profile_services_default,
    true  // 启用统计
);

// 使用ServiceCaller进行客户端调用
auto response = ServiceCaller<robo_ctrl::srv::RobotMove>::callServiceSync(
    client, request, node, std::chrono::seconds(10), "robot_move");
```

## 重构收益

### 1. 代码简化
- 服务调用从 ~15 行减少到 ~5 行
- 错误处理统一化
- 日志记录自动化

### 2. 可维护性提升
- 模板化的设计易于扩展
- 统一的接口规范
- 更好的错误跟踪

### 3. 功能增强
- 自动统计功能
- 超时控制
- 服务状态监控

### 4. 类型安全
- 编译时类型检查
- 自动推导服务类型
- 减少运行时错误

## 注意事项

1. **头文件路径**: 确保 `service_server_template.hpp` 的包含路径正确
2. **编译依赖**: 需要包含 `robo_ctrl` 包
3. **命名空间**: 注意服务名称的命名规范
4. **错误处理**: ServiceCaller已包含基本错误处理，但可根据需要自定义

## 未来改进

1. **配置文件支持**: 将服务名称和参数配置化
2. **异步支持**: 添加异步服务调用支持
3. **重试机制**: 添加自动重试功能
4. **性能监控**: 增强统计和监控功能
5. **服务发现**: 自动发现可用服务

## 总结

通过使用 `ServiceServerTemplate` 和相关辅助类，我们成功简化了 `main.cpp` 中的服务调用逻辑，提高了代码的可维护性和可读性。这种模板化的方法可以应用到其他需要大量服务调用的ROS2项目中。
