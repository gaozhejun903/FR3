#!/bin/bash

# ServiceServerTemplate 重构演示脚本
# 这个脚本演示了如何使用重构后的机器人控制代码

echo "=========================================="
echo "ServiceServerTemplate 重构演示"
echo "=========================================="

# 设置ROS2环境
source /opt/ros/humble/setup.bash
cd /home/phoenix/roboarm/FairinoDualArm
source install/setup.bash

echo ""
echo "1. 编译重构后的代码..."
colcon build --packages-select dualarm --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ $? -ne 0 ]; then
    echo "编译失败！请检查代码错误。"
    exit 1
fi

echo "编译成功！"
echo ""

echo "2. 可用的程序："
echo "   - robot_main: 原始版本（部分简化）"
echo "   - robot_main_refactored: 完全重构版本"
echo ""

echo "3. 启动重构后的机器人主控程序..."
echo "注意：这个程序需要其他ROS2节点（如机器人控制器、夹爪控制器）正在运行"
echo ""

# 检查必要的服务是否可用
echo "检查必要的服务是否可用..."

# 函数：检查服务是否存在
check_service() {
    local service_name=$1
    local timeout=3
    
    echo -n "检查服务 $service_name ... "
    
    if timeout $timeout ros2 service list | grep -q "$service_name"; then
        echo "✓ 可用"
        return 0
    else
        echo "✗ 不可用"
        return 1
    fi
}

# 检查关键服务
services_ok=true

check_service "/L/robot_move" || services_ok=false
check_service "/L/robot_act" || services_ok=false
check_service "/L/robot_move_cart" || services_ok=false
check_service "/epg50_gripper/command" || services_ok=false

echo ""

if [ "$services_ok" = false ]; then
    echo "⚠️  警告：某些必要的服务不可用。"
    echo "请确保以下节点正在运行："
    echo "  - 机器人控制器节点"
    echo "  - 夹爪控制器节点"
    echo ""
    echo "您仍然可以运行程序查看服务等待逻辑。"
    echo ""
fi

echo "选择要运行的程序："
echo "1) robot_main (原始版本，部分简化)"
echo "2) robot_main_refactored (完全重构版本)"
echo "3) 显示ServiceServerTemplate使用示例"
echo "4) 退出"
echo ""

read -p "请选择 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "启动原始版本（部分简化）..."
        echo "按 Ctrl+C 停止程序"
        ros2 run dualarm robot_main
        ;;
    2)
        echo ""
        echo "启动完全重构版本..."
        echo "按 Ctrl+C 停止程序"
        ros2 run dualarm robot_main_refactored
        ;;
    3)
        echo ""
        echo "=========================================="
        echo "ServiceServerTemplate 使用示例"
        echo "=========================================="
        echo ""
        echo "1. 创建服务服务器："
        echo ""
        cat << 'EOF'
#include "robo_ctrl/include/robo_ctrl/service_server_template.hpp"

// 创建服务服务器
auto move_server = std::make_shared<ServiceServerTemplate<robo_ctrl::srv::RobotMove>>(
    "robot_move_server",           // 节点名称
    "/robot_move",                 // 服务名称
    [](const auto& request, auto& response) {  // 回调函数
        // 处理机器人移动请求
        RCLCPP_INFO(rclcpp::get_logger("move_server"), 
                   "Processing move request");
        
        // 执行移动逻辑...
        
        response->success = true;
        response->message = "Move completed successfully";
    },
    rmw_qos_profile_services_default,  // QoS配置
    true                               // 启用统计信息
);
EOF
        echo ""
        echo "2. 使用ServiceCaller进行客户端调用："
        echo ""
        cat << 'EOF'
// 创建请求
auto request = std::make_shared<robo_ctrl::srv::RobotMove::Request>();
request->move_type = 1;  // 笛卡尔移动
// ... 设置其他参数

// 调用服务（同步）
auto response = ServiceCaller<robo_ctrl::srv::RobotMove>::callServiceSync(
    client,                        // 服务客户端
    request,                       // 请求
    node,                          // ROS2节点
    std::chrono::seconds(10),      // 超时时间
    "robot_move_service"           // 服务名称（用于日志）
);

// 检查结果
if (response && response->success) {
    RCLCPP_INFO(node->get_logger(), "Service call succeeded: %s", 
               response->message.c_str());
} else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
}
EOF
        echo ""
        echo "3. 等待服务可用："
        echo ""
        cat << 'EOF'
// 等待服务可用
if (!waitForServiceEnhanced(client, node, "robot_move", std::chrono::seconds(30))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available");
    return false;
}
EOF
        echo ""
        echo "详细信息请查看: README_ServiceTemplate_Refactoring.md"
        ;;
    4)
        echo "退出演示"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo ""
echo "演示结束"
