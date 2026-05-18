#include "kalman.h"
#include <iostream>
#include <vector>

/**
 * @brief 如何在主程序中使用卡尔曼滤波器的示例
 */
void useKalmanFilterInMainProgram() {
    // 1. 创建卡尔曼滤波器实例
    KalmanFilter position_filter;

    // 2. 设置滤波器参数
    position_filter.setDeltaTime(0.033);      // 30Hz更新频率
    position_filter.setProcessNoise(0.01);    // 较小的过程噪声
    position_filter.setMeasurementNoise(0.5); // 观测噪声

    // 3. 模拟从传感器或视觉系统获取的xyz位置数据
    std::vector<double> sensor_measurement = { 10.5, 15.2, 8.7 }; // [x, y, z]

    // 4. 使用滤波器进行预测和更新
    std::vector<double> predicted_pos = position_filter.predict();
    std::vector<double> filtered_pos  = position_filter.update(sensor_measurement);

    // 5. 获取滤波后的结果
    std::vector<double> current_position = position_filter.getPosition();
    std::vector<double> current_velocity = position_filter.getVelocity();

    std::cout << "传感器观测: [" << sensor_measurement[0] << ", " << sensor_measurement[1] << ", "
              << sensor_measurement[2] << "]" << std::endl;
    std::cout << "滤波位置: [" << current_position[0] << ", " << current_position[1] << ", " << current_position[2]
              << "]" << std::endl;
    std::cout << "估计速度: [" << current_velocity[0] << ", " << current_velocity[1] << ", " << current_velocity[2]
              << "]" << std::endl;
}

/**
 * @brief 如何在ROS中使用卡尔曼滤波器的示例
 */
class RobotKalmanFilter {
private:
    KalmanFilter tcp_pose_filter_;

public:
    RobotKalmanFilter() {
        // 初始化滤波器参数
        tcp_pose_filter_.setDeltaTime(0.02);       // 50Hz
        tcp_pose_filter_.setProcessNoise(0.005);   // 低过程噪声
        tcp_pose_filter_.setMeasurementNoise(0.1); // 观测噪声
    }

    // 处理来自机器人的TCP位置数据
    std::vector<double> filterTCPPose(const std::vector<double>& raw_tcp_pose) {
        // 预测步骤
        tcp_pose_filter_.predict();

        // 使用原始TCP位置更新滤波器
        std::vector<double> filtered_pose = tcp_pose_filter_.update(raw_tcp_pose);

        return filtered_pose;
    }

    // 获取当前TCP位置和速度
    std::vector<double> getCurrentTCPPosition() {
        return tcp_pose_filter_.getPosition();
    }

    std::vector<double> getCurrentTCPVelocity() {
        return tcp_pose_filter_.getVelocity();
    }

    // 重置滤波器
    void resetFilter() {
        tcp_pose_filter_.reset();
    }
};

int main() {
    std::cout << "=== 基本使用示例 ===" << std::endl;
    useKalmanFilterInMainProgram();

    std::cout << "\n=== ROS机器人应用示例 ===" << std::endl;
    RobotKalmanFilter robot_filter;

    // 模拟机器人TCP位置数据序列
    std::vector<std::vector<double>> tcp_positions = { { 100.0, 200.0, 300.0 },
                                                       { 100.5, 200.3, 300.1 },
                                                       { 101.2, 200.8, 300.4 },
                                                       { 101.8, 201.2, 300.6 } };

    for (size_t i = 0; i < tcp_positions.size(); i++) {
        std::vector<double> filtered = robot_filter.filterTCPPose(tcp_positions[i]);
        std::vector<double> velocity = robot_filter.getCurrentTCPVelocity();

        std::cout << "原始TCP位置: [" << tcp_positions[i][0] << ", " << tcp_positions[i][1] << ", "
                  << tcp_positions[i][2] << "]" << std::endl;
        std::cout << "滤波TCP位置: [" << filtered[0] << ", " << filtered[1] << ", " << filtered[2] << "]" << std::endl;
        std::cout << "TCP速度: [" << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << "]" << std::endl;
        std::cout << "---" << std::endl;
    }

    return 0;
}
