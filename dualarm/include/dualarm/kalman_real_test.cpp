#include "kalman.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

int main() {
    std::cout << "=== 真实帧率变化卡尔曼滤波器测试 ===" << std::endl;

    // 创建两个滤波器进行对比
    KalmanFilter adaptive_filter;
    KalmanFilter fixed_filter;

    // 设置参数
    adaptive_filter.setProcessNoise(0.01);
    adaptive_filter.setMeasurementNoise(0.1);
    adaptive_filter.enableAdaptiveTiming(true); // 启用自适应时间

    fixed_filter.setProcessNoise(0.01);
    fixed_filter.setMeasurementNoise(0.1);
    fixed_filter.setDeltaTime(0.033); // 固定30Hz

    // 模拟真实的传感器数据（物体匀速运动）
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 1.0, vy = 2.0, vz = 0.5; // 真实速度

    // 模拟不同的采样间隔
    std::vector<int> sampling_intervals = { 33, 50, 20, 100, 25, 40, 15, 60 }; // 毫秒

    std::cout << "时间(ms)\t真实位置\t\t自适应滤波\t\t固定帧率滤波\t自适应帧率\n";
    std::cout << "--------\t--------\t\t----------\t\t------------\t--------\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < sampling_intervals.size(); i++) {
        // 等待指定的时间间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(sampling_intervals[i]));

        // 计算真实位置（基于实际时间和真实速度）
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed      = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        double real_dt    = elapsed.count() / 1000.0;

        double real_x = x + vx * real_dt;
        double real_y = y + vy * real_dt;
        double real_z = z + vz * real_dt;

        // 模拟传感器噪声
        std::vector<double> noisy_measurement = { real_x + (rand() % 20 - 10) * 0.01, // ±0.1的噪声
                                                  real_y + (rand() % 20 - 10) * 0.01,
                                                  real_z + (rand() % 20 - 10) * 0.01 };

        // 使用两个滤波器处理
        std::vector<double> adaptive_result = adaptive_filter.update(noisy_measurement);
        std::vector<double> fixed_result    = fixed_filter.update(noisy_measurement);

        double adaptive_fps = adaptive_filter.getCurrentFrameRate();

        std::printf(
            "%7.0f\t\t[%.2f,%.2f,%.2f]\t[%.2f,%.2f,%.2f]\t\t[%.2f,%.2f,%.2f]\t\t%.1f Hz\n",
            elapsed.count() / 1.0,
            real_x,
            real_y,
            real_z,
            adaptive_result[0],
            adaptive_result[1],
            adaptive_result[2],
            fixed_result[0],
            fixed_result[1],
            fixed_result[2],
            adaptive_fps
        );
    }

    std::cout << "\n=== 速度估计对比 ===" << std::endl;
    std::vector<double> adaptive_vel = adaptive_filter.getVelocity();
    std::vector<double> fixed_vel    = fixed_filter.getVelocity();

    std::cout << "真实速度: [" << vx << ", " << vy << ", " << vz << "]" << std::endl;
    std::cout << "自适应滤波器估计速度: [" << adaptive_vel[0] << ", " << adaptive_vel[1] << ", " << adaptive_vel[2]
              << "]" << std::endl;
    std::cout << "固定帧率滤波器估计速度: [" << fixed_vel[0] << ", " << fixed_vel[1] << ", " << fixed_vel[2] << "]"
              << std::endl;

    // 计算误差
    double adaptive_vel_error =
        sqrt(pow(adaptive_vel[0] - vx, 2) + pow(adaptive_vel[1] - vy, 2) + pow(adaptive_vel[2] - vz, 2));
    double fixed_vel_error = sqrt(pow(fixed_vel[0] - vx, 2) + pow(fixed_vel[1] - vy, 2) + pow(fixed_vel[2] - vz, 2));

    std::cout << "\n自适应滤波器速度误差: " << adaptive_vel_error << std::endl;
    std::cout << "固定帧率滤波器速度误差: " << fixed_vel_error << std::endl;

    if (adaptive_vel_error < fixed_vel_error) {
        std::cout << "自适应滤波器速度估计更准确！" << std::endl;
    } else {
        std::cout << "固定帧率滤波器速度估计更准确！" << std::endl;
    }

    return 0;
}
