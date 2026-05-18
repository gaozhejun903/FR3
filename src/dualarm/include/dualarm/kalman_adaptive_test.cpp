#include "kalman.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <random>

int main() {
    std::cout << "=== 自适应帧率卡尔曼滤波器测试 ===" << std::endl;
    std::cout << "========================================" << std::endl;

    // 创建卡尔曼滤波器实例
    KalmanFilter adaptive_filter;
    KalmanFilter fixed_filter;

    // 设置参数
    adaptive_filter.setProcessNoise(0.01);
    adaptive_filter.setMeasurementNoise(0.1);
    adaptive_filter.enableAdaptiveTiming(true); // 启用自适应时间

    fixed_filter.setProcessNoise(0.01);
    fixed_filter.setMeasurementNoise(0.1);
    fixed_filter.setDeltaTime(0.033); // 固定30Hz

    // 模拟变化的帧率数据
    std::vector<std::vector<double>> measurements = { { 1.0, 2.0, 3.0 },
                                                      { 1.1, 2.1, 3.1 },
                                                      { 1.2, 2.2, 3.2 },
                                                      { 1.3, 2.3, 3.3 },
                                                      { 1.4, 2.4, 3.4 } };

    // 模拟不同的时间间隔
    std::vector<int> delays_ms = { 33, 50, 20, 100, 25 }; // 变化的延迟

    std::cout << "测试不同帧率下的滤波效果:" << std::endl;
    std::cout << "----------------------------" << std::endl;

    for (size_t i = 0; i < measurements.size(); i++) {
        // 模拟变化的时间间隔
        if (i > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delays_ms[i - 1]));
        }

        // 自适应滤波器处理
        std::vector<double> adaptive_pred     = adaptive_filter.predict();
        std::vector<double> adaptive_updated  = adaptive_filter.update(measurements[i]);
        std::vector<double> adaptive_velocity = adaptive_filter.getVelocity();

        // 固定帧率滤波器处理
        std::vector<double> fixed_pred     = fixed_filter.predict();
        std::vector<double> fixed_updated  = fixed_filter.update(measurements[i]);
        std::vector<double> fixed_velocity = fixed_filter.getVelocity();

        std::cout << "第 " << i + 1 << " 次测量:" << std::endl;
        std::cout << "  观测值: [" << measurements[i][0] << ", " << measurements[i][1] << ", " << measurements[i][2]
                  << "]" << std::endl;

        if (i > 0) {
            double actual_fps = 1000.0 / delays_ms[i - 1];
            std::cout << "  实际帧率: " << actual_fps << " Hz" << std::endl;
            std::cout << "  自适应滤波器检测帧率: " << adaptive_filter.getCurrentFrameRate() << " Hz" << std::endl;
            std::cout << "  时间间隔: " << adaptive_filter.getCurrentDeltaTime() << " s" << std::endl;
        }

        std::cout << "  自适应滤波位置: [" << adaptive_updated[0] << ", " << adaptive_updated[1] << ", "
                  << adaptive_updated[2] << "]" << std::endl;
        std::cout << "  固定帧率滤波位置: [" << fixed_updated[0] << ", " << fixed_updated[1] << ", " << fixed_updated[2]
                  << "]" << std::endl;
        std::cout << "  自适应速度估计: [" << adaptive_velocity[0] << ", " << adaptive_velocity[1] << ", "
                  << adaptive_velocity[2] << "]" << std::endl;
        std::cout << "  固定速度估计: [" << fixed_velocity[0] << ", " << fixed_velocity[1] << ", " << fixed_velocity[2]
                  << "]" << std::endl;
        std::cout << std::endl;
    }

    std::cout << "=== 帧率统计测试 ===" << std::endl;
    std::cout << "测试100次随机时间间隔的处理" << std::endl;

    KalmanFilter stats_filter;
    stats_filter.enableAdaptiveTiming(true);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(10, 100); // 10-100ms随机延迟

    double total_fps       = 0.0;
    int valid_measurements = 0;

    for (int i = 0; i < 100; i++) {
        if (i > 0) {
            int delay = dis(gen);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }

        // 生成模拟测量值
        std::vector<double> test_measurement = { 10.0 + i * 0.1 + (dis(gen) - 50) * 0.01, // 添加一些噪声
                                                 20.0 + i * 0.2 + (dis(gen) - 50) * 0.01,
                                                 30.0 + i * 0.1 + (dis(gen) - 50) * 0.01 };

        stats_filter.update(test_measurement);

        if (i > 0) { // 跳过第一次（没有时间间隔）
            double fps = stats_filter.getCurrentFrameRate();
            if (fps >= 1.0 && fps <= 1000.0) { // 合理范围内的帧率
                total_fps += fps;
                valid_measurements++;
            }
        }
    }

    if (valid_measurements > 0) {
        double avg_fps = total_fps / valid_measurements;
        std::cout << "平均帧率: " << avg_fps << " Hz" << std::endl;
        std::cout << "有效测量次数: " << valid_measurements << "/99" << std::endl;
    }

    return 0;
}
