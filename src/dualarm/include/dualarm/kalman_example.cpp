#include "kalman.h"
#include <iostream>
#include <vector>

int main() {
    // 创建卡尔曼滤波器实例
    KalmanFilter kf;

    // 设置参数
    kf.setDeltaTime(0.033);      // 30Hz频率
    kf.setProcessNoise(0.1);     // 过程噪声
    kf.setMeasurementNoise(1.0); // 观测噪声

    // 模拟观测数据
    std::vector<std::vector<double>> measurements = { { 1.0, 2.0, 3.0 },
                                                      { 1.1, 2.1, 3.1 },
                                                      { 1.2, 2.2, 3.2 },
                                                      { 1.3, 2.3, 3.3 },
                                                      { 1.4, 2.4, 3.4 } };

    std::cout << "卡尔曼滤波器xyz位置估计示例:" << std::endl;
    std::cout << "================================" << std::endl;

    for (size_t i = 0; i < measurements.size(); i++) {
        // 预测步骤
        std::vector<double> predicted = kf.predict();

        // 更新步骤
        std::vector<double> updated = kf.update(measurements[i]);

        // 获取速度估计
        std::vector<double> velocity = kf.getVelocity();

        std::cout << "第 " << i + 1 << " 次迭代:" << std::endl;
        std::cout << "  观测值: [" << measurements[i][0] << ", " << measurements[i][1] << ", " << measurements[i][2]
                  << "]" << std::endl;
        std::cout << "  预测位置: [" << predicted[0] << ", " << predicted[1] << ", " << predicted[2] << "]"
                  << std::endl;
        std::cout << "  更新位置: [" << updated[0] << ", " << updated[1] << ", " << updated[2] << "]" << std::endl;
        std::cout << "  估计速度: [" << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << "]" << std::endl;
        std::cout << std::endl;
    }

    return 0;
}
