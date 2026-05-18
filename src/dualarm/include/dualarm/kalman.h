#ifndef KALMAN_H
#define KALMAN_H

#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>

/**
 * @brief 标准卡尔曼滤波器类，用于xyz位置估计，支持变化的传入帧率
 *
 * 状态向量: [x, y, z, vx, vy, vz] (6维)
 * 观测向量: [x, y, z] (3维)
 * 使用匀速运动模型，能够自适应变化的时间间隔
 */
class KalmanFilter {
public:
    /**
     * @brief 构造函数，初始化卡尔曼滤波器参数
     */
    KalmanFilter();

    /**
     * @brief 析构函数
     */
    ~KalmanFilter();

    /**
     * @brief 预测步骤
     * @return 预测的位置向量 [x, y, z]
     */
    std::vector<double> predict();

    /**
     * @brief 更新步骤
     * @param measurement 观测值向量 [x, y, z]
     * @return 更新后的位置向量 [x, y, z]
     */
    std::vector<double> update(const std::vector<double>& measurement);

    /**
     * @brief 获取当前位置估计
     * @return 当前位置向量 [x, y, z]
     */
    std::vector<double> getPosition() const;

    /**
     * @brief 获取当前速度估计
     * @return 当前速度向量 [vx, vy, vz]
     */
    std::vector<double> getVelocity() const;

    /**
     * @brief 获取完整状态向量
     * @return 状态向量 [x, y, z, vx, vy, vz]
     */
    std::vector<double> getState() const;

    /**
     * @brief 设置过程噪声方差
     * @param noise 过程噪声方差值
     */
    void setProcessNoise(double noise);

    /**
     * @brief 设置观测噪声方差
     * @param noise 观测噪声方差值
     */
    void setMeasurementNoise(double noise);

    /**
     * @brief 设置时间间隔（禁用自适应时间）
     * @param dt 时间间隔(秒)
     */
    void setDeltaTime(double dt);

    /**
     * @brief 启用或禁用自适应时间间隔
     * @param enable true=启用自适应时间，false=使用固定时间间隔
     */
    void enableAdaptiveTiming(bool enable);

    /**
     * @brief 获取当前时间间隔
     * @return 当前时间间隔(秒)
     */
    double getCurrentDeltaTime() const;

    /**
     * @brief 获取当前帧率
     * @return 当前帧率(Hz)
     */
    double getCurrentFrameRate() const;

    /**
     * @brief 重置滤波器状态
     */
    void reset();

private:
    // 状态向量 [x, y, z, vx, vy, vz]
    Eigen::VectorXd x;

    // 状态协方差矩阵
    Eigen::MatrixXd P;

    // 状态转移矩阵
    Eigen::MatrixXd F;

    // 观测矩阵
    Eigen::MatrixXd H;

    // 过程噪声协方差矩阵（动态计算）
    Eigen::MatrixXd Q;

    // 过程噪声基础矩阵（用于计算实际的Q）
    Eigen::MatrixXd Q_base;

    // 观测噪声协方差矩阵
    Eigen::MatrixXd R;

    // 单位矩阵
    Eigen::MatrixXd I;

    // 是否已初始化
    bool initialized;

    // 时间管理相关
    double current_dt_;                                             // 当前时间间隔
    std::chrono::high_resolution_clock::time_point last_timestamp_; // 上次更新的时间戳
    bool use_adaptive_timing_;                                      // 是否使用自适应时间间隔

    /**
     * @brief 更新状态转移矩阵F
     */
    void updateStateTransitionMatrix();

    /**
     * @brief 根据当前时间间隔更新过程噪声矩阵Q
     */
    void updateProcessNoiseMatrix();

    /**
     * @brief 计算自适应时间间隔
     * @return 计算得到的时间间隔(秒)
     */
    double calculateDeltaTime();
};

#endif // KALMAN_H
