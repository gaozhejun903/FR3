#include "dualarm/kalman.h"
#include <iostream>
#include <algorithm>

KalmanFilter::KalmanFilter() {
    // 状态向量维度为6: [x, y, z, vx, vy, vz]
    // 观测向量维度为3: [x, y, z]
    int state_dim = 6;
    int obs_dim   = 3;

    // 初始化状态向量 [x, y, z, vx, vy, vz]
    x = Eigen::VectorXd::Zero(state_dim);

    // 初始化状态协方差矩阵P - 降低初始不确定性
    P = Eigen::MatrixXd::Identity(state_dim, state_dim);
    // 位置的初始不确定性
    P(0, 0) = P(1, 1) = P(2, 2) = 100.0;
    // 速度的初始不确定性
    P(3, 3) = P(4, 4) = P(5, 5) = 10.0;

    // 状态转移矩阵F (匀速模型) - 初始化为单位矩阵
    F = Eigen::MatrixXd::Identity(state_dim, state_dim);
    // dt将在运行时动态计算，这里先设置为0
    current_dt_ = 0.033;       // 默认30Hz作为初始值
    F(0, 3)     = current_dt_; // x = x + vx*dt
    F(1, 4)     = current_dt_; // y = y + vy*dt
    F(2, 5)     = current_dt_; // z = z + vz*dt

    // 观测矩阵H (观测位置)
    H       = Eigen::MatrixXd::Zero(obs_dim, state_dim);
    H(0, 0) = 1.0; // 观测x
    H(1, 1) = 1.0; // 观测y
    H(2, 2) = 1.0; // 观测z

    // 过程噪声协方差矩阵Q - 基础噪声值
    Q_base                    = Eigen::MatrixXd::Zero(state_dim, state_dim);
    double process_noise_base = 0.01; // 降低过程噪声
    // 位置过程噪声基础值
    Q_base(0, 0) = process_noise_base;
    Q_base(1, 1) = process_noise_base;
    Q_base(2, 2) = process_noise_base;
    // 速度过程噪声基础值
    Q_base(3, 3) = process_noise_base * 1.0; // 减少速度过程噪声
    Q_base(4, 4) = process_noise_base * 1.0;
    Q_base(5, 5) = process_noise_base * 1.0;

    // 初始化实际使用的Q矩阵
    Q = Eigen::MatrixXd::Zero(state_dim, state_dim);
    updateProcessNoiseMatrix();

    // 观测噪声协方差矩阵R
    R = Eigen::MatrixXd::Identity(obs_dim, obs_dim) * 1.0;

    // 单位矩阵
    I = Eigen::MatrixXd::Identity(state_dim, state_dim);

    // 时间戳管理
    last_timestamp_      = std::chrono::high_resolution_clock::now();
    use_adaptive_timing_ = true; // 默认启用自适应时间间隔

    initialized = false;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::setProcessNoise(double noise) {
    // 更新基础噪声值
    Q_base(0, 0) = noise;
    Q_base(1, 1) = noise;
    Q_base(2, 2) = noise;
    Q_base(3, 3) = noise * 1.0; // 速度噪声系数调整
    Q_base(4, 4) = noise * 1.0;
    Q_base(5, 5) = noise * 1.0;

    // 重新计算实际的Q矩阵
    updateProcessNoiseMatrix();
}

void KalmanFilter::setMeasurementNoise(double noise) {
    R = Eigen::MatrixXd::Identity(3, 3) * noise;
}

void KalmanFilter::setDeltaTime(double dt) {
    current_dt_ = dt;
    updateStateTransitionMatrix();
    updateProcessNoiseMatrix();
    use_adaptive_timing_ = false; // 手动设置dt时禁用自适应时间
}

void KalmanFilter::enableAdaptiveTiming(bool enable) {
    use_adaptive_timing_ = enable;
    if (enable) {
        last_timestamp_ = std::chrono::high_resolution_clock::now();
    }
}

void KalmanFilter::updateStateTransitionMatrix() {
    F(0, 3) = current_dt_;
    F(1, 4) = current_dt_;
    F(2, 5) = current_dt_;
}

void KalmanFilter::updateProcessNoiseMatrix() {
    // 过程噪声矩阵需要根据时间间隔进行缩放
    // 对于连续白噪声模型，Q应该与dt成正比
    double dt_factor = current_dt_;

    // 位置噪声：Q_pos = Q_base * dt
    Q(0, 0) = Q_base(0, 0) * dt_factor;
    Q(1, 1) = Q_base(1, 1) * dt_factor;
    Q(2, 2) = Q_base(2, 2) * dt_factor;

    // 速度噪声：Q_vel = Q_base * dt
    Q(3, 3) = Q_base(3, 3) * dt_factor;
    Q(4, 4) = Q_base(4, 4) * dt_factor;
    Q(5, 5) = Q_base(5, 5) * dt_factor;

    // 添加位置-速度交叉项（可选，用于更精确的建模）
    double cross_term = Q_base(0, 0) * dt_factor * dt_factor * 0.5;
    Q(0, 3)           = cross_term;
    Q(3, 0)           = cross_term;
    Q(1, 4)           = cross_term;
    Q(4, 1)           = cross_term;
    Q(2, 5)           = cross_term;
    Q(5, 2)           = cross_term;
}

double KalmanFilter::calculateDeltaTime() {
    if (!use_adaptive_timing_) {
        return current_dt_;
    }

    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration     = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_timestamp_);
    double dt         = duration.count() / 1000000.0; // 转换为秒

    // 设置合理的dt范围，避免异常值
    const double min_dt = 0.005; // 最小5ms (200Hz max)
    const double max_dt = 1.0;   // 最大1s (1Hz min)
    dt                  = std::max(min_dt, std::min(max_dt, dt));

    last_timestamp_ = current_time;
    return dt;
}

std::vector<double> KalmanFilter::predict() {
    // 如果启用自适应时间，计算实际的时间间隔
    if (use_adaptive_timing_) {
        current_dt_ = calculateDeltaTime();
        updateStateTransitionMatrix();
        updateProcessNoiseMatrix();
    }

    // 预测步骤
    // x_k|k-1 = F * x_k-1|k-1
    x = F * x;

    // P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    P = F * P * F.transpose() + Q;

    // 返回预测的位置 [x, y, z]
    std::vector<double> predicted_position = { x(0), x(1), x(2) };
    return predicted_position;
}

std::vector<double> KalmanFilter::update(const std::vector<double>& measurement) {
    if (measurement.size() != 3) {
        std::cerr << "Error: Measurement vector must have 3 elements (x, y, z)" << std::endl;
        return { 0.0, 0.0, 0.0 };
    }

    // 转换输入向量为Eigen向量
    Eigen::VectorXd z(3);
    z << measurement[0], measurement[1], measurement[2];

    // 如果第一次初始化
    if (!initialized) {
        x(0)        = z(0); // x
        x(1)        = z(1); // y
        x(2)        = z(2); // z
        x(3)        = 0.0;  // vx
        x(4)        = 0.0;  // vy
        x(5)        = 0.0;  // vz
        initialized = true;

        // 初始化时间戳
        if (use_adaptive_timing_) {
            last_timestamp_ = std::chrono::high_resolution_clock::now();
        }

        return { x(0), x(1), x(2) };
    }

    // 如果启用自适应时间，计算实际的时间间隔
    if (use_adaptive_timing_) {
        current_dt_ = calculateDeltaTime();
        updateStateTransitionMatrix();
        updateProcessNoiseMatrix();
    }

    // 更新步骤
    // y = z - H * x_k|k-1 (创新或残差)
    Eigen::VectorXd y = z - H * x;

    // S = H * P_k|k-1 * H^T + R (创新协方差)
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // K = P_k|k-1 * H^T * S^-1 (卡尔曼增益)
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // x_k|k = x_k|k-1 + K * y (更新状态估计)
    x = x + K * y;

    // P_k|k = (I - K * H) * P_k|k-1 (更新协方差矩阵)
    P = (I - K * H) * P;

    // 返回更新后的位置 [x, y, z]
    std::vector<double> updated_position = { x(0), x(1), x(2) };
    return updated_position;
}

std::vector<double> KalmanFilter::getPosition() const {
    return { x(0), x(1), x(2) };
}

std::vector<double> KalmanFilter::getVelocity() const {
    return { x(3), x(4), x(5) };
}

double KalmanFilter::getCurrentDeltaTime() const {
    return current_dt_;
}

double KalmanFilter::getCurrentFrameRate() const {
    return 1.0 / current_dt_;
}

void KalmanFilter::reset() {
    x = Eigen::VectorXd::Zero(6);

    // 重新设置协方差矩阵
    P       = Eigen::MatrixXd::Identity(6, 6);
    P(0, 0) = P(1, 1) = P(2, 2) = 100.0; // 位置不确定性
    P(3, 3) = P(4, 4) = P(5, 5) = 10.0;  // 速度不确定性

    initialized = false;

    // 重置时间戳
    if (use_adaptive_timing_) {
        last_timestamp_ = std::chrono::high_resolution_clock::now();
    }
}

std::vector<double> KalmanFilter::getState() const {
    std::vector<double> state(6);
    for (int i = 0; i < 6; i++) {
        state[i] = x(i);
    }
    return state;
}