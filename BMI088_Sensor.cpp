//
// Created by hujiet on 2026/6/17.
//

#include "BMI088_Sensor.h"

BMI088_Sensor::BMI088_Sensor() : gen(std::random_device{}()),
                      gyro_noise_dist(0.0, 0.0077),  // 陀螺仪本底白噪声 0.0077 rad/s
                      acc_noise_dist(0.0, 0.054) {   // 加速度计本底白噪声 0.054 m/s^2

        // 陀螺仪初始化
        constant_gyro_bias << 0.01, -0.005, 0.001; // 静态零偏 (rad/s)
        dynamic_gyro_bias.setZero();

        T_c = 100.0;
        sigma_fogm = 0.001;
        double driving_noise_std = std::sqrt(2.0 * std::pow(sigma_fogm, 2) / T_c);
        fogm_driving_noise = std::normal_distribution<double>(0.0, driving_noise_std);

        // 加速度计初始化 (假设典型的装配应力带来的静态零偏)
        constant_acc_bias << 0.00, 0.00, 0.00; // 静态零偏 (m/s^2)
    }

// --- 内部辅助函数：合成电机高频振动 ---
void BMI088_Sensor::compute_vibration(const std::vector<double>& motor_speeds, double t,
                       Eigen::Vector3d& vib_gyro, Eigen::Vector3d& vib_acc) {
    vib_gyro.setZero();
    vib_acc.setZero();

    // 设定振动烈度
    double amp_gyro = 0.05; // 陀螺仪振动幅值 (rad/s)
    double amp_acc = 1.0;   // 加速度计振动幅值 (m/s^2，约0.1G)

    for (size_t i = 0; i < motor_speeds.size(); ++i) {
        double freq_hz = motor_speeds[i] / (2.0 * M_PI);

        // 简单的谐波叠加：使用正余弦以及相位偏移模拟多轴耦合的复杂机架共振
        double phase1 = 2.0 * M_PI * freq_hz * t;
        double phase2 = phase1 + M_PI_4; // 错开相位

        vib_gyro(0) += amp_gyro * std::sin(phase1);
        vib_gyro(1) += amp_gyro * std::cos(phase1);
        vib_gyro(2) += (amp_gyro * 0.5) * std::sin(phase2); // Z轴(偏航)振动通常较小

        vib_acc(0) += amp_acc * std::sin(phase1);
        vib_acc(1) += amp_acc * std::cos(phase1);
        vib_acc(2) += (amp_acc * 1.5) * std::sin(phase2); // Z轴(上下)振动通常最剧烈
    }
}

// --- 接口 1: 读取受污染的陀螺仪数据 ---
Eigen::Vector3d BMI088_Sensor::read_gyro(const Eigen::Vector3d& omega_true,
                          const std::vector<double>& motor_speeds,
                          double t, double dt) {

    // 1. FOGM 动态零偏积分
    Eigen::Vector3d w_k(fogm_driving_noise(gen), fogm_driving_noise(gen), fogm_driving_noise(gen));
    dynamic_gyro_bias = (1.0 - dt / T_c) * dynamic_gyro_bias + w_k * std::sqrt(dt);

    // 2. 本底高斯白噪声
    Eigen::Vector3d white_noise(gyro_noise_dist(gen), gyro_noise_dist(gen), gyro_noise_dist(gen));

    // 3. 计算机械振动
    Eigen::Vector3d vib_gyro, vib_acc;
    compute_vibration(motor_speeds, t, vib_gyro, vib_acc);

    return omega_true + constant_gyro_bias + dynamic_gyro_bias + white_noise + vib_gyro;
}

// --- 接口 2: 读取受污染的加速度计数据 ---
// acc_true: 刚体在机体系下的真实比力 (Specific Force)
Eigen::Vector3d BMI088_Sensor::read_acc(const Eigen::Vector3d& acc_true,
                         const std::vector<double>& motor_speeds,
                         double t) {

    // 1. 本底高斯白噪声
    Eigen::Vector3d white_noise(acc_noise_dist(gen), acc_noise_dist(gen), acc_noise_dist(gen));

    // 2. 计算机械振动
    Eigen::Vector3d vib_gyro, vib_acc;
    compute_vibration(motor_speeds, t, vib_gyro, vib_acc);

    // 合成输出：真实比力 + 静态偏置 + 白噪声 + 高频振动
    return acc_true + constant_acc_bias + white_noise + vib_acc;
}