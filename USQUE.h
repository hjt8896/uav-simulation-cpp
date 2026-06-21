#ifndef AIRCRAFT_SIM_USQUE_H
#define AIRCRAFT_SIM_USQUE_H

#pragma once
#include "MathUtils.h"

class USQUE {
private:
    static constexpr int n = 9;               // 误差状态维度: [delta_sigma, delta_omega, delta_bias]
    static constexpr int n_sigma = 2 * n + 1; // 19 个 Sigma 点

    // UKF 权重与超参数 (与之前保持完全一致)
    double alpha = 1e-3, kappa = 0.0, beta = 2.0;
    double lambda, gamma;
    Eigen::VectorXd wM, wC;

    // ==========================================
    // ★ 核心资产 1：外层名义状态 (防爆盾，绝对物理量)
    // ==========================================
    Eigen::Vector4d q_ref;     // 名义四元数 (标量在前)
    Eigen::Vector3d omega_ref; // 名义角速度
    Eigen::Vector3d bias_ref;  // 名义陀螺仪零偏

    // ==========================================
    // ★ 核心资产 2：内层误差状态 (计算引擎)
    // ==========================================
    Eigen::VectorXd error_x;   // 9维误差状态 (在预测和更新后必须强行清零！)
    Eigen::MatrixXd S;         // 9x9 满秩协方差平方根

    // 预分配内存，避免实时 new 对象
    Eigen::MatrixXd X_sigma;   // 误差 Sigma 点 (9 x 19)
    Eigen::MatrixXd S_Q;       // 过程噪声平方根
    Eigen::MatrixXd Y_sigma;
    // --- 独立传感器的本底观测噪声平方根 ---
    Eigen::Matrix3d S_R_mag;
    Eigen::Matrix3d S_R_gyro; // ★ 新增：陀螺仪观测噪声平方根
    void generateSigmaPoints();
    // 底层数学工具 (你之前手写的)
    static void cholDownDate(Eigen::MatrixXd& S, const Eigen::VectorXd& x, double w);

    // ★ 核心内部机制
    void inject_and_reset(); // 误差注入与重置机制

    // 绝对物理方程：只需要标准的四元数和角速度运动学！
    static Eigen::VectorXd absolute_kinematics(const Eigen::Vector4d& q, const Eigen::Vector3d& omega, const Eigen::Vector3d& bias);
    static void rk4_absolute_integration(Eigen::Vector4d& q, Eigen::Vector3d& omega, Eigen::Vector3d& bias, double dt);

public:
    USQUE();
    void setProcessNoise(const Eigen::MatrixXd& Q);
    void init_state(const Eigen::Vector4d& q0, const Eigen::Vector3d& w0, const Eigen::Vector3d& b0, const Eigen::MatrixXd& P0);

    // 预测步
    void predict(double dt);

    // 观测更新步 (抽象出统一接口)
    void measurement_update_core(const Eigen::Vector3d& Z_actual, const Eigen::MatrixXd& Z_sigma_pred, const Eigen::Matrix3d& S_R);

    void update_accel(const Eigen::Vector3d& Z_acc);
    void update_mag(const Eigen::Vector3d& Z_mag);
    void update_gyro(const Eigen::Vector3d& Z_gyro);
    // 传入绝对四元数、绝对角速度、绝对零偏
    static Eigen::Vector3d measurement_model_accel(const Eigen::Vector4d& q_abs);
    static Eigen::Vector3d measurement_model_mag(const Eigen::Vector4d& q_abs);
    static Eigen::Vector3d measurement_model_gyro(const Eigen::Vector3d& omega_abs, const Eigen::Vector3d& bias_abs);
    // 获取当前绝对姿态供外部闭环控制使用
    [[nodiscard]] Eigen::Vector4d get_quaternion() const { return q_ref; }
    [[nodiscard]] Eigen::Vector3d get_omega() const { return omega_ref; }
};

#endif //AIRCRAFT_SIM_USQUE_H