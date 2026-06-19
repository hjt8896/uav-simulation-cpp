//
// Created by hujiet on 2026/6/17.
//

#include "MRPSteering.h"
#include "MathUtils.h"
// // 辅助函数：MRP 运动学 B 矩阵 (与 RigidBody 里的逻辑一致)
// Eigen::Matrix3d MRPSteering::BmatMRP(const Eigen::Vector3d& sigma) {
//     double s2 = sigma.squaredNorm();
//     Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
//     Eigen::Matrix3d S_tilde;
//     S_tilde <<  0, -sigma(2), sigma(1),
//                 sigma(2), 0, -sigma(0),
//                -sigma(1), sigma(0), 0;
//     return (1.0 - s2) * I + 2.0 * S_tilde + 2.0 * sigma * sigma.transpose();
// }

// 构造函数：带有一组比较温和的默认参数
MRPSteering::MRPSteering(const double k1, const double k3, const double w_max, const bool ignore_ff)
    : K1(k1), K3(k3), omega_max(w_max), ignore_feedforward(ignore_ff) {}

// 动态调整参数 (方便后续调参)
void MRPSteering::set_gains(double k1, double k3, double w_max) {
    K1 = k1; K3 = k3; omega_max = w_max;
}

// 核心计算逻辑：外环 200Hz 或 1000Hz 调用
// 输入：当前姿态误差 sigma (本文假设目标姿态为 0，所以误差即为当前 sigma)
// 输出：期望角速度 omega_d，以及前馈角加速度 omega_d_dot
void MRPSteering::compute_steering(const Eigen::Vector3d& sigma,
                      Eigen::Vector3d& omega_d,
                      Eigen::Vector3d& omega_d_dot){

    // 1. 计算受限于 arctan 饱和曲线的期望角速度
    for (int i = 0; i < 3; ++i) {
        double sig = sigma(i);
        // Basilisk Eq 18: 经典非线性成型滤波器
        double val = std::atan(M_PI_2 / omega_max * (K1 * sig + K3 * sig * sig * sig)) / M_PI_2 * omega_max;
        omega_d(i) = -val; // 负号保证向误差的反方向收敛
    }

    // 初始化前馈项为 0
    omega_d_dot.setZero();

    // 2. 解析计算目标角加速度 (用于内环 SMC 的完美前馈)
    if (!ignore_feedforward) {
        // 利用 B 矩阵预测当前的姿态变化率 sigma_p
        // Eigen::Matrix3d B = BmatMRP(sigma);
        Eigen::Vector3d sigma_p = MathUtils::mrp_kinematics(sigma,omega_d);

        // Basilisk Eq 21: 解析求导链式法则
        for (int i = 0; i < 3; ++i) {
            double sig = sigma(i);
            double term1 = 3.0 * K3 * sig * sig + K1;
            double inner_term = M_PI_2 / omega_max * (K1 * sig + K3 * sig * sig * sig);
            double denominator = std::pow(inner_term, 2) + 1.0;

            double val = term1 / denominator;
            omega_d_dot(i) = -val * sigma_p(i);
            // 假设你算出的原始目标角速度是 omega_d_raw
            // 限制最大目标角速度为 3.0 rad/s (大约 170度/秒，极其灵敏但不会失控)
            double max_rate = 3.0;
            if (omega_d.norm() > max_rate) {
                omega_d = omega_d.normalized() * max_rate;
            }
        }
    }
}