//
// Created by hujiet on 2026/6/17.
//

#include "SMCController.h"

Eigen::Vector3d SMCController::sat(const Eigen::Vector3d& s) const
{
    Eigen::Vector3d res;
    for (int i = 0; i < 3; ++i) {
        if (s(i) > epsilon) res(i) = 1.0;
        else if (s(i) < -epsilon) res(i) = -1.0;
        else res(i) = s(i) / epsilon;
    }
    return res;
}

// 构造函数：初始化所有物理参数和增益
SMCController::SMCController(Eigen::Matrix3d inertia) : J(inertia) {
    integral_e.setZero();

    // 默认参数 (需要根据实际 1kg 飞机去整定)
    lambda = Eigen::Vector3d(10.0, 10.0, 5.0).asDiagonal();
    eta    = Eigen::Vector3d(5.0, 5.0, 5.0).asDiagonal();
    K      = Eigen::Vector3d(2.0, 2.0, 1.0).asDiagonal();

    epsilon = 0.1; // 边界层，越小越精确但更容易抖振
    max_integral = Eigen::Vector3d(2.0, 2.0, 2.0); // 积分限幅
}

// 重置控制器 (比如在飞机解锁 Disarm 时调用)
void SMCController::reset() {
    integral_e.setZero();
}

// 核心计算律：1000Hz 循环调用
Eigen::Vector3d SMCController::compute_torque(const Eigen::Vector3d& omega,
                               const Eigen::Vector3d& omega_d,
                               const Eigen::Vector3d& omega_d_dot,
                               const double dt) {
    // 1. 计算追踪误差
    Eigen::Vector3d e = omega - omega_d;

    // 2. 积分更新与限幅 (Anti-Windup)
    integral_e += e * dt;
    for (int i = 0; i < 3; ++i) {
        integral_e(i) = std::clamp(integral_e(i), -max_integral(i), max_integral(i));
    }

    // 3. 计算积分滑模面 s
    Eigen::Vector3d s = e + lambda * integral_e;

    // 4. 计算前馈与等效控制力矩 (抵消陀螺效应，跟踪目标加速度)
    Eigen::Vector3d tau_eq = omega.cross(J * omega) + J * omega_d_dot;

    // 5. 计算反馈与切换控制力矩 (压制误差和未知扰动)
    Eigen::Vector3d tau_feedback = -J * (lambda * e + eta * s);
    Eigen::Vector3d tau_sw = -J * K * sat(s);

    // 6. 综合输出目标力矩
    Eigen::Vector3d tau_total = tau_eq + tau_feedback + tau_sw;

    return tau_total;
}