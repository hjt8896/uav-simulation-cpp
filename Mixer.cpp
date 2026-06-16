//
// Created by hujiet on 2026/6/16.
//

#include "Mixer.h"

Mixer::Mixer(const double l, const double k_f, const double k_m, const double min_w = 100.0, const double max_w = 1000.0)
        : min_omega(min_w), max_omega(max_w) {

    // 1. 根据物理构型构建正向矩阵 M
    // 顺序: M1(l,l,CCW), M2(-l,-l,CCW), M3(l,-l,CW), M4(-l,l,CW)
    M << k_f,      k_f,      k_f,      k_f,
         k_f * l, -k_f * l, -k_f * l,  k_f * l,
        -k_f * l,  k_f * l, -k_f * l,  k_f * l,
        -k_m,     -k_m,      k_m,      k_m;

    // 2. 预计算逆矩阵
    M_inv = M.inverse();
}

// 核心分配函数：输入目标推力与力矩，输出 4 个电机的目标转速
std::vector<double> Mixer::allocate(double Fz, double tau_phi, double tau_theta, double tau_psi) const
{
    Eigen::Vector4d control_input(Fz, tau_phi, tau_theta, tau_psi);

    // 解算目标转速的平方
    Eigen::Vector4d omega_sq = M_inv * control_input;

    std::vector<double> motor_speeds(4, 0.0);

    for (int i = 0; i < 4; ++i) {
        // 3. 饱和处理逻辑 (极其重要！)
        // 如果算出负的平方，说明指令越界，强制归零保护
        double sq_val = std::max(0.0, omega_sq(i));
        double speed = std::sqrt(sq_val);

        // 限制在电机的物理边界内
        speed = std::clamp(speed, min_omega, max_omega);
        motor_speeds[i] = speed;
    }

    return motor_speeds;
}
