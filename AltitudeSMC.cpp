//
// Created by hujiet on 2026/6/19.
//

#include "AltitudeSMC.h"
#include <algorithm>

AltitudeSMC::AltitudeSMC(double vehicle_mass) : m(vehicle_mass) {
    // 1. Z 轴位置环参数
    K_pos_z = 3.0; // 刚度拉满
    max_vz = 4.0;  // 允许的最大升降速度 4m/s

    // 2. Z 轴滑模控制参数 (极其强悍的抗掉高能力)
    K_vz = 5.0;      // 线性收敛增益
    W_vz = 3.0;      // 切换增益
    epsilon_z = 0.15;// 极薄的边界层，保证干脆的刹车
}

double AltitudeSMC::sat(double s) const {
    if (s > epsilon_z) return 1.0;
    if (s < -epsilon_z) return -1.0;
    return s / epsilon_z;
}

double AltitudeSMC::compute_thrust(double current_z, double target_z, double current_vz, double cos_tilt) const {
    // =====================================
    // 1. Z 轴位置 P 环 -> 生成期望下降速度 v_zd
    // =====================================
    double err_z = current_z - target_z;
    double v_zd = -K_pos_z * err_z;
    v_zd = std::clamp(v_zd, -max_vz, max_vz);

    // =====================================
    // 2. Z 轴速度滑模环 (SMC) -> 计算理想垂直升力 F_z
    // =====================================
    double s_z = current_vz - v_zd;

    // NED 系重力向下为正 9.81
    double F_z = m * 9.81 + m * (K_vz * s_z + W_vz * sat(s_z));

    // =====================================
    // 3. 姿态倾角补偿 (Tilt Compensation)
    // =====================================
    if (cos_tilt < 0.3) cos_tilt = 0.3; // 极限防翻车保护

    double dynamic_thrust = F_z / cos_tilt;

    // 限制最大物理推力防爆表 (假设1kg飞机给 30N 极限推力)
    dynamic_thrust = std::clamp(dynamic_thrust, 0.0, 30.0);

    return dynamic_thrust;
}