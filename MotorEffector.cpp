//
// Created by hujiet on 2026/6/16.
//
#include "MotorEffector.h"

// 构造函数：初始化时加上 tau 和 omega_target
MotorEffector::MotorEffector(Eigen::Vector3d position, double rotor_inertia, double thrust_coeff, double torque_coeff, double direction)
        : pos_B(position), Jr(rotor_inertia), k_f(thrust_coeff), k_m(torque_coeff), spin_dir(direction),
          omega_motor(0.0), omega_target(0.0), tau(0.02) {} // ★ tau = 0.02s (20ms) 是典型 5 寸机的响应时间

// 设置目标转速（如果是仿真，这里可以加入一阶滞后的动力学更新）
void MotorEffector::set_speed(double speed) {
    omega_target = speed;
}

// 核心：向飞船主体提交力与力矩贡献 (严格遵守 FRD 前右下坐标系)
void MotorEffector::updateContributions(double time, BackSubContributions& contrib, const RigidBodyState& state) {
    // 1. 推力 (朝向机体上方，所以是 -Z 方向！)
    Eigen::Vector3d force_B(0, 0, -k_f * omega_motor * omega_motor);

    // 2. 气动反扭矩 (CCW电机spin=1，产生CW反扭矩，FRD下CW是 +Z！)
    Eigen::Vector3d torque_aero_B(0, 0, spin_dir * k_m * omega_motor * omega_motor);

    // 3. 偏航力矩 (极其优雅！用叉乘 r x F 自动算出 Roll 和 Pitch 力矩！)
    // 因为前面的 force_B 已经是 -Z 了，这里的叉乘算出来的符号绝对是对的！
    Eigen::Vector3d torque_thrust_B = pos_B.cross(force_B);

    // 4. 陀螺力矩
    Eigen::Vector3d h_rotor(0, 0, Jr * spin_dir * omega_motor);
    Eigen::Vector3d torque_gyro_B = -state.omega.cross(h_rotor);

    contrib.vecTrans += force_B;
    contrib.vecRot += (torque_aero_B + torque_thrust_B + torque_gyro_B);
}

void MotorEffector::computeDerivatives(double time, double dt){
    // 使用欧拉积分推演一阶惯性微分方程
    // 公式：dw/dt = (w_target - w_motor) / tau
    double omega_motor_dot = (omega_target - omega_motor) / tau;

    // 更新此时此刻电机的真实转速
    omega_motor += omega_motor_dot * dt;
}

