//
// Created by hujiet on 2026/6/16.
//
#include "MotorEffector.h"

MotorEffector::MotorEffector(Eigen::Vector3d position, double rotor_inertia, double thrust_coeff, double torque_coeff, double direction)
        : pos_B(position), Jr(rotor_inertia), k_f(thrust_coeff), k_m(torque_coeff), spin_dir(direction), omega_motor(0.0) {}

// 设置目标转速（如果是仿真，这里可以加入一阶滞后的动力学更新）
void MotorEffector::set_speed(double speed) {
    omega_motor = speed;
}

// 核心：向飞船主体提交力与力矩贡献
void MotorEffector::updateContributions(double time, BackSubContributions& contrib, const RigidBodyState& state) {
    // 推力
    Eigen::Vector3d force_B(0, 0, k_f * omega_motor * omega_motor);
    // 气动反扭矩
    Eigen::Vector3d torque_aero_B(0, 0, -spin_dir * k_m * omega_motor * omega_motor);
    // 偏航力矩
    Eigen::Vector3d torque_thrust_B = pos_B.cross(force_B);
    // 陀螺力矩 (注意这里使用 state.omega)
    Eigen::Vector3d h_rotor(0, 0, Jr * spin_dir * omega_motor);
    Eigen::Vector3d torque_gyro_B = -state.omega.cross(h_rotor);

    contrib.vecTrans += force_B;
    contrib.vecRot += (torque_aero_B + torque_thrust_B + torque_gyro_B);
}

void MotorEffector::computeDerivatives(double time, double dt){
    // 如果电机有一阶滞后模型，在这里更新电机的角加速度
    // 比如：omega_motor_dot = (omega_target - omega_motor) / tau;
}

