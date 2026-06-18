//
// Created by hujiet on 2026/6/16.
//

#ifndef AIRCRAFT_SIM_MOTOREFFECTOR_H
#define AIRCRAFT_SIM_MOTOREFFECTOR_H


#include "eigen3/Eigen/Dense"
#include "BaseEffector.h"
class MotorEffector : public BaseEffector {
private:
    Eigen::Vector3d pos_B;    // 电机在机体坐标系下的安装位置 (x, y, z)
    double Jr;                // 转子的转动惯量
    double k_f;               // 升力系数 (Thrust coefficient)
    double k_m;               // 反扭矩系数 (Torque coefficient)
    double spin_dir;          // 旋转方向：1.0 为 CCW, -1.0 为 CW

    double omega_motor;       // 当前电机转速 (rad/s)
    double omega_target; // ★ 电调接收到的目标转速指令
    double tau;          // ★ 电机的一阶惯性时间常数 (响应快慢)

public:
    MotorEffector(Eigen::Vector3d position, double rotor_inertia, double thrust_coeff, double torque_coeff, double direction);

    // 设置目标转速（如果是仿真，这里可以加入一阶滞后的动力学更新）
    void set_speed(double speed);

    // 核心：向飞船主体提交力与力矩贡献
    void updateContributions(double time, BackSubContributions& contrib, const RigidBodyState& state) override;

    void computeDerivatives(double time, double dt) override;
};


#endif //AIRCRAFT_SIM_MOTOREFFECTOR_H