//
// Created by hujiet on 2026/6/17.
//

#ifndef AIRCRAFT_SIM_MRPSTEERING_H
#define AIRCRAFT_SIM_MRPSTEERING_H


#pragma once
#include "eigen3/Eigen/Dense"
#include <cmath>

class MRPSteering {
private:
    // 核心增益参数
    double K1;          // 比例增益 (控制线性区)
    double K3;          // 三次项增益 (控制非线性加速区)
    double omega_max;   // 绝对饱和限幅阀值 (rad/s)

    bool ignore_feedforward; // 是否关闭前馈 (默认开启)

    // 辅助函数：MRP 运动学 B 矩阵 (与 RigidBody 里的逻辑一致)
    // static Eigen::Matrix3d BmatMRP(const Eigen::Vector3d& sigma);

public:
    // 构造函数：带有一组比较温和的默认参数
    MRPSteering(double k1 = 10.0, double k3 = 2.5, double w_max = 5.0, bool ignore_ff = false);

    // 动态调整参数 (方便后续调参)
    void set_gains(double k1, double k3, double w_max);

    // 核心计算逻辑：外环 200Hz 或 1000Hz 调用
    // 输入：当前姿态误差 sigma (本文假设目标姿态为 0，所以误差即为当前 sigma)
    // 输出：期望角速度 omega_d，以及前馈角加速度 omega_d_dot
    void compute_steering(const Eigen::Vector3d& sigma,
                          Eigen::Vector3d& omega_d,
                          Eigen::Vector3d& omega_d_dot);
};


#endif //AIRCRAFT_SIM_MRPSTEERING_H