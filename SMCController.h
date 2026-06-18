//
// Created by hujiet on 2026/6/17.
//

#ifndef AIRCRAFT_SIM_SMCCONTROLLER_H
#define AIRCRAFT_SIM_SMCCONTROLLER_H


#pragma once
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <algorithm>

class SMCController {
private:
    Eigen::Matrix3d J;          // 机身转动惯量矩阵

    // 控制器增益矩阵 (通常是对角阵)
    Eigen::Matrix3d lambda;     // 积分滑模面权重 (决定收敛速度)
    Eigen::Matrix3d eta;        // 线性趋近率增益
    Eigen::Matrix3d K;          // 切换控制增益 (必须大于系统最大可能扰动)

    double epsilon;             // 边界层厚度 (Boundary Layer)，用于平滑 sgn 函数

    Eigen::Vector3d integral_e; // 误差积分累加器
    Eigen::Vector3d max_integral; // 积分限幅边界 (Anti-Windup)

    // 辅助函数：向量的平滑饱和函数 sat(s/epsilon)
    Eigen::Vector3d sat(const Eigen::Vector3d& s) const;

public:
    // 构造函数：初始化所有物理参数和增益
    SMCController(Eigen::Matrix3d inertia);

    // 重置控制器 (比如在飞机解锁 Disarm 时调用)
    void reset();

    // 核心计算律：1000Hz 循环调用
    Eigen::Vector3d compute_torque(const Eigen::Vector3d& omega,
                                   const Eigen::Vector3d& omega_d,
                                   const Eigen::Vector3d& omega_d_dot,
                                   double dt);
};


#endif //AIRCRAFT_SIM_SMCCONTROLLER_H