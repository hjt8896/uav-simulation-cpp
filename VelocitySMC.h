#ifndef AIRCRAFT_SIM_VELOCITYSMC_H
#define AIRCRAFT_SIM_VELOCITYSMC_H

#pragma once
#include "eigen3/Eigen/Dense"

class VelocitySMC {
public:
    VelocitySMC(double vehicle_mass);
    
    // 输入：当前位置/速度，目标位置，dt
    // 输出：解耦出来的总推力 T 和 目标姿态 MRP sigma_d
    void compute_control(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                         const Eigen::Vector3d& p_d, double dt,
                         double& thrust_out, Eigen::Vector3d& sigma_d_out) const;

private:
    double m; // 质量
    
    // 控制参数
    double K_pos = 0.1; // 外环位置 P 增益
    Eigen::Vector3d K_v; // 内环速度滑模等速增益
    Eigen::Vector3d W_v; // 内环速度滑模指数趋近增益
    double epsilon = 0.1; // 饱和函数边界层厚度

    double sat(double s) const;
};

#endif