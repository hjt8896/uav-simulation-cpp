#ifndef AIRCRAFT_SIM_ALTITUDESMC_H
#define AIRCRAFT_SIM_ALTITUDESMC_H

#pragma once
#include "eigen3/Eigen/Dense"

class AltitudeSMC {
public:
    // 传入飞机的质量
    AltitudeSMC(double vehicle_mass);

    // 核心计算律：计算带有倾角补偿的最终推力
    // current_z: 当前高度 (NED坐标系，向下为正)
    // target_z: 目标高度
    // current_vz: 当前垂直速度 (向下为正)
    // cos_tilt: 绝对倾角的余弦值 (即 R_NB 矩阵的 (2,2) 元素)
    double compute_thrust(double current_z, double target_z, double current_vz, double cos_tilt) const;

private:
    double m; // 飞机质量

    // Z轴位置环参数
    double K_pos_z;
    double max_vz;

    // Z轴速度滑模环参数
    double K_vz;
    double W_vz;
    double epsilon_z;

    // 饱和函数
    double sat(double s) const;
};

#endif // AIRCRAFT_SIM_ALTITUDESMC_H