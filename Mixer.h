//
// Created by hujiet on 2026/6/16.
//

#ifndef AIRCRAFT_SIM_MIXER_H
#define AIRCRAFT_SIM_MIXER_H


#pragma once
#include "eigen3/Eigen/Dense"
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

class Mixer {
private:
    Eigen::Matrix4d M;       // 正向分配矩阵
    Eigen::Matrix4d M_inv;   // 逆向分配矩阵 (控制分配矩阵)

    double max_omega;        // 电机最大转速 (rad/s)
    double min_omega;        // 电机最小转速 (怠速, 防止空中停转)

public:
    Mixer(const double l, const double k_f, const double k_m, const double min_w , const double max_w );

    // 核心分配函数：输入目标推力与力矩，输出 4 个电机的目标转速
    std::vector<double> allocate(double Fz, double tau_phi, double tau_theta, double tau_psi) const;
};


#endif //AIRCRAFT_SIM_MIXER_H