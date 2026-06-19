#ifndef AIRCRAFT_SIM_MIXER_H
#define AIRCRAFT_SIM_MIXER_H

#pragma once
#include <vector>
#include "eigen3/Eigen/Dense"

class Mixer {
public:
    Mixer(double arm_length, double k_f, double k_m, double min_speed, double max_speed);
    
    // 输入：期望总推力(N)，期望Roll、Pitch、Yaw力矩(Nm)
    // 输出：严格对应 MuJoCo [FR, RR, RL, FL] 顺序的四个电机转速 (rad/s)
    std::vector<double> allocate(double thrust, double tau_phi, double tau_theta, double tau_psi);

private:
    double l_;
    double k_f_;
    double k_m_;
    double min_speed_;
    double max_speed_;
    
    // 预先算好的混控逆矩阵
    Eigen::Matrix4d allocation_matrix_inv_; 
};

#endif