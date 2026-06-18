//
// Created by hujiet on 2026/6/17.
//

#ifndef AIRCRAFT_SIM_BMI088_SENSOR_H
#define AIRCRAFT_SIM_BMI088_SENSOR_H

#pragma once
#include "eigen3/Eigen/Dense"
#include <random>
#include <cmath>
#include <vector>

class BMI088_Sensor {
private:
    std::mt19937 gen;

    // 陀螺仪噪声与零偏参数
    std::normal_distribution<double> gyro_noise_dist;
    std::normal_distribution<double> fogm_driving_noise;
    Eigen::Vector3d constant_gyro_bias;
    Eigen::Vector3d dynamic_gyro_bias;

    double T_c;
    double sigma_fogm;

    // 加速度计噪声与零偏参数
    std::normal_distribution<double> acc_noise_dist;
    Eigen::Vector3d constant_acc_bias;

    // --- 内部辅助函数：合成电机高频振动 ---
    static void compute_vibration(const std::vector<double>& motor_speeds, double t,
                                  Eigen::Vector3d& vib_gyro, Eigen::Vector3d& vib_acc);
public:
    BMI088_Sensor();


    // --- 接口 1: 读取受污染的陀螺仪数据 ---
    Eigen::Vector3d read_gyro(const Eigen::Vector3d& omega_true,
                              const std::vector<double>& motor_speeds,
                              double t, double dt);

    // --- 接口 2: 读取受污染的加速度计数据 ---
    // acc_true: 刚体在机体系下的真实比力 (Specific Force)
    Eigen::Vector3d read_acc(const Eigen::Vector3d& acc_true,
                             const std::vector<double>& motor_speeds,
                             double t);
};

#endif //AIRCRAFT_SIM_BMI088_SENSOR_H