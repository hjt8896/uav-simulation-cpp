#ifndef AIRCRAFT_SIM_BAROMETER_SENSOR_H
#define AIRCRAFT_SIM_BAROMETER_SENSOR_H

#pragma once
#include <random>

class Barometer_Sensor {
private:
    std::mt19937 gen;
    std::normal_distribution<double> hf_noise_dist;       // 高频风噪 (旋翼气流)
    std::normal_distribution<double> fogm_driving_noise;  // FOGM 驱动白噪声

    double dynamic_bias; // 气压计的缓慢漂移
    double T_c;          // 漂移时间常数 (秒)

public:
    Barometer_Sensor();

    // 传入真实高度 (Z 轴坐标) 和积分步长 dt，返回受污染的气压计高度
    double read_baro(double z_true, double dt);
};

#endif //AIRCRAFT_SIM_BAROMETER_SENSOR_H