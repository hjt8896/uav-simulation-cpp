#include "Barometer_Sensor.h"
#include <cmath>

Barometer_Sensor::Barometer_Sensor() : 
    gen(std::random_device{}()),
    hf_noise_dist(0.0, 0.2) // 假设旋翼气流带来 0.2m 的高频跳动 (标准差)
{
    dynamic_bias = 0.0;
    T_c = 100.0; // 假设 100 秒为一个大漂移周期 (慢速漂移)
    
    // FOGM 驱动噪声的标准差
    double sigma_fogm = 0.05; 
    double driving_noise_std = std::sqrt(2.0 * std::pow(sigma_fogm, 2) / T_c);
    fogm_driving_noise = std::normal_distribution<double>(0.0, driving_noise_std);
}

double Barometer_Sensor::read_baro(double z_true, double dt) {
    // 1. FOGM 动态零偏积分 (模拟随时间缓慢变化的天气气压漂移)
    double w_k = fogm_driving_noise(gen);
    dynamic_bias = (1.0 - dt / T_c) * dynamic_bias + w_k * std::sqrt(dt);

    // 2. 高频风噪 (白噪声)
    double hf_noise = hf_noise_dist(gen);

    // 3. 返回最终观测值
    return z_true + dynamic_bias + hf_noise;
}