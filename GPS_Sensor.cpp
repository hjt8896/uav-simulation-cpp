#include "GPS_Sensor.h"
#include <cmath>

GPS_Sensor::GPS_Sensor() : 
    gen(std::random_device{}()),
    xy_white_noise(0.0, 0.5), // XY 轴测量白噪声标准差 0.5m
    z_white_noise(0.0, 1.0),  // Z 轴由于卫星几何构型，误差翻倍 1.0m
    vel_noise(0.0, 0.1)       // GPS 多普勒测速误差 0.1m/s
{
    dynamic_bias.setZero();
    T_c = 300.0; // 星历漂移极慢，假设 300 秒 (5分钟)

    double sigma_fogm = 0.05; 
    double driving_noise_std = std::sqrt(2.0 * std::pow(sigma_fogm, 2) / T_c);
    fogm_driving_noise = std::normal_distribution<double>(0.0, driving_noise_std);

    update_period = 0.1; // 10Hz 刷新率
    latency = 0.1;       // 100ms 数据延迟
    last_update_time = 0.0;
}

bool GPS_Sensor::read_gps(const Eigen::Vector3d& pos_true, const Eigen::Vector3d& vel_true, 
                          double current_time, double dt, 
                          Eigen::Vector3d& out_pos, Eigen::Vector3d& out_vel) {
    
    // 1. 每毫秒都在暗中演化星历漂移 (FOGM)
    Eigen::Vector3d w_k(fogm_driving_noise(gen), fogm_driving_noise(gen), fogm_driving_noise(gen));
    dynamic_bias = (1.0 - dt / T_c) * dynamic_bias + w_k * std::sqrt(dt);

    // 2. 生成当前时刻带有噪声的原始观测值
    Eigen::Vector3d noisy_pos = pos_true + dynamic_bias;
    noisy_pos(0) += xy_white_noise(gen);
    noisy_pos(1) += xy_white_noise(gen);
    noisy_pos(2) += z_white_noise(gen);

    Eigen::Vector3d noisy_vel = vel_true + Eigen::Vector3d(vel_noise(gen), vel_noise(gen), vel_noise(gen));

    // 3. 压入延迟队列
    delay_buffer.push_back({current_time, noisy_pos, noisy_vel});

    // 4. 判断是否到了 GPS 模块 "吐数据" 的节拍 (10Hz)
    if (current_time - last_update_time >= update_period) {
        
        // 寻找符合硬件延迟 (100ms 前) 的那帧数据
        bool found = false;
        while (!delay_buffer.empty() && (current_time - delay_buffer.front().time >= latency)) {
            out_pos = delay_buffer.front().pos;
            out_vel = delay_buffer.front().vel;
            delay_buffer.pop_front();
            found = true;
        }

        if (found) {
            last_update_time = current_time;
            return true; // 告诉外部，收到了一帧新 GPS 数据！
        }
    }

    return false; // 当前毫秒没有 GPS 数据
}