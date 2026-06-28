#ifndef AIRCRAFT_SIM_GPS_SENSOR_H
#define AIRCRAFT_SIM_GPS_SENSOR_H

#pragma once
#include <random>
#include <deque>
#include "eigen3/Eigen/Dense"

// 暂存历史数据的结构体
struct GpsData {
    double time;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};

class GPS_Sensor {
private:
    std::mt19937 gen;
    std::normal_distribution<double> xy_white_noise; // XY 轴白噪声
    std::normal_distribution<double> z_white_noise;  // Z 轴白噪声 (通常更差)
    std::normal_distribution<double> vel_noise;      // 速度白噪声
    std::normal_distribution<double> fogm_driving_noise; // FOGM 星历漂移驱动噪声

    Eigen::Vector3d dynamic_bias; // XYZ 的星历漂移
    double T_c;                   // 漂移时间常数

    std::deque<GpsData> delay_buffer; // 延迟队列
    double update_period;             // GPS 刷新周期 (例如 10Hz = 0.1s)
    double latency;                   // GPS 硬件延迟 (例如 100ms = 0.1s)
    double last_update_time;          // 上次发送数据的时间

public:
    GPS_Sensor();

    // 因为 GPS 不是每毫秒都有数据，所以返回 bool。
    // 只有返回 true 时，out_pos 和 out_vel 才会被填入最新的延迟数据。
    bool read_gps(const Eigen::Vector3d& pos_true, const Eigen::Vector3d& vel_true,
                  double current_time, double dt,
                  Eigen::Vector3d& out_pos, Eigen::Vector3d& out_vel);
};

#endif //AIRCRAFT_SIM_GPS_SENSOR_H