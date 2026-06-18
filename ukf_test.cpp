#include <iostream>
#include <fstream>
#include <cmath>
#include "AttitudeUKF.h"
#include "BMI088_Sensor.h"
#include "NotchFilter.h"
// 辅助函数：MRP 运动学微分方程 (用于推演真实姿态)
Eigen::Vector3d compute_true_mrp_dot(const Eigen::Vector3d& sigma, const Eigen::Vector3d& omega) {
    double sigma_sq = sigma.squaredNorm();
    Eigen::Matrix3d sigma_cross;
    sigma_cross <<     0.0,  -sigma(2),   sigma(1),
                  sigma(2),        0.0,  -sigma(0),
                 -sigma(1),   sigma(0),        0.0;

    Eigen::Matrix3d B = (1.0 - sigma_sq) * Eigen::Matrix3d::Identity()
                      + 2.0 * sigma_cross
                      + 2.0 * sigma * sigma.transpose();
    return 0.25 * B * omega;
}

// 辅助函数：MRP 转 DCM (用于把真实的地理场投影到机体系，供传感器读取)
Eigen::Matrix3d true_mrp_to_dcm(const Eigen::Vector3d& sigma) {
    double sigma_sq = sigma.squaredNorm();
    double den = 1.0 + sigma_sq;
    double den_sq = den * den;
    Eigen::Matrix3d sigma_cross;
    sigma_cross <<     0.0,  -sigma(2),   sigma(1),
                  sigma(2),        0.0,  -sigma(0),
                 -sigma(1),   sigma(0),        0.0;
    return Eigen::Matrix3d::Identity()
         - (4.0 * (1.0 - sigma_sq) / den_sq) * sigma_cross
         + (8.0 / den_sq) * (sigma_cross * sigma_cross);
}

int main() {
    std::cout << "--- 启动 SR-UKF 虚拟宇宙 (搭载纯血 BMI088 物理级模型) ---" << std::endl;

    // 1. 初始化算法与传感器
    AttitudeUKF ukf;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9);
    Q.block<3,3>(0,0) *= 1e-6;
    Q.block<3,3>(3,3) *= 1e-4;
    Q.block<3,3>(6,6) *= 1e-8;
    ukf.setProcessNoise(Q);

    // 初始化陷波滤波器 (干掉 200Hz 的电机震动)
    NotchFilter filter_acc_x, filter_acc_y, filter_acc_z;
    float f_samp = 1000.0f;
    float f_motor = 200.0f;
    filter_acc_x.init(f_motor, f_samp, 2.0f);
    filter_acc_y.init(f_motor, f_samp, 2.0f);
    filter_acc_z.init(f_motor, f_samp, 2.0f);

    // ★ 新增：为陀螺仪独立配备陷波器
    NotchFilter filter_gyro_x, filter_gyro_y, filter_gyro_z;
    filter_gyro_x.init(f_motor, f_samp, 2.0f);
    filter_gyro_y.init(f_motor, f_samp, 2.0f);
    filter_gyro_z.init(f_motor, f_samp, 2.0f);

    BMI088_Sensor bmi088;

    // 2. 仿真环境与物理真值初始化
    double dt = 0.001;
    double sim_duration = 10.0;
    int total_ticks = sim_duration / dt;

    Eigen::Vector3d true_sigma = Eigen::Vector3d::Zero(); // 初始真实姿态全平

    // 模拟 4 个电机的转速 (约 10000 RPM，产生高频破坏性震动)
    std::vector<double> motor_speeds = {1047.0, 1050.0, 1045.0, 1060.0};

    std::ofstream log_file("ukf_sim_log.csv");
    log_file << "time,true_p,true_q,true_r,est_p,est_q,est_r,true_roll_mrp,est_roll_mrp\n";

    // --- 上帝视角的时钟循环 ---
    for (int tick = 0; tick < total_ticks; ++tick) {
        double current_time = tick * dt;

        // ==========================================
        // 模块 A：宇宙真理 (生成真实的物理状态)
        // ==========================================
        // 设定无人机的真实角速度：一个优雅的组合正弦机动
        Eigen::Vector3d true_omega(
            1.0 * std::sin(2.0 * M_PI * 0.5 * current_time), // Roll 在摇摆
            0.5 * std::cos(2.0 * M_PI * 0.2 * current_time), // Pitch 在慢摇
            0.0
        );

        // 使用 RK4 积分出此时此刻绝对真实的姿态 true_sigma
        Eigen::Vector3d k1 = compute_true_mrp_dot(true_sigma, true_omega);
        Eigen::Vector3d k2 = compute_true_mrp_dot(true_sigma + 0.5 * dt * k1, true_omega);
        Eigen::Vector3d k3 = compute_true_mrp_dot(true_sigma + 0.5 * dt * k2, true_omega);
        Eigen::Vector3d k4 = compute_true_mrp_dot(true_sigma + dt * k3, true_omega);
        true_sigma += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        // ==========================================
        // 模块 B：传感器黑盒 (极其逼真的物理污染)
        // ==========================================
        // 1. 陀螺仪读取：真实角速度 + FOGM零偏 + 静态零偏 + 震动 + 白噪声
        Eigen::Vector3d gyro_read = bmi088.read_gyro(true_omega, motor_speeds, current_time, dt);

        // 2. 加速度计读取：必须先算出当前的真实比力
        Eigen::Matrix3d R_NB = true_mrp_to_dcm(true_sigma);
        Eigen::Vector3d f_N(0.0, 0.0, -9.81);
        Eigen::Vector3d true_acc_body = R_NB * f_N; // 将NED系下的重力比力投影到机体系

        Eigen::Vector3d acc_read = bmi088.read_acc(true_acc_body, motor_speeds, current_time);

        // 3. 罗盘读取 (这里临时手写一个污染模型，因为 BMI088 不带罗盘)
        Eigen::Vector3d mag_N(0.22, 0.0, 0.45);
        Eigen::Vector3d true_mag_body = R_NB * mag_N;
        Eigen::Vector3d mag_read = true_mag_body + Eigen::Vector3d::Random() * 0.02;

        // ==========================================
        // 模块 C：SR-UKF 飞控核心解算
        // ==========================================
        // C.1 陷波滤波：在送入 UKF 之前，先干掉 200Hz 震动！
        acc_read(0) = filter_acc_x.apply(acc_read(0));
        acc_read(1) = filter_acc_y.apply(acc_read(1));
        acc_read(2) = filter_acc_z.apply(acc_read(2));

        // ★ 新增：净化陀螺仪数据，保卫 PID 控制器！
        gyro_read(0) = filter_gyro_x.apply(gyro_read(0));
        gyro_read(1) = filter_gyro_y.apply(gyro_read(1));
        gyro_read(2) = filter_gyro_z.apply(gyro_read(2));

        ukf.predict(dt);
        ukf.update_gyro(gyro_read);
        ukf.update_accel(acc_read);

        if (tick % 20 == 0) {
            ukf.update_mag(mag_read);
        }

        // ==========================================
        // 模块 D：遥测与记录
        // ==========================================
        Eigen::Vector3d est_mrp = ukf.get_mrp();
        Eigen::Vector3d est_omega = ukf.get_omega(); // 拔出真正的估计角速度之剑

        log_file << current_time << ","
                 << true_omega(0) << "," << true_omega(1) << "," << true_omega(2) << ","
                 << est_omega(0) << "," << est_omega(1) << "," << est_omega(2) << ","
                 << true_sigma(0) << "," << est_mrp(0) << "\n";
    }

    log_file.close();
    std::cout << "--- 仿真结束，数据已写入 ukf_sim_log.csv ---" << std::endl;
    return 0;
}