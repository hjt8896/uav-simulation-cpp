#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include "CommonTypes.h"
#include "RigidBody.h"
#include "MotorEffector.h"
#include "Mixer.h"
#include "SMCController.h"
#include "MRPSteering.h"
#include "WindEffector.h"
#include "AttitudeUKF.h"
#include "BMI088_Sensor.h"
#include "NotchFilter.h"
#include "MathUtils.h"

// 辅助函数：将真实状态映射为传感器需要的比力
Eigen::Vector3d get_true_specific_force(const RigidBodyState& state) {
    Eigen::Matrix3d R_NB = MathUtils::mrp_to_dcm(state.sigma);
    Eigen::Vector3d f_N(0, 0, -9.81);
    return R_NB * (state.a + f_N);
}

int main() {
    std::cout << "--- SR-UKF & SMC 终极平行宇宙对比仿真 ---" << std::endl;

    double mass = 1.0;
    Eigen::Matrix3d inertia = Eigen::Vector3d(0.01, 0.01, 0.02).asDiagonal();
    double l = 0.2, k_f = 1e-5, k_m = 2e-7, Jr = 1e-5;

    // ================= 1. 初始化理想无人机 (上帝视角) =================
    RigidBody quad_ideal(mass, inertia);
    quad_ideal.state.p = Eigen::Vector3d(0, 0, -10.0);
    MotorEffector m1_i(Eigen::Vector3d(l, l, 0), Jr, k_f, k_m, 1.0);
    MotorEffector m2_i(Eigen::Vector3d(-l, -l, 0), Jr, k_f, k_m, 1.0);
    MotorEffector m3_i(Eigen::Vector3d(l, -l, 0), Jr, k_f, k_m, -1.0);
    MotorEffector m4_i(Eigen::Vector3d(-l, l, 0), Jr, k_f, k_m, -1.0);
    quad_ideal.add_effector(&m1_i); quad_ideal.add_effector(&m2_i);
    quad_ideal.add_effector(&m3_i); quad_ideal.add_effector(&m4_i);

    // ================= 2. 初始化真实无人机 (带有传感器与 UKF) =================
    RigidBody quad_real(mass, inertia);
    quad_real.state.p = Eigen::Vector3d(0, 0, -10.0);
    MotorEffector m1_r(Eigen::Vector3d(l, l, 0), Jr, k_f, k_m, 1.0);
    MotorEffector m2_r(Eigen::Vector3d(-l, -l, 0), Jr, k_f, k_m, 1.0);
    MotorEffector m3_r(Eigen::Vector3d(l, -l, 0), Jr, k_f, k_m, -1.0);
    MotorEffector m4_r(Eigen::Vector3d(-l, l, 0), Jr, k_f, k_m, -1.0);
    quad_real.add_effector(&m1_r); quad_real.add_effector(&m2_r);
    quad_real.add_effector(&m3_r); quad_real.add_effector(&m4_r);

    // ================= 3. 环境与算法初始化 =================
    // 阵风：在 0.5s 到 0.6s 施加 0.5Nm Roll 轴力矩
    WindEffector gust_ideal(0.5, 0.6, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.0, 0.0));
    WindEffector gust_real(0.5, 0.6, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.0, 0.0));
    quad_ideal.add_effector(&gust_ideal);
    quad_real.add_effector(&gust_real);

    // 控制器 (两架飞机各一套)
    Mixer mixer(l, k_f, k_m, 100.0, 1000.0);
    SMCController smc_ideal(inertia), smc_real(inertia);
    MRPSteering steering_ideal(2.0, 0.5, 10.0, false), steering_real(2.0, 0.5, 10.0, false);

    // 传感器与滤波器 (仅真实飞机拥有)
    BMI088_Sensor bmi088;
    AttitudeUKF ukf;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9);
    Q.block<3,3>(0,0) *= 1e-6; Q.block<3,3>(3,3) *= 1e-4; Q.block<3,3>(6,6) *= 1e-8;
    ukf.setProcessNoise(Q);

    NotchFilter notch_gyro_x, notch_gyro_y, notch_gyro_z;
    NotchFilter notch_acc_x, notch_acc_y, notch_acc_z;
    float f_motor = 200.0f, f_samp = 1000.0f;
    notch_gyro_x.init(f_motor, f_samp, 2.0f); notch_gyro_y.init(f_motor, f_samp, 2.0f); notch_gyro_z.init(f_motor, f_samp, 2.0f);
    notch_acc_x.init(f_motor, f_samp, 2.0f);  notch_acc_y.init(f_motor, f_samp, 2.0f);  notch_acc_z.init(f_motor, f_samp, 2.0f);

    // ================= 4. 并行仿真循环 =================
    double t = 0.0, dt = 0.001;
    double base_thrust = mass * 9.81;
    Eigen::Vector3d target_sigma(0, 0, 0);
    std::vector<double> motor_speeds_real = {0,0,0,0}; // 用于生成震动

    std::ofstream log("ab_test_log.csv");
    log << "time,ideal_roll,real_roll,ukf_est_roll,ideal_tau,real_tau\n";

    std::cout << "起飞！正在穿越 0.5s 的恐怖阵风..." << std::endl;

    for (int i = 0; i <= 2000; ++i) {
        // ----------------- [A] 理想无人机 (上帝视角) -----------------
        Eigen::Vector3d w_d_i, wd_dot_i;
        steering_ideal.compute_steering(quad_ideal.state.sigma - target_sigma, w_d_i, wd_dot_i);
        Eigen::Vector3d tau_i = smc_ideal.compute_torque(quad_ideal.state.omega, w_d_i, wd_dot_i, dt);
        auto spds_i = mixer.allocate(base_thrust, tau_i(0), tau_i(1), tau_i(2));
        m1_i.set_speed(spds_i[0]); m2_i.set_speed(spds_i[1]); m3_i.set_speed(spds_i[2]); m4_i.set_speed(spds_i[3]);
        quad_ideal.step_rk4(t, dt);

        // ----------------- [B] 真实无人机 (闭环滤波) -----------------
        // 1. 传感器获取污染数据 (使用上一帧的电机转速生成震动)
        Eigen::Vector3d gyro_raw = bmi088.read_gyro(quad_real.state.omega, motor_speeds_real, t, dt);
        Eigen::Vector3d acc_raw = bmi088.read_acc(get_true_specific_force(quad_real.state), motor_speeds_real, t);

        // 罗盘数据 (加入白噪声)
        Eigen::Matrix3d R_NB = MathUtils::mrp_to_dcm(quad_real.state.sigma);
        Eigen::Vector3d mag_raw = R_NB * Eigen::Vector3d(0.22, 0.0, 0.45) + Eigen::Vector3d::Random() * 0.02;

        // 2. 陷波净化
        Eigen::Vector3d gyro_clean(notch_gyro_x.apply(gyro_raw(0)), notch_gyro_y.apply(gyro_raw(1)), notch_gyro_z.apply(gyro_raw(2)));
        Eigen::Vector3d acc_clean(notch_acc_x.apply(acc_raw(0)), notch_acc_y.apply(acc_raw(1)), notch_acc_z.apply(acc_raw(2)));

        // 3. SR-UKF 状态解算
        ukf.predict(dt);
        ukf.update_gyro(gyro_clean);
        ukf.update_accel(acc_clean);
        if (i % 20 == 0) ukf.update_mag(mag_raw);

        // 4. ★ 极其硬核：将 UKF 估计的“幻觉”喂给滑模控制器！
        Eigen::Vector3d w_d_r, wd_dot_r;
        steering_real.compute_steering(ukf.get_mrp() - target_sigma, w_d_r, wd_dot_r);
        Eigen::Vector3d tau_r = smc_real.compute_torque(ukf.get_omega(), w_d_r, wd_dot_r, dt);

        auto spds_r = mixer.allocate(base_thrust, tau_r(0), tau_r(1), tau_r(2));
        m1_r.set_speed(spds_r[0]); m2_r.set_speed(spds_r[1]); m3_r.set_speed(spds_r[2]); m4_r.set_speed(spds_r[3]);
        motor_speeds_real = {spds_r[0], spds_r[1], spds_r[2], spds_r[3]}; // 更新真实转速

        quad_real.step_rk4(t, dt);

        // ----------------- [C] 数据记录 -----------------
        log << t << ","
            << quad_ideal.state.sigma(0) << ","
            << quad_real.state.sigma(0) << ","
            << ukf.get_mrp()(0) << ","
            << tau_i(0) << "," << tau_r(0) << "\n";

        t += dt;
    }

    log.close();
    std::cout << "仿真结束！请用 Python 脚本查看 ab_test_log.csv 的震撼对比！" << std::endl;
    return 0;
}