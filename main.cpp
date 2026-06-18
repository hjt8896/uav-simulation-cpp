#include <iostream>
#include <vector>
#include <iomanip>
#include "CommonTypes.h"
#include "RigidBody.h"
#include "MotorEffector.h"
#include "Mixer.h"
#include "SMCController.h"
#include "MRPSteering.h"
#include "WindEffector.h"

int main() {
    std::cout << "--- Advanced Cascade Quadcopter Simulation ---" << std::endl;

    // ================= 1. 物理世界与无人机初始化 =================
    double mass = 1.0;
    Eigen::Matrix3d inertia = Eigen::Vector3d(0.01, 0.01, 0.02).asDiagonal();
    RigidBody quad(mass, inertia);
    quad.state.p = Eigen::Vector3d(0, 0, 10); // 10米高空悬停

    double l = 0.2, k_f = 1e-5, k_m = 2e-7, Jr = 1e-5;
    MotorEffector m1(Eigen::Vector3d( l,  l, 0), Jr, k_f, k_m,  1.0);
    MotorEffector m2(Eigen::Vector3d(-l, -l, 0), Jr, k_f, k_m,  1.0);
    MotorEffector m3(Eigen::Vector3d( l, -l, 0), Jr, k_f, k_m, -1.0);
    MotorEffector m4(Eigen::Vector3d(-l,  l, 0), Jr, k_f, k_m, -1.0);
    quad.add_effector(&m1); quad.add_effector(&m2);
    quad.add_effector(&m3); quad.add_effector(&m4);

    // ================= 2. 飞控算法大脑初始化 =================
    // 神经分配器 (Mixer)
    Mixer mixer(l, k_f, k_m, 100.0, 1000.0);

    // 内环：滑模控制器 (SMC)
    SMCController smc(inertia);

    // 外环：MRP 运动学控制器
    MRPSteering steering(2.0, 0.5, 10.0, false); // K1=2.0, K3=0.5, 最大10rad/s

    // 实例化一个阵风扰动：在 0.5s 到 0.6s 期间，施加 0.5Nm 的 Roll 轴力矩
    WindEffector gust_wind(0.5, 0.6, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.0, 0.0));
    quad.add_effector(&gust_wind); // 直接像挂电机一样挂在飞机上！

    // ================= 3. 闭环仿真大循环 =================
    double t = 0.0;
    double base_thrust = mass * 9.81; // 标称悬停推力
    Eigen::Vector3d target_sigma(0, 0, 0); // 目标姿态：完美水平

    std::cout << "Simulation running. A massive 0.5Nm Roll disturbance will hit at T=0.5s!" << std::endl;


    for (int i = 0; i <= 2000; ++i) {
        double dt = 0.001;

        // 1. 外环 (Attitude)
        Eigen::Vector3d omega_d, omega_d_dot;
        steering.compute_steering(quad.state.sigma - target_sigma, omega_d, omega_d_dot);

        // 2. 内环 (Rate)
        Eigen::Vector3d tau_cmd = smc.compute_torque(quad.state.omega, omega_d, omega_d_dot, dt);

        // 3. 分配
        auto speeds = mixer.allocate(base_thrust, tau_cmd(0), tau_cmd(1), tau_cmd(2));
        m1.set_speed(speeds[0]); m2.set_speed(speeds[1]);
        m3.set_speed(speeds[2]); m4.set_speed(speeds[3]);

        // 4. 物理世界时间推移 (风力会在底层被自动结算！)
        quad.step_rk4(t, dt);
        t += dt;

        // --- F. 终端遥测数据打印 ---
        if (i % 100 == 0) { // 每 0.1s 打印一次
            std::cout << std::fixed << std::setprecision(4)
                      << "T: " << t << "s | "
                      << "Roll MRP: " << std::setw(8) << quad.state.sigma(0) << " | "
                      << "Motor1: " << std::setw(6) << speeds[0] << " rad/s"
                      << std::endl;
        }
    }

    std::cout << "\nTest Complete. The SMC Controller should have crushed the wind disturbance!" << std::endl;
    return 0;
}