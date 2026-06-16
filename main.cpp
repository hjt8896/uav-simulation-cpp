#include <iostream>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include "MotorEffector.h"
#include "RigidBody.h"
// TIP 要<b>Run</b>代码，请按 <shortcut actionId="Run"/> 或点击装订区域中的 <icon src="AllIcons.Actions.Execute"/> 图标。
int main() {
    std::cout << "--- Quadcopter Dynamics Simulation Initialization ---" << std::endl;

    // 1. 初始化无人机参数 (X字型)
    double mass = 1.0; // 1kg
    Eigen::Matrix3d inertia = Eigen::Vector3d(0.01, 0.01, 0.02).asDiagonal();
    RigidBody quad(mass, inertia);

    // 设定初始状态：在 10米 高空静止
    quad.state.p = Eigen::Vector3d(0, 0, 10);

    // 电机参数
    double l = 0.2;         // 轴距投影 (米)
    double k_f = 1e-5;      // 升力系数
    double k_m = 2e-7;      // 反扭矩系数
    double Jr = 1e-5;       // 转子惯量

    // 挂载 4 个电机效应器
    MotorEffector m1(Eigen::Vector3d( l,  l, 0), Jr, k_f, k_m,  1.0); // 右前 CCW
    MotorEffector m2(Eigen::Vector3d(-l, -l, 0), Jr, k_f, k_m,  1.0); // 左后 CCW
    MotorEffector m3(Eigen::Vector3d( l, -l, 0), Jr, k_f, k_m, -1.0); // 右后 CW
    MotorEffector m4(Eigen::Vector3d(-l,  l, 0), Jr, k_f, k_m, -1.0); // 左前 CW

    quad.add_effector(&m1);
    quad.add_effector(&m2);
    quad.add_effector(&m3);
    quad.add_effector(&m4);

    // 2. 计算完美悬停时的电机转速
    // 4 * k_f * omega^2 = m * g  =>  omega = sqrt(m * g / (4 * k_f))
    double hover_omega = std::sqrt((mass * 9.81) / (4.0 * k_f));
    std::cout << "Theoretical Hover Motor Speed: " << hover_omega << " rad/s" << std::endl;

    // 注入一个微小的扰动转速 (比如让 m1 多转 1 rad/s，看看非线性系统怎么崩溃)
    m1.set_speed(hover_omega);
    m2.set_speed(hover_omega);
    m3.set_speed(hover_omega);
    m4.set_speed(hover_omega);

    // 3. 仿真循环
    double t = 0.0;
    double dt = 0.001; // 1ms 步长，匹配 1000Hz 的控制频率
    int steps = 1000;  // 运行 1 秒

    std::cout << "\nStarting RK4 Integration..." << std::endl;
    for (int i = 0; i <= steps; ++i) {
        quad.step_rk4(t, dt);
        t += dt;

        // 每 100ms (0.1秒) 打印一次状态
        if (i % 100 == 0) {
            std::cout << "Time: " << t << "s | "
                      << "Alt(Z): " << quad.state.p(2) << " m | "
                      << "MRP: [" << quad.state.sigma(0) << ", "
                                  << quad.state.sigma(1) << ", "
                                  << quad.state.sigma(2) << "]" << std::endl;
        }
    }

    std::cout << "\nSimulation Complete. Due to the 1 rad/s disturbance on Motor 1, "
              << "you should observe the altitude dropping and MRP deviating from zero." << std::endl;

    return 0;
}