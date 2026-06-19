#include "Mixer.h"
#include <cmath>
#include <algorithm>

Mixer::Mixer(double arm_length, double k_f, double k_m, double min_speed, double max_speed)
    : l_(arm_length), k_f_(k_f), k_m_(k_m), min_speed_(min_speed), max_speed_(max_speed) {

    double c = k_m_ / k_f_;
    
    // 构建正向推力映射矩阵 M
    // 顺序严格对应咱们的设定: 0:FR(前右), 1:RR(后右), 2:RL(后左), 3:FL(前左)
    Eigen::Matrix4d M;
    M <<  1.0,  1.0,  1.0,  1.0,   // 总推力 (4个电机齐心协力)
         -l_,  -l_,   l_,   l_,    // Roll (右侧FR/RR用力，右边抬起，飞机向左倒，即负Roll)
          l_,  -l_,  -l_,   l_,    // ★ Pitch (前侧FR/FL用力，机头抬起，即正Pitch)
          c,   -c,    c,   -c;     // ★ Yaw (FR逆时针转，给机身顺时针的正向反扭力矩)

    // 飞控核心逻辑：求逆！得到控制分配矩阵
    allocation_matrix_inv_ = M.inverse();
}

std::vector<double> Mixer::allocate(double thrust, double tau_phi, double tau_theta, double tau_psi) {
    // 1. 组装期望指令向量
    Eigen::Vector4d commands(thrust, tau_phi, tau_theta, tau_psi);

    // 2. 矩阵乘法：一键解算出四个电机所需的绝对物理推力 (Newtons)
    Eigen::Vector4d forces = allocation_matrix_inv_ * commands;

    // 3. 换算转速并进行极其严格的安全限幅
    std::vector<double> speeds(4);
    for (int i = 0; i < 4; ++i) {
        // 防止负推力 (电机不可能反转吹风)
        double f = std::max(0.0, forces(i)); 
        
        // 推力转角速度 (omega = sqrt(F / k_f))
        double speed = std::sqrt(f / k_f_);  
        
        // C++17 的 clamp 优雅限幅，防止转速爆表或停转
        speeds[i] = std::clamp(speed, min_speed_, max_speed_);
    }

    return speeds;
}