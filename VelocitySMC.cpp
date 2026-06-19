//
// Created by hujiet on 2026/6/19.
//

#include "VelocitySMC.h"
#include <cmath>

#include "MathUtils.h"

VelocitySMC::VelocitySMC(double vehicle_mass) : m(vehicle_mass) {
    // 1. 位置环刚度：赋予其极强的目标追踪欲望
    K_pos = 0.05;

    // 2. 边界层厚度：收紧边界，找回干脆利落的刹车手感
    epsilon = 2.0;

    // 3. 速度滑模增益：从老爷车升级为跑车！
    K_v << 3.0, 3.0, 5.0;  // 之前是 0.5, 0.5, 1.0
    W_v << 0.05, 0.05, 1.0;  // 之前是 0.2, 0.2, 0.5
}

double VelocitySMC::sat(double s) const {
    if (s > epsilon) return 1.0;
    if (s < -epsilon) return -1.0;
    return s / epsilon;
}

void VelocitySMC::compute_control(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                                 const Eigen::Vector3d& p_d, double dt,
                                 double& thrust_out, Eigen::Vector3d& sigma_d_out) const
{
    // 1. 外环位置 P 控制器 -> 生成期望速度 v_d
    Eigen::Vector3d e_p = p - p_d;
    Eigen::Vector3d v_d = -K_pos * e_p;

    // 限制期望速度，防止飞控开局暴走
    double max_vel = 5.0;
    if (v_d.norm() > max_vel) v_d = v_d.normalized() * max_vel;

    // 2. 内环速度滑模控制器
    Eigen::Vector3d s_v = v - v_d; // 滑模面

    Eigen::Vector3d v_d_dot = Eigen::Vector3d(0.5,0.5,0.5); // 简化的期望加速度
    Eigen::Vector3d g_N(0, 0, 9.81); // NED系下重力向下

    // 计算趋近律项
    Eigen::Vector3d saturation_term;
    saturation_term << sat(s_v(0)), sat(s_v(1)), sat(s_v(2));

    // 计算期望的总控制力向量 F_d_N
    Eigen::Vector3d F_d_N = m * (v_d_dot - g_N - K_v.cwiseProduct(s_v) - W_v.cwiseProduct(saturation_term));

    // 3. 终极代理解耦
    // 推力大小就是控制力向量的模长
    thrust_out = F_d_N.norm();

    // 提取期望的机体 Z 轴在地理系下的向量
    Eigen::Vector3d u_z_d = -F_d_N.normalized();

    // 4. 从推力矢量反推期望的 DCM 矩阵 (假设期望偏航角 psi_d = 0)
    // 构造一个与 u_z_d 严格正交的虚拟机体 X 轴和 Y 轴
    Eigen::Vector3d x_N_proj(1.0, 0.0, 0.0); // 期望的机头朝北
    Eigen::Vector3d u_y_d = u_z_d.cross(x_N_proj).normalized();
    Eigen::Vector3d u_x_d = u_y_d.cross(u_z_d).normalized();

    Eigen::Matrix3d R_NB_d;
    R_NB_d.col(0) = u_x_d; // 期望的机体 X 轴
    R_NB_d.col(1) = u_y_d; // 期望的机体 Y 轴
    R_NB_d.col(2) = u_z_d; // 期望的机体 Z 轴

    // 5. 将目标 DCM 矩阵转回目标 MRP (sigma_d)
    Eigen::Vector4d q = MathUtils::RotationMatrixToQuat(R_NB_d);
    // 四元数转 MRP: sigma = q_vector / (1 + q_scalar)
    sigma_d_out = Eigen::Vector3d(q(1), q(2), q(3)) / (1.0 + q(0));
}
