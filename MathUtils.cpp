//
// Created by hujiet on 2026/6/18.
//

#include "MathUtils.h"
#include <cmath>

namespace MathUtils {

    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m <<  0,   -v(2),  v(1),
             v(2),  0,    -v(0),
            -v(1),  v(0),  0;
        return m;
    }
// ========================================================
// MRP 转换为 旋转矩阵 (DCM)
// 假设输入的是MRP，表示从地理系(N)到机体系(B)的旋转，输出为 N到B 坐标变换阵
// ========================================================
    Eigen::Matrix3d mrp_to_dcm(const Eigen::Vector3d& sigma) {
        double s2 = sigma.squaredNorm();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d S = skew(sigma);
        return I + (8.0 * S * S - 4.0 * (1.0 - s2) * S) / std::pow(1.0 + s2, 2);
    }

    Eigen::Matrix3d mrp_B_matrix(const Eigen::Vector3d& sigma) {
        double s2 = sigma.squaredNorm();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d S = skew(sigma);
        return (1.0 - s2) * I + 2.0 * S + 2.0 * sigma * sigma.transpose();
    }

    Eigen::Vector3d mrp_kinematics(const Eigen::Vector3d& sigma, const Eigen::Vector3d& omega) {
        return 0.25 * mrp_B_matrix(sigma) * omega;
    }

    Eigen::Vector3d mrp_switchto_shadow(const Eigen::Vector3d& sigma)
    {
        // ★ 核心修复：MRP 影子集切换 (Shadow Set Switching)
        double sigma_sq = sigma.squaredNorm();

        // 如果模长平方大于 1 (意味着误差角超过 180 度)
        // 立即映射到影子集，保证它走最短路径，且绝对不会撞上 360 度奇异点
        if (sigma_sq > 1.0) {
            return -sigma / sigma_sq;
        }
        else return sigma;
    }

    // ★ 根据 Schaub & Junkins Eq (3.147) 实现的 MRP 绝对叠加公式
    Eigen::Vector3d mrp_add(const Eigen::Vector3d& sigma_prime, const Eigen::Vector3d& sigma_double_prime) {
        double n1 = sigma_prime.squaredNorm();
        double n2 = sigma_double_prime.squaredNorm();
        double dot = sigma_prime.dot(sigma_double_prime);

        // 分子: (1 - |sigma'|^2) * sigma'' + (1 - |sigma''|^2) * sigma' - 2 * sigma'' x sigma'
        Eigen::Vector3d num = (1.0 - n1) * sigma_double_prime +
                              (1.0 - n2) * sigma_prime -
                              2.0 * sigma_double_prime.cross(sigma_prime);

        // 分母: 1 + |sigma'|^2 * |sigma''|^2 - 2 * sigma'·sigma''
        double den = 1.0 + n1 * n2 - 2.0 * dot;

        return num / den;
    }

    // 将误差 MRP 转换为误差四元数 (标量在前 q0, q1, q2, q3)
     Eigen::Vector4d mrp_to_quaternion(const Eigen::Vector3d& sigma) {
        double sq_norm = sigma.squaredNorm();
        double scale = 1.0 / (1.0 + sq_norm);

        Eigen::Vector4d q;
        q(0) = scale * (1.0 - sq_norm);              // 标量部分 q0
        q.segment<3>(1) = scale * 2.0 * sigma;       // 矢量部分 q1, q2, q3

        return q;
    }

    // 将四元数转换为 MRP (带最短路径强制转换，防止掉入影子集)
     Eigen::Vector3d quaternion_to_mrp(const Eigen::Vector4d& q) {
        Eigen::Vector4d q_work = q;

        // USQUE 终极保命符：强制 $q_0$ 为正，保证映射出的 MRP 永远在原点附近
        if (q_work(0) < 0.0) {
            q_work = -q_work;
        }

        double denominator = 1.0 + q_work(0);

        // 防止极端的数值奇异点 (虽然在 USQUE 误差空间里极难发生，但为了代码健壮性)
        if (std::abs(denominator) < 1e-12) {
            return Eigen::Vector3d::Zero(); // 或者返回你定义的极大值
        }

        return q_work.segment<3>(1) / denominator;
    }
    // ========================================================
    // 1. 四元数 (w, x, y, z) 转换为 旋转矩阵 (DCM)
    // 假设输入的是单位四元数，表示从地理系(N)到机体系(B)的旋转，输出为主动旋转矩阵，注意和上面不一样
    // ========================================================
    Eigen::Matrix3d QuatToRotationMatrix(const Eigen::Vector4d& q) {
        // 提取分量 (MuJoCo和航天惯例：q0是标量w)
        double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
        Eigen::Matrix3d R;

        // 极其暴力的代数展开
        R(0, 0) = 1.0 - 2.0*qy*qy - 2.0*qz*qz;
        R(0, 1) = 2.0*(qx*qy - qw*qz);
        R(0, 2) = 2.0*(qx*qz + qw*qy);

        R(1, 0) = 2.0*(qx*qy + qw*qz);
        R(1, 1) = 1.0 - 2.0*qx*qx - 2.0*qz*qz;
        R(1, 2) = 2.0*(qy*qz - qw*qx);

        R(2, 0) = 2.0*(qx*qz - qw*qy);
        R(2, 1) = 2.0*(qy*qz + qw*qx);
        R(2, 2) = 1.0 - 2.0*qx*qx - 2.0*qy*qy;

        return R;
    }

    // 将四元数转换为方向余弦矩阵 (DCM)
    // 约定：标量在前 [q0, q1, q2, q3]^T
    // 作用：返回的矩阵 R_NB 可以将向量从参考系(N)映射到当前机体系(B)，即 v_B = R_NB * v_N
   Eigen::Matrix3d quat_to_dcm(const Eigen::Vector4d& q) {
        // 为了代码整洁和极致的访问速度，先做个解包
        double q0 = q(0);
        double q1 = q(1);
        double q2 = q(2);
        double q3 = q(3);

        Eigen::Matrix3d dcm;

        // 第一行
        dcm(0, 0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        dcm(0, 1) = 2.0 * (q1*q2 + q0*q3);
        dcm(0, 2) = 2.0 * (q1*q3 - q0*q2);

        // 第二行
        dcm(1, 0) = 2.0 * (q1*q2 - q0*q3);
        dcm(1, 1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        dcm(1, 2) = 2.0 * (q2*q3 + q0*q1);

        // 第三行
        dcm(2, 0) = 2.0 * (q1*q3 + q0*q2);
        dcm(2, 1) = 2.0 * (q2*q3 - q0*q1);
        dcm(2, 2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        return dcm;
    }

    // ========================================================
    // 2. 旋转矩阵 (DCM) 转换为 四元数 (w, x, y, z)输入为主动旋转矩阵，和上面对应
    // 采用了极其严谨的对角线最大值判断，彻底消灭除以 0 的隐患
    // ========================================================
    Eigen::Vector4d RotationMatrixToQuat(const Eigen::Matrix3d& R) {
        Eigen::Vector4d q;
        double tr = R.trace(); // 矩阵的迹: R00 + R11 + R22

        if (tr > 0.0) {
            // 正常情况：旋转角较小
            double S = std::sqrt(tr + 1.0) * 2.0; // S = 4 * qw
            q(0) = 0.25 * S;
            q(1) = (R(2, 1) - R(1, 2)) / S;
            q(2) = (R(0, 2) - R(2, 0)) / S;
            q(3) = (R(1, 0) - R(0, 1)) / S;
        } else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
            // 奇点保护分支 1：X 轴方向的主导旋转
            double S = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S = 4 * qx
            q(0) = (R(2, 1) - R(1, 2)) / S;
            q(1) = 0.25 * S;
            q(2) = (R(0, 1) + R(1, 0)) / S;
            q(3) = (R(0, 2) + R(2, 0)) / S;
        } else if (R(1, 1) > R(2, 2)) {
            // 奇点保护分支 2：Y 轴方向的主导旋转
            double S = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S = 4 * qy
            q(0) = (R(0, 2) - R(2, 0)) / S;
            q(1) = (R(0, 1) + R(1, 0)) / S;
            q(2) = 0.25 * S;
            q(3) = (R(1, 2) + R(2, 1)) / S;
        } else {
            // 奇点保护分支 3：Z 轴方向的主导旋转
            double S = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S = 4 * qz
            q(0) = (R(1, 0) - R(0, 1)) / S;
            q(1) = (R(0, 2) + R(2, 0)) / S;
            q(2) = (R(1, 2) + R(2, 1)) / S;
            q(3) = 0.25 * S;
        }

        // 确保四元数的标量 qw 始终为正（航天界约定，避免产生双覆盖歧义）
        if (q(0) < 0) {
            q = -q;
        }

        return q;
    }

    // 四元数乘法: q_res = p * q (注意：顺序极其重要！)
    // 约定：标量在前 [q0, q1, q2, q3]^T
     Eigen::Vector4d quat_mult(const Eigen::Vector4d& p, const Eigen::Vector4d& q) {
        Eigen::Vector4d q_res;

        // 拆包 p
        double p0 = p(0);
        Eigen::Vector3d p_vec = p.segment<3>(1);

        // 拆包 q
        double q0 = q(0);
        Eigen::Vector3d q_vec = q.segment<3>(1);

        // 组装标量部分
        q_res(0) = p0 * q0 - p_vec.dot(q_vec);

        // 组装矢量部分
        q_res.segment<3>(1) = p0 * q_vec + q0 * p_vec + p_vec.cross(q_vec);

        return q_res;
    }

    // 四元数求逆 (共轭)：q^{-1}
    // 用于计算两个姿态的相对旋转差: delta_q = q_actual * quat_inverse(q_ref)
     Eigen::Vector4d quat_inverse(const Eigen::Vector4d& q) {
        Eigen::Vector4d q_inv;
        q_inv(0) = q(0);                         // 标量不变
        q_inv.segment<3>(1) = -q.segment<3>(1);  // 矢量取反
        return q_inv;
    }
}