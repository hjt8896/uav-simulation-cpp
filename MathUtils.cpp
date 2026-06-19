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
}