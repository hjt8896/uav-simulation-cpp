#ifndef AIRCRAFT_SIM_MATHUTILS_H
#define AIRCRAFT_SIM_MATHUTILS_H

#pragma once
#include "eigen3/Eigen/Dense"

namespace MathUtils {
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::Matrix3d mrp_to_dcm(const Eigen::Vector3d& sigma); // 返回 R_NB (N到B)
    Eigen::Matrix3d mrp_B_matrix(const Eigen::Vector3d& sigma);
    Eigen::Vector3d mrp_kinematics(const Eigen::Vector3d& sigma, const Eigen::Vector3d& omega);
    Eigen::Vector3d mrp_switchto_shadow(const Eigen::Vector3d& sigma);
    Eigen::Vector3d mrp_add(const Eigen::Vector3d& sigma_prime, const Eigen::Vector3d& sigma_double_prime);
    Eigen::Vector4d mrp_to_quaternion(const Eigen::Vector3d& sigma);
    Eigen::Vector3d quaternion_to_mrp(const Eigen::Vector4d& q);
    Eigen::Matrix3d QuatToRotationMatrix(const Eigen::Vector4d& q);
    Eigen::Matrix3d quat_to_dcm(const Eigen::Vector4d& q);
    Eigen::Vector4d RotationMatrixToQuat(const Eigen::Matrix3d& R);
    Eigen::Vector4d quat_mult(const Eigen::Vector4d& p, const Eigen::Vector4d& q);
    Eigen::Vector4d quat_inverse(const Eigen::Vector4d& q);
}

#endif //AIRCRAFT_SIM_MATHUTILS_H