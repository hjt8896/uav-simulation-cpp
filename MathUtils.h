#ifndef AIRCRAFT_SIM_MATHUTILS_H
#define AIRCRAFT_SIM_MATHUTILS_H

#pragma once
#include "eigen3/Eigen/Dense"

namespace MathUtils {
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::Matrix3d mrp_to_dcm(const Eigen::Vector3d& sigma); // 返回 R_NB (N到B)
    Eigen::Matrix3d mrp_B_matrix(const Eigen::Vector3d& sigma);
    Eigen::Vector3d mrp_kinematics(const Eigen::Vector3d& sigma, const Eigen::Vector3d& omega);
}

#endif //AIRCRAFT_SIM_MATHUTILS_H