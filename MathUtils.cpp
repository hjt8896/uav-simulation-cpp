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
}