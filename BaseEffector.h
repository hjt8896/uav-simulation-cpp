//
// Created by hujiet on 2026/6/16.
//

#ifndef AIRCRAFT_SIM_BASEEFFECTOR_H
#define AIRCRAFT_SIM_BASEEFFECTOR_H


#include "CommonTypes.h"
#include "eigen3/Eigen/Dense"

// 效应器贡献总和 (简化版的 Back-Substitution)
struct BackSubContributions {
    Eigen::Vector3d vecTrans; // 机体系下的合力
    Eigen::Vector3d vecRot;   // 机体系下的合力矩
};

class BaseEffector {
public:
    virtual ~BaseEffector() = default;

    // 更新为 3 个参数的版本
    virtual void updateContributions(double time, BackSubContributions& contrib, const RigidBodyState& state) = 0;

    // 更新为 2 个参数的版本
    virtual void computeDerivatives(double time, double dt) = 0;
};


#endif //AIRCRAFT_SIM_BASEEFFECTOR_H