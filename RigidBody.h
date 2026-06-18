//
// Created by hujiet on 2026/6/16.
//

#ifndef AIRCRAFT_SIM_RIGIDBODY_H
#define AIRCRAFT_SIM_RIGIDBODY_H

#include "BaseEffector.h"
#include <vector>



class RigidBody {
public:
    double m;             // 总质量
    Eigen::Matrix3d I;    // 总转动惯量矩阵
    RigidBodyState state; // 当前状态

    // 效应器池
    std::vector<BaseEffector*> effectors;

    RigidBody(double mass, Eigen::Matrix3d inertia);
    void add_effector(BaseEffector* eff);
    // 对应 BSK 的 equationsOfMotion: 计算状态在给定时间 t 的导数
    [[nodiscard]] RigidBodyState compute_derivatives(double time, const RigidBodyState& current_state) const;
    // RK4 积分器核心实现
    void step_rk4(double t, double dt);

private:
    // 辅助函数：用于 RK4 内部状态累加
    static RigidBodyState add_states(const RigidBodyState& s, const RigidBodyState& dot, double dt);
};
#endif //AIRCRAFT_SIM_RIGIDBODY_H