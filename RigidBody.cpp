//
// Created by hujiet on 2026/6/16.
//
#include "RigidBody.h"

#include <utility>
#include "MathUtils.h"

RigidBody::RigidBody(double mass, Eigen::Matrix3d inertia) : m(mass), I(std::move(inertia)) {
    state.p.setZero();
    state.v.setZero();
    state.sigma.setZero();
    state.omega.setZero();
}

void RigidBody::add_effector(BaseEffector* eff) {
    effectors.push_back(eff);
}

// 对应 BSK 的 equationsOfMotion: 计算状态在给定时间 t 的导数
RigidBodyState RigidBody::compute_derivatives(double time, const RigidBodyState& current_state) const
{
    BackSubContributions contrib;
    contrib.vecTrans.setZero();
    contrib.vecRot.setZero();

    // 1. 轮询所有效应器，累加推力、反扭矩、陀螺力矩、角动量交换！
    for (auto* eff : effectors) {
        eff->updateContributions(time, contrib, current_state);
    }

    // 2. 引入重力 (FRD机体坐标系下)
    Eigen::Vector3d gravity_N(0, 0, 9.81);
    Eigen::Matrix3d R_NB = MathUtils::mrp_to_dcm(current_state.sigma);
    Eigen::Vector3d gravity_B = R_NB * gravity_N;
    contrib.vecTrans += m * gravity_B;

    // 3. 解牛顿-欧拉方程
    RigidBodyState dot;

    // 平移运动学
    dot.p = current_state.v;
    // 平移动力学 (惯性系下的加速度)
    dot.v = R_NB * (contrib.vecTrans / m);

    // 旋转运动学 (MRP 微分)
    dot.sigma = MathUtils::mrp_kinematics(current_state.sigma, current_state.omega);
    // 旋转动力学 (欧拉方程: I*dw + w x Iw = tau)
    dot.omega = I.inverse() * (contrib.vecRot - current_state.omega.cross(I * current_state.omega));

    return dot;
}

// RK4 积分器核心实现
void RigidBody::step_rk4(double t, double dt) {
    // 计算 k1, k2, k3, k4
    RigidBodyState k1 = compute_derivatives(t, state);
    RigidBodyState s2 = add_states(state, k1, dt / 2.0);

    RigidBodyState k2 = compute_derivatives(t + dt / 2.0, s2);
    RigidBodyState s3 = add_states(state, k2, dt / 2.0);

    RigidBodyState k3 = compute_derivatives(t + dt / 2.0, s3);
    RigidBodyState s4 = add_states(state, k3, dt);

    RigidBodyState k4 = compute_derivatives(t + dt, s4);

    // 累加更新状态
    state.p += (dt / 6.0) * (k1.p + 2*k2.p + 2*k3.p + k4.p);
    state.a =  (k1.v + 2*k2.v + 2*k3.v + k4.v)/6.0;
    state.v += dt * state.a;
    state.sigma += (dt / 6.0) * (k1.sigma + 2*k2.sigma + 2*k3.sigma + k4.sigma);
    state.omega += (dt / 6.0) * (k1.omega + 2*k2.omega + 2*k3.omega + k4.omega);

    // ★★★ MRP 核心骚操作：Shadowing 切换，防止奇异性 ★★★
    if (state.sigma.norm() > 1.0) {
        state.sigma = -state.sigma / state.sigma.squaredNorm();
    }

    // 顺便让所有效应器也前进一步 (比如电机的角速度积分)
    for (auto* eff : effectors) {
        eff->computeDerivatives(t, dt);
    }
}

// 辅助函数：用于 RK4 内部状态累加
RigidBodyState RigidBody::add_states(const RigidBodyState& s, const RigidBodyState& dot, double dt) {
    RigidBodyState res;
    res.p = s.p + dot.p * dt;
    res.v = s.v + dot.v * dt;
    res.sigma = s.sigma + dot.sigma * dt;
    res.omega = s.omega + dot.omega * dt;
    return res;
}