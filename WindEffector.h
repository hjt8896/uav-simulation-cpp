//
// Created by hujiet on 2026/6/17.
//

#ifndef AIRCRAFT_SIM_WINDEFFECTOR_H
#define AIRCRAFT_SIM_WINDEFFECTOR_H


#pragma once
#include "BaseEffector.h"

class WindEffector : public BaseEffector {
private:
    Eigen::Vector3d wind_force_B;   // 阵风带来的平移力 (机体系)
    Eigen::Vector3d wind_torque_B;  // 阵风带来的旋转力矩 (机体系)
    double start_time;              // 阵风开始时间
    double end_time;                // 阵风结束时间

public:
    // 构造函数：默认无扰动，时间窗口为无穷小
    WindEffector(double t_start = 0.0, double t_end = 0.0,
                 Eigen::Vector3d force = Eigen::Vector3d::Zero(),
                 Eigen::Vector3d torque = Eigen::Vector3d::Zero())
        : start_time(t_start), end_time(t_end), wind_force_B(force), wind_torque_B(torque) {}

    // 留一个接口，允许外部（比如气象模拟节点）实时修改扰动大小
    void set_disturbance(const Eigen::Vector3d& force, const Eigen::Vector3d& torque) {
        wind_force_B = force;
        wind_torque_B = torque;
    }

    // 重新设置起止时间
    void set_time_window(double t_start, double t_end) {
        start_time = t_start;
        end_time = t_end;
    }

    // 核心接口：向 RigidBody 主体提交扰动贡献
    void updateContributions(double time, BackSubContributions& contrib, const RigidBodyState& state) override {
        if (time >= start_time && time < end_time) {
            // 在时间窗口内，把风力和风力矩叠加上去
            contrib.vecTrans += wind_force_B;
            contrib.vecRot += wind_torque_B;
        }
    }

    // 扰动模型目前没有自身的一阶滞后等内部状态，所以导数更新为空
    void computeDerivatives(double time, double dt) override {
        // Future Work: 如果风速是一个一阶马尔可夫过程(Dryden风场)，可以在这里积分风的状态
    }
};


#endif //AIRCRAFT_SIM_WINDEFFECTOR_H