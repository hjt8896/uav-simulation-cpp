//
// Created by hujiet on 2026/6/16.
//

#ifndef AIRCRAFT_SIM_COMMONTYPES_H
#define AIRCRAFT_SIM_COMMONTYPES_H
#include "eigen3/Eigen/Dense"

// 系统的状态向量
struct RigidBodyState {
    Eigen::Vector3d p;      // 惯性系位置
    Eigen::Vector3d v;      // 惯性系速度
    Eigen::Vector3d sigma;  // MRP 姿态 (Body to Inertial)
    Eigen::Vector3d omega;  // 机体坐标系下的角速度
};


#endif //AIRCRAFT_SIM_COMMONTYPES_H