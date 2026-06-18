#ifndef AIRCRAFT_SIM_ATTITUDEUKF_H
#define AIRCRAFT_SIM_ATTITUDEUKF_H

#pragma once
#include "eigen3/Eigen/Dense"
#include <vector>
#include <cmath>

class AttitudeUKF {
private:
    static constexpr int n = 9;             // 状态维度
    static constexpr int n_sigma = 2 * n + 1; // 19 个 Sigma 点

    // 算法超参数
    double alpha = 1e-3;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda, gamma;

    Eigen::VectorXd wM;
    Eigen::VectorXd wC;

    // 当前最优状态与协方差平方根
    Eigen::VectorXd x_hat;
    Eigen::MatrixXd S;

    Eigen::MatrixXd X_sigma; // 当前 Sigma 点
    Eigen::MatrixXd S_Q;     // 过程噪声平方根
    Eigen::MatrixXd Y_sigma;
    // --- 独立传感器的本底观测噪声平方根 ---
    Eigen::Matrix3d S_R_mag;
    Eigen::Matrix3d S_R_gyro; // ★ 新增：陀螺仪观测噪声平方根

    // 核心算法函数
    static void cholDownDate(Eigen::MatrixXd& S, const Eigen::VectorXd& x, double w);
    static Eigen::VectorXd compute_derivatives(const Eigen::VectorXd& x);
    static Eigen::VectorXd system_dynamics(const Eigen::VectorXd& state_in, double dt);

    // // 提取公共的 MRP 转 DCM 矩阵逻辑
    // static Eigen::Matrix3d mrp_to_dcm(const Eigen::Vector3d& sigma);

    // 分离的观测方程
    static Eigen::Vector3d measurement_model_accel(const Eigen::VectorXd& x);
    static Eigen::Vector3d measurement_model_mag(const Eigen::VectorXd& x);
    static Eigen::Vector3d measurement_model_gyro(const Eigen::VectorXd& x); // ★ 新增

    // ★ 抽取出来的通用测量更新引擎
    void measurement_update_core(const Eigen::Vector3d& Z_actual, const Eigen::MatrixXd& Z_sigma, const Eigen::Matrix3d& S_R_curr);

public:
    AttitudeUKF();
    void generateSigmaPoints();
    void setProcessNoise(const Eigen::MatrixXd& Q);
    void predict(double dt);

    // ★ 拆分后的序贯更新接口
    void update_accel(const Eigen::Vector3d& Z_acc);
    void update_mag(const Eigen::Vector3d& Z_mag);
    void update_gyro(const Eigen::Vector3d& Z_gyro); // ★ 新增

    // 获取当前姿态供外部使用
    [[nodiscard]] Eigen::Vector3d get_mrp() const { return x_hat.segment<3>(0); }
    [[nodiscard]] Eigen::Vector3d get_omega() const { return x_hat.segment<3>(3); }
};

#endif //AIRCRAFT_SIM_ATTITUDEUKF_H