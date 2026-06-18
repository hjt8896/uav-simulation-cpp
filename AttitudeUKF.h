//
// Created by hujiet on 2026/6/17.
//

#ifndef AIRCRAFT_SIM_ATTITUDEUKF_H
#define AIRCRAFT_SIM_ATTITUDEUKF_H


#pragma once
#include "eigen3/Eigen/Dense"
#include <vector>
#include <cmath>

class AttitudeUKF {
private:
    static const int n = 9;             // 状态维度
    static const int n_sigma = 2 * n + 1; // 19 个 Sigma 点

    // 算法超参数 (对应 Basilisk 源码里的 alpha, beta, kappa)
    double alpha = 1e-3;
    double kappa = 0.0;
    double beta = 2.0;
    double lambda, gamma;

    // 权重向量
    Eigen::VectorXd wM;
    Eigen::VectorXd wC;

    // 核心状态与协方差的平方根 (Cholesky 因子)
    Eigen::VectorXd x_hat;
    Eigen::MatrixXd S; // 即 Basilisk 里的 sBar， P = S * S^T

    // Sigma 点矩阵 (9 行 x 19 列)
    Eigen::MatrixXd X_sigma;

    Eigen::MatrixXd S_Q; // 过程噪声矩阵 Q 的平方根 (Cholesky 因子)
    Eigen::MatrixXd Y_sigma; // 存放 19 个 Sigma 点经过物理预测后的新位置

    Eigen::VectorXd x_bar;
    // S_R 是传感器观测噪声矩阵 R 的平方根 (对角阵，包含 acc 和 mag 的白噪声标准差)
    Eigen::MatrixXd S_R;

    static void cholDownDate(Eigen::MatrixXd& S, const Eigen::VectorXd& x, double w);
    static Eigen::VectorXd compute_derivatives(const Eigen::VectorXd& x);
    static Eigen::VectorXd system_dynamics(const Eigen::VectorXd& state_in, double dt);
    // --- 核心观测方程：Z = h(X) ---
    static Eigen::VectorXd measurement_model(const Eigen::VectorXd& x);
public:
    AttitudeUKF() ;
    // --- 核心方法：生成 Sigma 点 ---
    void generateSigmaPoints() ;
    // 初始化时设定过程噪声
    void setProcessNoise(const Eigen::MatrixXd& Q);
    void predict(double dt);
    void update(const Eigen::VectorXd& Z_actual);
};


#endif //AIRCRAFT_SIM_ATTITUDEUKF_H