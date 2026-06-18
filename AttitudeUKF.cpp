//
// Created by hujiet on 2026/6/17.
//

#include "AttitudeUKF.h"

AttitudeUKF::AttitudeUKF() {
    x_hat = Eigen::VectorXd::Zero(n);
    S = Eigen::MatrixXd::Identity(n, n) * 0.1; // 初始误差平方根

    wM = Eigen::VectorXd::Zero(n_sigma);
    wC = Eigen::VectorXd::Zero(n_sigma);
    X_sigma = Eigen::MatrixXd::Zero(n, n_sigma);

    // 1. 初始化 Lambda 和 Gamma (完美对齐 Basilisk 源码)
    lambda = alpha * alpha * (n + kappa) - n;
    gamma = std::sqrt(n + lambda);

    // 2. 初始化权重向量
    wM(0) = lambda / (n + lambda);
    wC(0) = wM(0) + (1.0 - alpha * alpha + beta);

    for (int i = 1; i < n_sigma; ++i) {
        wM(i) = 1.0 / (2.0 * (n + lambda));
        wC(i) = wM(i);
    }
}

// --- 核心方法：生成 Sigma 点 ---
void AttitudeUKF::generateSigmaPoints() {
    // 第一列：就是当前状态中心点
    X_sigma.col(0) = x_hat;

    // 后面 18 列：向 S 矩阵的各个特征方向展开
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd offset = gamma * S.col(i);
        X_sigma.col(i + 1)     = x_hat + offset;
        X_sigma.col(i + 1 + n) = x_hat - offset;
    }
}

// 辅助函数：模拟 Basilisk 里的 ukfCholDownDate
// Eigen 的 LLT 官方原生支持正向 rankUpdate，对于负权重的 downdate
// 工业界通常使用 Givens 旋转手写以保证绝对的数值稳定。
// 核心算法：Cholesky 降维更新 (Rank-1 Downdate)
// 目标：计算 S_new，使得 S_new * S_new^T = S * S^T - W * x * x^T
// 输入：S (下三角协方差平方根), x_in (要减去的特征向量), W (那个巨大的负权重)
void AttitudeUKF::cholDownDate(Eigen::MatrixXd& S, const Eigen::VectorXd& x_in, double W) {

    // 1. 提取权重的绝对值，并把权重融合进特征向量 x 中
    // 因为 W 在这里是中心点的负权重，我们取绝对值进行降维计算
    double w_abs = std::abs(W);
    Eigen::VectorXd x = std::sqrt(w_abs) * x_in;

    int n_dim = S.rows();

    // 2. 依次对每一个维度进行 Givens 旋转
    for (int i = 0; i < n_dim; ++i) {

        // 提取当前对角线元素
        double s_ii = S(i, i);
        double x_i = x(i);

        // 安全性检查：如果挖的沙子比原有的还多，说明矩阵濒临非正定
        // 在实际飞控中，通常会强行卡死或者稍微补一点极小的正数来保命
        double r_sq = s_ii * s_ii - x_i * x_i;
        if (r_sq <= 1e-8) {
            r_sq = 1e-8; // 终极防炸机补丁
        }

        // 计算新的对角线元素 (也就是旋转后的半径)
        double r = std::sqrt(r_sq);
        S(i, i) = r;

        // 计算 Givens 旋转的 cos(c) 和 sin(s)
        double c = r / s_ii;
        double s = x_i / s_ii;

        // 3. 将这个旋转应用到当前列的其余元素上
        for (int j = i + 1; j < n_dim; ++j) {
            double s_ji = S(j, i);
            double x_j = x(j);

            // 更新下三角矩阵 S 的剩余元素
            S(j, i) = (s_ji - s * x_j) / c;

            // ★ 绝杀细节：这里用的不是旧的 s_ji，而是刚刚算出来的、全新的 S(j, i)！
            // 这样就彻底避开了除法，防止数值爆炸！
            x(j) = c * x_j - s * S(j, i);
        }
    }
}

// --- 核心物理方程：计算导数 X_dot = f(X) ---
Eigen::VectorXd AttitudeUKF::compute_derivatives(const Eigen::VectorXd& x) {
    // 1. 从 9 维状态中拆包解构
    Eigen::Vector3d sigma = x.segment<3>(0);
    Eigen::Vector3d omega = x.segment<3>(3);
    Eigen::Vector3d bias  = x.segment<3>(6);

    // 2. 计算 MRP 运动学矩阵 B(sigma)
    double sigma_sq = sigma.squaredNorm();
    Eigen::Matrix3d sigma_cross;
    sigma_cross <<     0.0,  -sigma(2),   sigma(1),
                  sigma(2),        0.0,  -sigma(0),
                 -sigma(1),   sigma(0),        0.0;

    Eigen::Matrix3d B = (1.0 - sigma_sq) * Eigen::Matrix3d::Identity()
                      + 2.0 * sigma_cross
                      + 2.0 * sigma * sigma.transpose();

    // 3. 计算三组导数
    Eigen::Vector3d sigma_dot = 0.25 * B * omega;
    Eigen::Vector3d omega_dot = Eigen::Vector3d::Zero(); // 短期恒定假设

    double T_c = 100.0; // FOGM 相关时间
    Eigen::Vector3d bias_dot = -(1.0 / T_c) * bias;

    // 4. 打包返回 9 维导数
    Eigen::VectorXd x_dot(n);
    x_dot << sigma_dot, omega_dot, bias_dot;
    return x_dot;
}

 // 初始化时设定过程噪声
void AttitudeUKF::setProcessNoise(const Eigen::MatrixXd& Q) {
    // 对 Q 进行 Cholesky 分解得到平方根 S_Q
    Eigen::LLT<Eigen::MatrixXd> lltOfQ(Q);
    S_Q = lltOfQ.matrixL();
    Y_sigma = Eigen::MatrixXd::Zero(n, n_sigma);
}

// ★ 终极核心：时间更新 (预测步)
// 对应 Basilisk 源码的 sunlineUKFTimeUpdate
void AttitudeUKF::predict(double dt) {

    // 1. 撒网：生成 19 个 Sigma 点
    generateSigmaPoints();
    Eigen::VectorXd x_bar = Eigen::VectorXd::Zero(n);
    // 2. 让子弹飞：将 19 个点全部塞进物理方程里推演 1ms
    for (int i = 0; i < n_sigma; ++i) {
        // system_dynamics() 是我们接下来要写的非线性状态转移方程
        Y_sigma.col(i) = system_dynamics(X_sigma.col(i), dt);
    }

    // 3. 收网 (均值)：计算先验状态估计 x_bar
    for (int i = 0; i < n_sigma; ++i) {
        x_bar += wM(i) * Y_sigma.col(i);
    }

    // 4. 收网 (协方差平方根)：SR-UKF 核心矩阵重构
    // 完美复刻 Basilisk 的 AT 矩阵构造
    // M 矩阵维度：(2n + n) 行，n 列
    Eigen::MatrixXd M((2 * n) + n, n);

    // 4.1 填入周围 18 个点的误差 (注意 wC 是正数，可以直接开根号)
    for (int i = 1; i <= 2 * n; ++i) {
        M.row(i - 1) = std::sqrt(wC(i)) * (Y_sigma.col(i) - x_bar).transpose();
    }

    // 4.2 将过程噪声平方根 S_Q 压入最后 n 行
    M.bottomRows(n) = S_Q.transpose();

    // 4.3 极其优雅的 QR 分解 (只有 2 行代码！)
    // 对 M 执行 QR 分解，提取出上三角矩阵 R
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(M);
    Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();

    // 我们需要的协方差平方根 S_bar 就是 R 矩阵顶部的 n x n 块
    Eigen::MatrixXd S_bar = R.block(0, 0, n, n).transpose();
    // (注：UKF文献中常取下三角，所以做了个转置)

    // 5. 扣除中心点的负权重：Cholesky 降维更新 (Downdate)
    // 对应 Basilisk 源码里的 ukfCholDownDate
    Eigen::VectorXd err_0 = Y_sigma.col(0) - x_bar;
    cholDownDate(S_bar, err_0, wC(0)); // wC(0) 是那个巨大的负数

    // 6. 更新系统状态，准备进入观测更新
    x_hat = x_bar;
    S = S_bar;
}

// 留给你的物理引擎接口
Eigen::VectorXd AttitudeUKF::system_dynamics(const Eigen::VectorXd& state_in, const double dt) {
    // 使用 RK4 (四阶龙格-库塔法) 进行极其精准的数值积分
    const Eigen::VectorXd k1 = compute_derivatives(state_in);
    const Eigen::VectorXd k2 = compute_derivatives(state_in + 0.5 * dt * k1);
    const Eigen::VectorXd k3 = compute_derivatives(state_in + 0.5 * dt * k2);
    const Eigen::VectorXd k4 = compute_derivatives(state_in + dt * k3);

    // 返回积分后的下一个状态
    return state_in + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// --- 物理映射：提取公共的 MRP 转 DCM ---
Eigen::Matrix3d AttitudeUKF::mrp_to_dcm(const Eigen::Vector3d& sigma) {
    double sigma_sq = sigma.squaredNorm();
    double den = 1.0 + sigma_sq;
    double den_sq = den * den;

    Eigen::Matrix3d sigma_cross;
    sigma_cross <<     0.0,  -sigma(2),   sigma(1),
                  sigma(2),        0.0,  -sigma(0),
                 -sigma(1),   sigma(0),        0.0;

    return Eigen::Matrix3d::Identity()
         - (4.0 * (1.0 - sigma_sq) / den_sq) * sigma_cross
         + (8.0 / den_sq) * (sigma_cross * sigma_cross);
}

// --- 分离的观测方程 ---
Eigen::Vector3d AttitudeUKF::measurement_model_accel(const Eigen::VectorXd& x) {
    Eigen::Matrix3d R_NB = mrp_to_dcm(x.segment<3>(0));
    Eigen::Vector3d f_N(0.0, 0.0, -9.81); // 桌面静止时，感受到的比力向上
    return R_NB * f_N;
}

Eigen::Vector3d AttitudeUKF::measurement_model_mag(const Eigen::VectorXd& x) {
    Eigen::Matrix3d R_NB = mrp_to_dcm(x.segment<3>(0));
    Eigen::Vector3d mag_N(0.22, 0.0, 0.45); // 地磁参考场 (随地理位置定)
    return R_NB * mag_N;
}

// ★ 极其性感的通用底层测量更新引擎
void AttitudeUKF::measurement_update_core(const Eigen::Vector3d& Z_actual, const Eigen::MatrixXd& Z_sigma, const Eigen::Matrix3d& S_R_curr) {
    int m = 3;

    // 1. 计算预测观测均值
    Eigen::Vector3d z_hat = Eigen::Vector3d::Zero();
    for (int i = 0; i < n_sigma; ++i) {
        z_hat += wM(i) * Z_sigma.col(i);
    }

    // 2. 构建 S_y 并做 QR 分解与 Downdate
    Eigen::MatrixXd M_meas((2 * n) + m, m);
    for (int i = 1; i <= 2 * n; ++i) {
        M_meas.row(i - 1) = std::sqrt(wC(i)) * (Z_sigma.col(i) - z_hat).transpose();
    }
    M_meas.bottomRows(m) = S_R_curr.transpose();

    Eigen::HouseholderQR<Eigen::MatrixXd> qr_meas(M_meas);
    Eigen::MatrixXd R_meas = qr_meas.matrixQR().triangularView<Eigen::Upper>();
    Eigen::MatrixXd S_y = R_meas.block(0, 0, m, m).transpose();

    Eigen::Vector3d z_err_0 = Z_sigma.col(0) - z_hat;
    cholDownDate(S_y, z_err_0, wC(0));

    // 3. 计算互协方差 P_xz (注意：使用当前的 X_sigma 和 x_hat！)
    Eigen::MatrixXd P_xz = Eigen::MatrixXd::Zero(n, m);
    for (int i = 0; i < n_sigma; ++i) {
        P_xz += wC(i) * (X_sigma.col(i) - x_hat) * (Z_sigma.col(i) - z_hat).transpose();
    }

    // 4. 计算卡尔曼增益 K (LLT求解保证稳定)
    Eigen::MatrixXd P_zz = S_y * S_y.transpose();
    Eigen::MatrixXd K = P_xz * P_zz.llt().solve(Eigen::Matrix3d::Identity());

    // 5. 状态更新
    x_hat = x_hat + K * (Z_actual - z_hat);

    // 6. U-Matrix 连续降维更新协方差
    Eigen::MatrixXd U = K * S_y;
    for (int i = 0; i < m; ++i) {
        cholDownDate(S, U.col(i), -1.0);
    }
}

// --- 接口：加速度计自适应序贯更新 ---
void AttitudeUKF::update_accel(const Eigen::Vector3d& Z_acc) {
    // 1. 基于当前最新状态(或predict的先验)重新生成 19 个 Sigma 点
    generateSigmaPoints();

    // 2. 自适应观测噪声 (AKF)
    double acc_norm = Z_acc.norm();
    double error = std::abs(acc_norm - 9.81);
    double base_noise = 0.5; // 基础噪声
    double adaptive_noise = (error < 0.5) ? base_noise : (base_noise + 15.0 * error);
    Eigen::Matrix3d S_R_acc = Eigen::Matrix3d::Identity() * adaptive_noise;

    // 3. 预测观测映射
    Eigen::MatrixXd Z_sigma(3, n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        Z_sigma.col(i) = measurement_model_accel(X_sigma.col(i));
    }

    // 4. 调用底层核心引擎
    measurement_update_core(Z_acc, Z_sigma, S_R_acc);
}

// --- 接口：罗盘序贯更新 ---
void AttitudeUKF::update_mag(const Eigen::Vector3d& Z_mag) {
    // 1. 如果在同一毫秒内刚做了 accel 更新，这里的 generate 会基于 accel 修正后的新状态再撒点！
    generateSigmaPoints();

    // 2. 预测观测映射
    Eigen::MatrixXd Z_sigma(3, n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        Z_sigma.col(i) = measurement_model_mag(X_sigma.col(i));
    }

    // 3. 假设罗盘初始化时给了固定的 S_R_mag (比如 0.1)
    if (S_R_mag.isZero()) { S_R_mag = Eigen::Matrix3d::Identity() * 0.1; }

    // 4. 调用底层核心引擎
    measurement_update_core(Z_mag, Z_sigma, S_R_mag);
}

// --- 陀螺仪观测方程 ---
Eigen::Vector3d AttitudeUKF::measurement_model_gyro(const Eigen::VectorXd& x) {
    // 陀螺仪的读数预期 = 真实的机体角速度(索引3-5) + 陀螺仪零偏(索引6-8)
    return x.segment<3>(3) + x.segment<3>(6);
}

// --- 接口：陀螺仪序贯更新 ---
void AttitudeUKF::update_gyro(const Eigen::Vector3d& Z_gyro) {
    // 1. 基于当前最新状态重新生成 19 个 Sigma 点
    generateSigmaPoints();

    // 2. 预测观测映射
    Eigen::MatrixXd Z_sigma(3, n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        Z_sigma.col(i) = measurement_model_gyro(X_sigma.col(i));
    }

    // 3. 初始化并设定陀螺仪的固定观测噪声
    // 陀螺仪通常很准，噪声设为一个较小的值，例如 0.01 rad/s
    if (S_R_gyro.isZero()) { S_R_gyro = Eigen::Matrix3d::Identity() * 0.01; }

    // 4. 召唤底层核心引擎，执行 QR 分解与 Cholesky 降维修正
    measurement_update_core(Z_gyro, Z_sigma, S_R_gyro);
}