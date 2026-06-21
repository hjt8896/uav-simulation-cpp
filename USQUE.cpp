//
// Created by hujiet on 2026/6/21.
//

#include "USQUE.h"

USQUE::USQUE() {
    // ==========================================
    q_ref = Eigen::Vector4d(1.0,0.0,0.0,0.0);
    // 安全第一：无论外部给的四元数有多准，进来第一件事先强行归一化，
    // 防止极其微小的浮点截断误差在后续乘法中被放大。
    q_ref.normalize();

    omega_ref = Eigen::Vector3d::Zero();
    bias_ref = Eigen::Vector3d::Zero();
    error_x = Eigen::VectorXd::Zero(n);
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

// 初始化时设定过程噪声
void USQUE::setProcessNoise(const Eigen::MatrixXd& Q) {
    // 对 Q 进行 Cholesky 分解得到平方根 S_Q
    Eigen::LLT<Eigen::MatrixXd> lltOfQ(Q);
    S_Q = lltOfQ.matrixL();
    Y_sigma = Eigen::MatrixXd::Zero(n, n_sigma);
}
// --- 核心方法：生成 Sigma 点 ---
void USQUE::generateSigmaPoints() {
    // 第一列：就是当前状态中心点
    X_sigma.col(0) = error_x;

    // 后面 18 列：向 S 矩阵的各个特征方向展开
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd offset = gamma * S.col(i);
        X_sigma.col(i + 1)     = error_x + offset;
        X_sigma.col(i + 1 + n) = error_x - offset;
    }
}

// 辅助函数：模拟 Basilisk 里的 ukfCholDownDate
// Eigen 的 LLT 官方原生支持正向 rankUpdate，对于负权重的 downdate
// 工业界通常使用 Givens 旋转手写以保证绝对的数值稳定。
// 核心算法：Cholesky 降维更新 (Rank-1 Downdate)
// 目标：计算 S_new，使得 S_new * S_new^T = S * S^T - W * x * x^T
// 输入：S (下三角协方差平方根), x_in (要减去的特征向量), W (那个巨大的负权重)
void USQUE::cholDownDate(Eigen::MatrixXd& S, const Eigen::VectorXd& x_in, double W) {

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

void USQUE::inject_and_reset() {
    // 1. 拆解最优误差状态
    Eigen::Vector3d delta_sigma = error_x.segment<3>(0);
    Eigen::Vector3d delta_omega = error_x.segment<3>(3);
    Eigen::Vector3d delta_bias  = error_x.segment<3>(6);

    // 2. 将误差 MRP 转换为误差四元数 (调用咱们刚推导的 MathUtils)
    Eigen::Vector4d delta_q = MathUtils::mrp_to_quaternion(delta_sigma);

    // 3. 注入到名义状态 (四元数用乘法，其他用加法)
    // 注意：这里的乘法顺序 delta_q * q_ref 代表局部扰动，非常重要！
    q_ref = MathUtils::quat_mult(delta_q, q_ref);
    omega_ref += delta_omega;
    bias_ref  += delta_bias;

    // 4. ★ 强行归一化名义四元数，防止长期的微小浮点误差积累
    q_ref.normalize();

    // 5. ★ 卸磨杀驴：清零 UKF 内层误差状态 (协方差 S 予以保留)
    error_x.setZero();
}

// 计算绝对状态的导数 [q_dot(4), omega_dot(3), bias_dot(3)]
Eigen::VectorXd USQUE::absolute_kinematics(const Eigen::Vector4d& q, const Eigen::Vector3d& omega, const Eigen::Vector3d& bias) {
    // 这里我们用一个 10 维向量打包导数返回
    Eigen::VectorXd dot = Eigen::VectorXd::Zero(10);

    // ==========================================
    // 1. 四元数运动学 (极简的魔法)
    // q_dot = 0.5 * q * [0, omega]^T
    // ==========================================
    Eigen::Vector4d omega_quat;
    omega_quat << 0.0, omega; // 拼装纯矢量四元数
    Eigen::Vector4d q_dot = 0.5 * MathUtils::quat_mult(q, omega_quat);

    // ==========================================
    // 2. 角速度动力学 (Euler's Equation)
    // ==========================================
    // 由于你现在的框架是估计器，我们和之前一样做“短期恒定假设”
    // 如果后续你需要接入控制力矩和 Mujoco 的转动惯量 J，
    // 直接在这里写: omega_dot = J_inv * (Torque - omega.cross(J * omega))
    Eigen::Vector3d omega_dot = Eigen::Vector3d::Zero();

    // ==========================================
    // 3. 陀螺仪零偏动力学
    // ==========================================
    // 一阶马尔可夫过程 (FOGM)，T_c 为相关时间
    double T_c = 100.0;
    Eigen::Vector3d bias_dot = -(1.0 / T_c) * bias;

    // 打包结果
    dot.segment<4>(0) = q_dot;
    dot.segment<3>(4) = omega_dot;
    dot.segment<3>(7) = bias_dot;

    return dot;
}

void USQUE::rk4_absolute_integration(Eigen::Vector4d& q, Eigen::Vector3d& omega, Eigen::Vector3d& bias, double dt) {
    // 将当前状态打包
    Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
    state << q, omega, bias;

    // RK4 核心四步走
    // k1
    Eigen::VectorXd k1 = absolute_kinematics(q, omega, bias);

    // k2
    Eigen::VectorXd state_k2 = state + 0.5 * dt * k1;
    Eigen::VectorXd k2 = absolute_kinematics(state_k2.segment<4>(0), state_k2.segment<3>(4), state_k2.segment<3>(7));

    // k3
    Eigen::VectorXd state_k3 = state + 0.5 * dt * k2;
    Eigen::VectorXd k3 = absolute_kinematics(state_k3.segment<4>(0), state_k3.segment<3>(4), state_k3.segment<3>(7));

    // k4
    Eigen::VectorXd state_k4 = state + dt * k3;
    Eigen::VectorXd k4 = absolute_kinematics(state_k4.segment<4>(0), state_k4.segment<3>(4), state_k4.segment<3>(7));

    // 组合加权，完成时间演化
    state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // 重新拆包写回变量
    q = state.segment<4>(0);
    omega = state.segment<3>(4);
    bias = state.segment<3>(7);

    // ★ 终极保命符：将积分后的绝对四元数强制拉回单位球面！
    // 因为这里是绝对物理空间，所以直接归一化是合法的、且必须的。
    q.normalize();
}

void USQUE::predict(double dt) {
    // 此时 error_x 绝对是 0，我们以此为中心撒出 19 个极小的误差 Sigma 点
    generateSigmaPoints(); // X_sigma 的第 0 列是 0，后面是 ±gamma * S

    // 预分配数组，用来存 19 个点跑完积分后的【新绝对状态】
    std::vector<Eigen::Vector4d> q_next_abs(n_sigma);
    std::vector<Eigen::Vector3d> omega_next_abs(n_sigma);
    std::vector<Eigen::Vector3d> bias_next_abs(n_sigma);

    // ==========================================
    // 阶段 1：流形展开 (误差 -> 绝对 -> 积分)
    // ==========================================
    for (int i = 0; i < n_sigma; ++i) {
        // 1. 把误差映射到绝对空间
        Eigen::Vector4d delta_q_i = MathUtils::mrp_to_quaternion(X_sigma.col(i).segment<3>(0));
        Eigen::Vector4d q_abs_i = MathUtils::quat_mult(delta_q_i, q_ref);
        Eigen::Vector3d omega_abs_i = omega_ref + X_sigma.col(i).segment<3>(3);
        Eigen::Vector3d bias_abs_i = bias_ref + X_sigma.col(i).segment<3>(6);

        // 2. 将绝对状态塞进 RK4 跑物理方程 (纯粹的绝对运动学和动力学)
        rk4_absolute_integration(q_abs_i, omega_abs_i, bias_abs_i, dt);

        // 3. 暂存积分后的绝对状态
        q_next_abs[i] = q_abs_i;
        omega_next_abs[i] = omega_abs_i;
        bias_next_abs[i] = bias_abs_i;
    }

    // ==========================================
    // 阶段 2：名义状态自积分
    // ==========================================
    Eigen::Vector4d q_ref_next = q_ref;
    Eigen::Vector3d omega_ref_next = omega_ref;
    Eigen::Vector3d bias_ref_next = bias_ref;
    rk4_absolute_integration(q_ref_next, omega_ref_next, bias_ref_next, dt);

    // ==========================================
    // 阶段 3：流形收缩 (新绝对状态 -> 新名义状态的误差空间)
    // ==========================================
    Eigen::MatrixXd Y_sigma_error = Eigen::MatrixXd::Zero(n, n_sigma);
    Eigen::Vector4d q_ref_next_inv = MathUtils::quat_inverse(q_ref_next);

    for (int i = 0; i < n_sigma; ++i) {
        // 算相对旋转: delta_q = q_actual * q_ref^{-1}
        Eigen::Vector4d dq_next = MathUtils::quat_mult(q_next_abs[i], q_ref_next_inv);

        // ★ 调用带最短路径保护的 quaternion_to_mrp
        Y_sigma_error.col(i).segment<3>(0) = MathUtils::quaternion_to_mrp(dq_next);
        Y_sigma_error.col(i).segment<3>(3) = omega_next_abs[i] - omega_ref_next;
        Y_sigma_error.col(i).segment<3>(6) = bias_next_abs[i] - bias_ref_next;
    }

    // ==========================================
    // 阶段 4：算均值与协方差，并更新老大哥
    // ==========================================
    // 算均值 (在极小、极平坦的误差空间里做欧式加法，极其安全)
    Eigen::VectorXd x_bar = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n_sigma; ++i) { x_bar += wM(i) * Y_sigma_error.col(i); }

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

    // 更新状态缓存
    error_x = x_bar;
    S = S_bar;
    q_ref = q_ref_next;
    omega_ref = omega_ref_next;
    bias_ref = bias_ref_next;

    // ★ 绝杀：预测步算出了非零的先验误差，立刻注入名义状态并清零！
    inject_and_reset();
}

// ★ 极其性感的通用底层测量更新引擎 (USQUE 版本)
void USQUE::measurement_update_core(const Eigen::Vector3d& Z_actual, const Eigen::MatrixXd& Z_sigma, const Eigen::Matrix3d& S_R_curr) {
    int m = 3; // 观测维度

    // 1. 计算预测观测均值
    Eigen::Vector3d z_hat = Eigen::Vector3d::Zero();
    for (int i = 0; i < n_sigma; ++i) {
        z_hat += wM(i) * Z_sigma.col(i);
    }

    // 2. 构建 S_y 并做 QR 分解与 Downdate
    // 这部分完全保留你原汁原味的 Basilisk 级稳定算法
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

    // 3. 计算互协方差 P_xz
    // 注意：此时的 X_sigma 里面装的全是极小的误差状态！
    Eigen::MatrixXd P_xz = Eigen::MatrixXd::Zero(n, m);
    for (int i = 0; i < n_sigma; ++i) {
        P_xz += wC(i) * (X_sigma.col(i) - error_x) * (Z_sigma.col(i) - z_hat).transpose();
    }

    // 4. 计算卡尔曼增益 K
    Eigen::MatrixXd P_zz = S_y * S_y.transpose();
    Eigen::MatrixXd K = P_xz * P_zz.llt().solve(Eigen::Matrix3d::Identity());

    // 5. 状态更新 (算出最优后验误差)
    error_x = error_x + K * (Z_actual - z_hat);

    // 6. U-Matrix 连续降维更新协方差 S
    Eigen::MatrixXd U = K * S_y;
    for (int i = 0; i < m; ++i) {
        cholDownDate(S, U.col(i), -1.0);
    }

    // 7. ★ 终极绝杀：将算出的后验误差注入外层名义四元数，并瞬间清零！
    // 经历了上面极其暴力的矩阵降维运算后，协方差 S 已经变小(变准)了，
    // 而均值 error_x 的价值已经榨干，归零准备下一轮。
    inject_and_reset();
}

// --- 接口：加速度计自适应序贯更新 ---
void USQUE::update_accel(const Eigen::Vector3d& Z_acc) {
    // 1. 基于当前最新状态(或predict的先验)重新生成 19 个 Sigma 点
    generateSigmaPoints();

    // 2. 自适应观测噪声 (AKF)
    double acc_norm = Z_acc.norm();
    double error = std::abs(acc_norm - 9.81);
    double base_noise = 0.5; // 基础噪声
    double adaptive_noise = (error < 0.5) ? base_noise : (base_noise * exp(error));
    Eigen::Matrix3d S_R_acc = Eigen::Matrix3d::Identity() * adaptive_noise;

    // 3. 预测观测映射
    Eigen::MatrixXd Z_sigma(3, n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        // 1. 拆解 Sigma 点中的误差
        Eigen::Vector3d delta_sigma = X_sigma.col(i).segment<3>(0);

        // 2. 误差空间 -> 绝对空间
        Eigen::Vector4d delta_q = MathUtils::mrp_to_quaternion(delta_sigma);
        Eigen::Vector4d q_abs_i = MathUtils::quat_mult(delta_q, q_ref);
        Z_sigma.col(i) = measurement_model_accel(q_abs_i);
    }

    // 4. 调用底层核心引擎
    measurement_update_core(Z_acc, Z_sigma, S_R_acc);

}

// --- 接口：罗盘序贯更新 ---
void USQUE::update_mag(const Eigen::Vector3d& Z_mag) {
    // 1. 如果在同一毫秒内刚做了 accel 更新，这里的 generate 会基于 accel 修正后的新状态再撒点！
    generateSigmaPoints();

    // 2. 预测观测映射
    Eigen::MatrixXd Z_sigma(3, n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        // 1. 拆解 Sigma 点中的误差
        Eigen::Vector3d delta_sigma = X_sigma.col(i).segment<3>(0);

        // 2. 误差空间 -> 绝对空间
        Eigen::Vector4d delta_q = MathUtils::mrp_to_quaternion(delta_sigma);
        Eigen::Vector4d q_abs_i = MathUtils::quat_mult(delta_q, q_ref);

        Z_sigma.col(i) = measurement_model_mag(q_abs_i);
    }

    // 3. 假设罗盘初始化时给了固定的 S_R_mag (比如 0.1)
    if (S_R_mag.isZero()) { S_R_mag = Eigen::Matrix3d::Identity() * 0.1; }

    // 4. 调用底层核心引擎
    measurement_update_core(Z_mag, Z_sigma, S_R_mag);
}

Eigen::Vector3d USQUE::measurement_model_gyro(const Eigen::Vector3d& omega_abs, const Eigen::Vector3d& bias_abs) {
    return omega_abs + bias_abs;
}

Eigen::Vector3d USQUE::measurement_model_accel(const Eigen::Vector4d& q_abs) {
    Eigen::Matrix3d R_NB = MathUtils::quat_to_dcm(q_abs); // 注意这里用四元数转DCM
    Eigen::Vector3d f_N(0.0, 0.0, -9.81);
    return R_NB * f_N;
}

Eigen::Vector3d USQUE::measurement_model_mag(const Eigen::Vector4d& q_abs) {
    Eigen::Matrix3d R_NB = MathUtils::quat_to_dcm(q_abs);
    Eigen::Vector3d mag_N(0.22, 0.0, 0.45);
    return R_NB * mag_N;
}

// --- 接口：陀螺仪序贯更新 ---
void USQUE::update_gyro(const Eigen::Vector3d& Z_gyro) {
    // 1. 基于当前最新状态重新生成 19 个 Sigma 点
    generateSigmaPoints();

    // 2. 预测观测映射
    Eigen::MatrixXd Z_sigma(3, n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        // 1. 拆解 Sigma 点中的误差
        Eigen::Vector3d delta_omega = X_sigma.col(i).segment<3>(3);
        Eigen::Vector3d delta_bias = X_sigma.col(i).segment<3>(6);
        // 2. 误差空间 -> 绝对空间
        Eigen::Vector3d omega_abs_i = omega_ref+delta_omega;
        Eigen::Vector3d bias_abs_i = bias_ref+delta_bias;
        Z_sigma.col(i) = measurement_model_gyro(omega_abs_i,bias_abs_i);
    }

    // 3. 初始化并设定陀螺仪的固定观测噪声
    // 陀螺仪通常很准，噪声设为一个较小的值，例如 0.01 rad/s
    if (S_R_gyro.isZero()) { S_R_gyro = Eigen::Matrix3d::Identity() * 0.01; }

    // 4. 召唤底层核心引擎，执行 QR 分解与 Cholesky 降维修正
    measurement_update_core(Z_gyro, Z_sigma, S_R_gyro);
}