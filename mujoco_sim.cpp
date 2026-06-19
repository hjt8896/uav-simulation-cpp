//
// Created by hujiet on 2026/6/18.
//
#include <iostream>
#include <fstream>
#include <vector>
#include "mujoco/mujoco.h"
#include "eigen3/Eigen/Dense"
#include <GLFW/glfw3.h> // ★ 召唤 GLFW 窗口管理器

// 引入你的飞控全家桶
#include "AttitudeUKF.h"
#include "SMCController.h"
#include "Mixer.h"
#include "NotchFilter.h"
#include "MathUtils.h"
#include "VelocitySMC.h"
#include "MRPSteering.h"
#include "AltitudeSMC.h"

// ==========================================
// [上帝视角] 全局指针与鼠标事件监听
// ==========================================
mjModel* m = nullptr;
mjvCamera cam;
mjvScene scn;

bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // 只有按住鼠标按键时才移动视角
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;       // 右键平移
    else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V; // 左键旋转
    else action = mjMOUSE_ZOOM;                                                   // 中键缩放

    // 驱动 MuJoCo 摄像机
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam); // 滚轮缩放
}

int main() {
    std::cout << "--- 3D 可视化 SITL 启动 ---" << std::endl;

    // 1. 初始化 GLFW 窗口
    if (!glfwInit()) { std::cerr << "GLFW 初始化失败！" << std::endl; return 1; }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "SR-UKF 飞控可视化", NULL, NULL);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSwapInterval(0); // 开启垂直同步 (大约 60Hz 刷新率)

    // 2. 加载 MuJoCo 模型
    char error[1000] = "Could not load XML model";
     m = mj_loadXML("quadcopter.xml", 0, error, 1000);
    mjData* d = mj_makeData(m);

    static int pos_id    = mj_name2id(m, mjOBJ_SENSOR, "true_pos");
    static int vel_id    = mj_name2id(m, mjOBJ_SENSOR, "true_linvel");
    static int quat_id   = mj_name2id(m, mjOBJ_SENSOR, "true_quat");
    static int angvel_id = mj_name2id(m, mjOBJ_SENSOR, "true_angvel");

    double* p_mj = &d->sensordata[m->sensor_adr[pos_id]];
    double* v_mj = &d->sensordata[m->sensor_adr[vel_id]];
    double* q_mj = &d->sensordata[m->sensor_adr[quat_id]];
    double* w_mj = &d->sensordata[m->sensor_adr[angvel_id]];

    // ★ 3. 初始化 MuJoCo 渲染数据结构

    mjvOption opt;                      // 渲染选项
    mjrContext con;                     // GPU 渲染上下文

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);                // 在内存里构建 3D 场景
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // 初始化 OpenGL 上下文

    // 设定一个帅气的初始相机视角 (俯视侧后方)
    cam.azimuth = 90; cam.elevation = -20; cam.distance = 3.0;
    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 1.0;

    // 2. 初始化你的飞控大脑
    double mass = 1.0;
    Eigen::Matrix3d inertia = Eigen::Vector3d(0.01, 0.01, 0.02).asDiagonal();
    double l = 0.2, k_f = 1e-5, k_m = 2e-7;
    Mixer mixer(l, k_f, k_m, 100.0, 1000.0);

    AltitudeSMC alt_smc(mass);
    MRPSteering steering_real{};
    SMCController smc(inertia);
    AttitudeUKF ukf;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9);
    Q.block<3,3>(0,0) *= 1e-6; Q.block<3,3>(3,3) *= 1e-4; Q.block<3,3>(6,6) *= 1e-8;
    ukf.setProcessNoise(Q);

    // 设定你期望无人机悬停的绝对 3D 空间坐标
    Eigen::Vector3d target_position(0.0, 0.0, -1.0); // 悬停在 1 米高空

    std::ofstream log("mujoco_sitl_log.csv");
    log << "time,target_h,height,mrp_roll,target_mrp_roll,motor1,motor2,motor3,motor4\n";

    std::cout << "引擎点火！无人机正在 1 米高空尝试抵抗重力..." << std::endl;
    std::cout << "3D 视界已打开！准备渲染..." << std::endl;

    // 4. ★ 核心：图形化主循环
    while (!glfwWindowShouldClose(window)) {
        // 计算渲染同步时间
        mjtNum simstart = d->time;
        // 1.0/60.0 意味着每隔 1/60 秒渲染一帧，期间物理引擎疯狂推演！
        while (d->time - simstart < 1.0 / 60.0) {
            double current_time = d->time;

            // ==========================================
            // [A] 传感器海关：从 MuJoCo(ENU) 提取并转换为飞控(FRD)
            // ==========================================
            // MuJoCo XML 里定义了传感器，默认前3个是加速度，后3个是陀螺仪
            Eigen::Vector3d acc_mj(d->sensordata[0], d->sensordata[1], d->sensordata[2]);
            Eigen::Vector3d gyro_mj(d->sensordata[3], d->sensordata[4], d->sensordata[5]);

            // ★ 宇宙翻转法则：X轴不变，Y和Z取反
            Eigen::Vector3d acc_frd(acc_mj(0), -acc_mj(1), -acc_mj(2));
            Eigen::Vector3d gyro_frd(gyro_mj(0), -gyro_mj(1), -gyro_mj(2));


            // freejoint 的前 3 个 DOF 永远是世界系 (ENU) 下的线加速度
            double true_acc_x_mj = d->qacc[0];
            double true_acc_y_mj = d->qacc[1];
            double true_acc_z_mj = d->qacc[2];

            // ==========================================
            // [A.3] 极其致命的坐标系翻转！MuJoCo(ENU) -> 飞控(NED/FRD)
            // ==========================================
            // ★ 海关翻转：MuJoCo(ENU) -> 飞控(NED/FRD)
            // 依然是 X 不变，Y 和 Z 取反
            Eigen::Vector3d current_true_acc(true_acc_x_mj, -true_acc_y_mj, -true_acc_z_mj);
            // X轴不变，Y轴和Z轴必须取反！
            Eigen::Vector3d current_pos(p_mj[0], -p_mj[1], -p_mj[2]);
            Eigen::Vector3d current_vel(v_mj[0], -v_mj[1], -v_mj[2]);
            Eigen::Vector3d current_angvel(w_mj[0], -w_mj[1], -w_mj[2]);

            // 四元数翻转 (qw, qx, qy, qz) -> ENU转NED
            // 四元数的标量 qw 不变，X不变，Y和Z取反
            Eigen::Vector4d current_quat(q_mj[0], q_mj[1], -q_mj[2], -q_mj[3]);
            Eigen::Matrix3d R_NB = MathUtils::QuatToRotationMatrix(current_quat).transpose();
            Eigen::Vector3d acc_true = acc_frd - R_NB * current_true_acc;

            // ==========================================
            // [B] 姿态解算滤波 (UKF)
            // ==========================================
            double dt = m->opt.timestep;
            ukf.predict(dt);
            ukf.update_gyro(gyro_frd);
            ukf.update_accel(acc_true);

            // ==========================================
            // [C]  SMC飞控 (高度和姿态闭环控制)
            // ==========================================
            // 定义两个飞控输出变量，交给内环
            double dynamic_thrust = 0.0;
            Eigen::Vector3d dynamic_target_sigma(0, 0, 0);

            // =====================================
            // 1. Z 轴高度指令生成 (试飞员按键接管)
            // =====================================
            double flight_speed_z = 3.0; // 键盘设定的目标移动速度
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) target_position(2) -= flight_speed_z * dt;
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) target_position(2) += flight_speed_z * dt;

            // =====================================
            // 2. 独立 Z 轴滑模定高控制器 (黑盒调用)
            // =====================================
            double cos_tilt = R_NB(2, 2);
            dynamic_thrust = alt_smc.compute_thrust(current_pos(2), target_position(2), current_vel(2), cos_tilt);

            // =====================================
            // 3. XY 轴人类试飞员接管 (直接映射目标姿态)
            // =====================================
            double max_tilt = 0.25;
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) dynamic_target_sigma(1) = max_tilt;
            else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) dynamic_target_sigma(1) = -max_tilt;
            else dynamic_target_sigma(1) = 0;

            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) dynamic_target_sigma(0) = -max_tilt;
            else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) dynamic_target_sigma(0) = max_tilt;
            else dynamic_target_sigma(0) = 0;

            // ==========================================
            // [内环] 直接接收 dynamic_thrust 和 dynamic_target_sigma
            // ==========================================
            Eigen::Vector3d omega_d(0, 0, 0);
            Eigen::Vector3d omega_d_dot(0, 0, 0);
            Eigen::Vector3d current_mrp = ukf.get_mrp();

            // 姿态环和角速度环照常运作，只是外环指令的来源变了
            steering_real.compute_steering(current_mrp - dynamic_target_sigma, omega_d, omega_d_dot);

            omega_d(2) = 0.0;
            Eigen::Vector3d tau_frd = smc.compute_torque(ukf.get_omega(), omega_d, omega_d_dot, dt);
            // ==========================================
            // [D] 执行器海关：极其严谨的神经重接！
            // ==========================================
            // 按照 Thrust, Roll, Pitch, Yaw 顺序传入期望力矩
            auto speeds = mixer.allocate(dynamic_thrust, tau_frd(0), tau_frd(1), tau_frd(2));

            // 直接按顺序写入 MuJoCo！再也不用在外部做恶心的索引映射了
            for (int i = 0; i < 4; ++i) {
                // 注意写入 XML 的是推力大小 (speeds^2 * k_f) 或者根据你 XML motor 的定义直接给 control 信号
                // 如果你的 XML <motor> 标签自带了 gear，你可能需要根据配置给 speeds，
                // 咱们这里延续你之前的物理公式：
                d->ctrl[i] = speeds[i] * speeds[i] * k_f;
            }

            if (current_time >= 0.5 && current_time <= 0.6) {
                // qfrc_applied 是 MuJoCo 里的外力数组。
                // 对于 freejoint: [0-2] 是 XYZ 平移力, [3-5] 是 XYZ 旋转力矩
                d->qfrc_applied[3] = 0.5; // 在 Roll 轴施加 0.5 Nm 的狂风力矩！
            } else {
                d->qfrc_applied[3] = 0.0; // 风停
            }

            // ==========================================
            // [E] 记录日志：上帝视角的真值 vs UKF 估计值
            // ==========================================
            // 从绝对真值四元数 (NED系) 提取真实的 MRP Roll (sigma_x)
            // current_quat 是 [qw, qx, qy, qz]
            double qw_true = current_quat(0);
            double qx_true = current_quat(1);
            // MRP 的经典转换公式：sigma = q_vec / (1 + q_scalar)
            double true_mrp_roll = qx_true / (1.0 + qw_true);
            double true_omega1 = current_angvel(0);
            // 将时间、真实姿态、UKF姿态、四个电机转速打入 CSV
            log << current_time << ","
                << -target_position(2) << ","
                << -current_pos(2) << ","
                << true_mrp_roll << ","
                << dynamic_target_sigma(0) << ","
                << speeds[0] << "," << speeds[1] << "," << speeds[2] << "," << speeds[3] << "\n";

            mj_step(m, d); // 物理引擎推进 1ms
        }

        // ==========================================
        // [B] 渲染时刻 (60Hz 触发)
        // ==========================================
        // 获取窗口大小
        int viewport_width, viewport_height;
        glfwGetFramebufferSize(window, &viewport_width, &viewport_height);
        mjrRect viewport = {0, 0, viewport_width, viewport_height};

        // 更新场景并在屏幕上画出来
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // 交换显存并处理鼠标/键盘事件
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 5. 优雅地打扫战场
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;

}