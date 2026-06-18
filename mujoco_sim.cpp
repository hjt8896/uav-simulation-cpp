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

int main() {
    std::cout << "--- 3D 可视化 SITL 启动 ---" << std::endl;

    // 1. 初始化 GLFW 窗口
    if (!glfwInit()) { std::cerr << "GLFW 初始化失败！" << std::endl; return 1; }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "SR-UKF 飞控可视化", NULL, NULL);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // 开启垂直同步 (大约 60Hz 刷新率)

    // 2. 加载 MuJoCo 模型
    char error[1000] = "Could not load XML model";
    mjModel* m = mj_loadXML("quadcopter.xml", 0, error, 1000);
    mjData* d = mj_makeData(m);

    // ★ 3. 初始化 MuJoCo 渲染数据结构
    mjvCamera cam;                      // 摄像机
    mjvOption opt;                      // 渲染选项
    mjvScene scn;                       // 3D 场景
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
    SMCController smc(inertia);
    AttitudeUKF ukf;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9);
    Q.block<3,3>(0,0) *= 1e-6; Q.block<3,3>(3,3) *= 1e-4; Q.block<3,3>(6,6) *= 1e-8;
    ukf.setProcessNoise(Q);

    // 初始化阵风/目标变量
    double base_thrust = mass * 9.81;
    Eigen::Vector3d target_sigma(0, 0, 0);

    std::ofstream log("mujoco_sitl_log.csv");
    log << "time,true_roll,ukf_roll,motor1,motor2,motor3,motor4\n";

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

            // ==========================================
            // [B] 飞控大脑解算 (UKF + SMC)
            // ==========================================
            double dt = m->opt.timestep;

            ukf.predict(dt);
            ukf.update_gyro(gyro_frd);
            ukf.update_accel(acc_frd); // MuJoCo极其牛逼，它的加速度计天然就包含了-g和机动加速度！

            // 计算滑模控制力矩 (暂时不接 MRPSteering 外环，直接控姿态到 0)
            Eigen::Vector3d omega_d(0, 0, 0);
            Eigen::Vector3d omega_d_dot(0, 0, 0);

            // 我们利用极简 P 控制作为临时外环，算出目标角速度
            Eigen::Vector3d current_mrp = ukf.get_mrp();
            omega_d = -5.0 * current_mrp; // P = 5.0

            Eigen::Vector3d tau_frd = smc.compute_torque(ukf.get_omega(), omega_d, omega_d_dot, dt);

            // ==========================================
            // [C] 执行器海关：极其严谨的神经重接！
            // ==========================================
            auto speeds = mixer.allocate(base_thrust, tau_frd(0), tau_frd(1), tau_frd(2));

            // C++(Mixer) -> XML(MuJoCo)
            d->ctrl[0] = speeds[0] * speeds[0] * k_f; // XML M1 (前右) <== C++ speeds[0] (前右)
            d->ctrl[1] = speeds[3] * speeds[3] * k_f; // XML M2 (后右) <== C++ speeds[3] (后右)
            d->ctrl[2] = speeds[1] * speeds[1] * k_f; // XML M3 (后左) <== C++ speeds[1] (后左)
            d->ctrl[3] = speeds[2] * speeds[2] * k_f; // XML M4 (前左) <== C++ speeds[2] (前左)

            if (current_time >= 0.5 && current_time <= 0.6) {
                // qfrc_applied 是 MuJoCo 里的外力数组。
                // 对于 freejoint: [0-2] 是 XYZ 平移力, [3-5] 是 XYZ 旋转力矩
                d->qfrc_applied[0] = 0.5; // 在 Roll 轴施加 0.5 Nm 的狂风力矩！
            } else {
                d->qfrc_applied[0] = 0.0; // 风停
            }

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