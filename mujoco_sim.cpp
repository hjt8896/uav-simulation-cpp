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
#include "LowPassFilter.h"
#include "MathUtils.h"
#include "VelocitySMC.h"
#include "MRPSteering.h"
#include "AltitudeSMC.h"
#include "BMI088_Sensor.h"
#include "Barometer_Sensor.h"
#include "GPS_Sensor.h"
#include "USQUE.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ★ 强制字节对齐，保持和 STM32 完全一致！
#pragma pack(push, 1)
struct HitlSensorPacket {
    uint8_t header1 = 0x55;      // 帧头 1
    uint8_t header2 = 0xAA;      // 帧头 2
    float gyro[3]{};               // 陀螺仪 (rad/s)
    float accel[3]{};              // 加速度计 (m/s^2)
    float mag[3]{};                // 磁力计 (Gauss)

    float pos[3]{};
    float vel[3]{};
    float baro_altitude{};

    // ★ 新增：传感器刷新心跳包！(Bit0: GPS有更新, Bit1: Baro有更新)
    uint8_t sensor_mask{};
    // ===================================
    // 2. 人类意志层 (遥控/期望指令)
    // ===================================
    float target_z{};        // 期望 Z 轴高度 (或者你也可以传推力拨杆值)
    float cmd_roll{};        // 期望 Roll 角度
    float cmd_pitch{};       // 期望 Pitch 角度
    float cmd_yaw_rate{};    // 期望 Yaw 旋转角速度

    uint32_t timestamp{};          // 仿真时间戳 (ms)
    uint8_t checksum{};            // 累加校验和
};
#pragma pack(pop)

// ★ 强制字节对齐
#pragma pack(push, 1)
struct HitlMotorPacket {
    uint8_t header1 = 0x55;      // 帧头 1
    uint8_t header2 = 0xBB;      // 帧头 2 (注意和传感器包的 0xAA 区分)
    float motor_speeds[4]{};     // 四个电机的目标转速 (rad/s)
    // 2. 飞控黑匣子数据 (USQUE 解算结果)
    float ukf_mrp[3]{};          // 飞控解算的 MRP 姿态
    float ukf_omega[3]{};        // 飞控解算的角速度
    uint8_t checksum{};          // 校验和
};
#pragma pack(pop)

// 辅助函数：计算校验和
uint8_t calculate_checksum(const HitlSensorPacket& pkt) {
    uint8_t* ptr = (uint8_t*)&pkt.gyro[0]; // 从数据净荷开始算
    uint8_t sum = 0;
    // 数据净荷总长度：20个float(76字节) + 1个uint32(4字节) + 1 uint8 = 85字节
    for (int i = 0; i < 85; ++i) {
        sum += ptr[i];
    }
    return sum;
}

// 辅助函数：计算校验和
uint8_t calculate_checksum(const HitlMotorPacket& pkt) {
    uint8_t* ptr = (uint8_t*)&pkt.motor_speeds[0]; // 从数据净荷开始算
    uint8_t sum = 0;
    // 数据净荷总长度：10个float(40字节)
    for (int i = 0; i < 40; ++i) {
        sum += ptr[i];
    }
    return sum;
}
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
std::vector<double> current_motor_speeds = {500.0, 500.0, 500.0, 500.0};
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

    // ★ 加上这段护城河代码！
    if (!m) {
        std::cerr << "❌ MuJoCo 启动惨败，原因: " << error << std::endl;
        return 1;
    }

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
    Eigen::Vector3d target_position(0.0, 0.0, -1.0); // 悬停在 1 米高空
    Eigen::Vector3d dynamic_target_sigma(0, 0, 0);

    USQUE ukf;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(9, 9);
    Q.block<3,3>(0,0) *= 1e-4; Q.block<3,3>(3,3) *= 1e-4; Q.block<3,3>(6,6) *= 1e-6;
    ukf.setProcessNoise(Q);
    BMI088_Sensor bmi088;
    Barometer_Sensor baro;
    GPS_Sensor gps;
    // ==========================================
    // [物理净化] 初始化陷波滤波器 (Notch Filter)
    // ==========================================
    // NotchFilter notch_gyro[3];
    // NotchFilter notch_acc[3];
    //
    // // 找准“病灶”：飞机悬停时电机转速约 500 rad/s
    // // 换算成物理赫兹频率：500 / (2 * PI) ≈ 79.5 Hz
    // float hover_freq_hz = 500.0 / (2.0 * M_PI);
    // float sample_rate = 1000.0; // 我们的物理循环是 1ms (1000Hz)
    // float Q_factor = 2.0;       // Q=2.0 意味着挖一个宽度适中的坑
    //
    // // 初始化 6 个通道的滤波器，专杀 79.5 Hz 的谐波！
    // for (int i = 0; i < 3; ++i) {
    //     notch_gyro[i].init(hover_freq_hz, sample_rate, Q_factor);
    //     notch_acc[i].init(hover_freq_hz, sample_rate, Q_factor);
    // }

    // ==========================================
    // [物理净化] 换装更坚固的低通防线 (LPF)
    // ==========================================
    LowPassFilter lpf_gyro[3];
    LowPassFilter lpf_acc[3];

    float sample_rate = 1000.0f;

    // ★ 核心手术：对加速度计下狠手，对陀螺仪网开一面！
    float acc_cutoff_hz = 30.0f;   // 彻底滤除所有电机高频震动，给 UKF 提供纯净重力向量
    float gyro_cutoff_hz = 120.0f; // 保持极高的响应速度，坚决拒绝相位延迟！

    for (int i = 0; i < 3; ++i) {
        lpf_gyro[i].init(gyro_cutoff_hz, sample_rate);
        lpf_acc[i].init(acc_cutoff_hz, sample_rate);
    }

    AltitudeSMC alt_smc(mass);
    MRPSteering steering_real{};
    SMCController smc(inertia);

    double l = 0.2, k_f = 1e-5, k_m = 2e-7;
    Mixer mixer(l, k_f, k_m, 100.0, 1000.0);
    // 初始假设四个电机都在悬停转速附近 (约 500 rad/s)
    // std::vector<double> current_motor_speeds = {500.0, 500.0, 500.0, 500.0};
    static std::vector<double> actual_motor_speeds = {500.0, 500.0, 500.0, 500.0};
    double alpha = m->opt.timestep / 0.01; // 离散化步长系数
    if (alpha > 1.0) alpha = 1.0; // 终极防爆保护：防止 dt 过大导致系统发散
    // 设定你期望无人机悬停的绝对 3D 空间坐标

    std::ofstream log("mujoco_sitl_log.csv");
    log << "time,target_z,pos_z,"
           "true_roll,target_roll,ukf_roll,true_pitch,target_pitch,ukf_pitch,true_yaw,target_yaw,ukf_yaw,"
           "true_omegax,target_omegax,ukf_omegax,true_omegay,target_omegay,ukf_omegay,true_omegaz,target_omegaz,ukf_omegaz,"
           "motor1,motor2,motor3,motor4\n";

    std::cout << "引擎点火！无人机正在 1 米高空尝试抵抗重力..." << std::endl;
    std::cout << "3D 视界已打开！准备渲染..." << std::endl;

    // ==========================================
    // ★ 视觉链路：柔性连接 Python 神经网络
    // ==========================================
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(8080);
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    bool use_vision = false; // ★ 视觉链路护盾标志位

    std::cout << "正在探寻 Python 视觉神经网络..." << std::endl;
    // 尝试握手
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cout << "⚠️ 警告：未检测到 Python 视觉节点，FPV 画面传输已关闭，降级为纯物理/飞控推演！" << std::endl;
    } else {
        use_vision = true;
        std::cout << "✅ 成功连接 Python 中转站，视觉链路已激活！" << std::endl;
    }

    std::cout << "正在接通 STM32H7 硬件在环隧道..." << std::endl;
    int serial_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    bool use_hitl = false; // ★ 核心护盾：状态标志位

    if (serial_fd < 0) {
        std::cout << "⚠️ 警告：未检测到 STM32 飞控，自动降级为纯虚拟 SITL 模式运行！" << std::endl;
    } else {
        // 只有连上了，才去配置波特率
        struct termios tty;
        tcgetattr(serial_fd, &tty);
        cfmakeraw(&tty);
        tcsetattr(serial_fd, TCSANOW, &tty);

        use_hitl = true; // 激活硬件在环标志
        std::cout << "✅ 成功连通 STM32 飞控，HITL 模式启动！" << std::endl;
    }

    // double yaw = 0.0;
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
            // Eigen::Vector3d current_angvel(w_mj[0], -w_mj[1], -w_mj[2]);

            // 四元数翻转 (qw, qx, qy, qz) -> ENU转NED
            // 四元数的标量 qw 不变，X不变，Y和Z取反
            Eigen::Vector4d current_quat(q_mj[0], q_mj[1], -q_mj[2], -q_mj[3]);
            Eigen::Matrix3d R_NB = MathUtils::QuatToRotationMatrix(current_quat).transpose();
            Eigen::Vector3d acc_true = acc_frd - R_NB * current_true_acc;

            // ==========================================
            // [A.4] 真实世界滤镜：注入高斯白噪声、零偏游走与高频振动
            // ==========================================
            double dt = m->opt.timestep;
            // 现在的 acc_true 和 gyro_frd 是 MuJoCo 给的绝对真值，我们把它弄脏！
            Eigen::Vector3d noisy_gyro = bmi088.read_gyro(gyro_frd, current_motor_speeds, current_time, dt);
            Eigen::Vector3d noisy_acc = bmi088.read_acc(acc_true, current_motor_speeds, current_time);
            static Eigen::Vector3d last_gps_pos = current_pos;
            static Eigen::Vector3d last_gps_vel = current_vel;
            uint8_t current_sensor_mask = 0; // 默认这毫秒啥也没刷新

            double noisy_baro_z = baro.read_baro(current_pos(2), dt);
            current_sensor_mask |= 0x02; // Bit 1 设为 1
            if (gps.read_gps(current_pos, current_vel, current_time, dt, last_gps_pos, last_gps_vel)) {
                // 进入此分支，说明正好等到了 10Hz 的那一拍！
                // 这里拿到的 gps_pos 就是带有 100ms 物理延迟的陈旧数据。
                // 你可以把它们打包装进 HitlSensorPacket 发给 STM32。
                current_sensor_mask |= 0x01; // Bit 0 设为 1，告诉 STM32：GPS 刷新了！
            }
            // ==========================================
            // [A.6] 罗盘上线：打破 Z 轴的不可观测诅咒！
            // ==========================================
            // 1. 模拟真实地磁场 (NED坐标系下，向北0.22，向下0.45，单位Gauss)
            Eigen::Vector3d mag_N_true(0.22, 0.0, 0.45);

            // 2. 将地球磁场转换到当前的真实机体坐标系
            Eigen::Vector3d mag_B_true = R_NB * mag_N_true;

            // 3. 加入一点点罗盘高斯白噪声 (快速手写一个简易噪声发生器)
            Eigen::Vector3d noisy_mag = mag_B_true + Eigen::Vector3d(
                ((double)rand() / RAND_MAX - 0.5) * 0.02,
                ((double)rand() / RAND_MAX - 0.5) * 0.02,
                ((double)rand() / RAND_MAX - 0.5) * 0.02
            );

            // ==========================================
            // [A.5] 物理净化术：全局低通滤波洗掉所有高频噪声
            // ==========================================
            Eigen::Vector3d filtered_gyro;
            Eigen::Vector3d filtered_acc;
            for (int i = 0; i < 3; ++i) {
                filtered_gyro(i) = lpf_gyro[i].apply(noisy_gyro(i));
                filtered_acc(i)  = lpf_acc[i].apply(noisy_acc(i));
            }

            // ==========================================
            // ★ HITL：向 STM32 飞控注入传感器数据！
            // ==========================================
            if (use_hitl)
            {
                // ★ 只有成功连上了硬件，才允许往外发数据
                HitlSensorPacket tx_pkt;
                // 填装数据 (转为 C++ 基础数组格式)
                tx_pkt.gyro[0] = noisy_gyro(0);
                tx_pkt.gyro[1] = noisy_gyro(1);
                tx_pkt.gyro[2] = noisy_gyro(2);

                tx_pkt.accel[0] = noisy_acc(0);
                tx_pkt.accel[1] = noisy_acc(1);
                tx_pkt.accel[2] = noisy_acc(2);

                tx_pkt.mag[0] = noisy_mag(0);
                tx_pkt.mag[1] = noisy_mag(1);
                tx_pkt.mag[2] = noisy_mag(2);

                tx_pkt.pos[0] = last_gps_pos(0);
                tx_pkt.pos[1] = last_gps_pos(1);
                tx_pkt.pos[2] = last_gps_pos(2);

                tx_pkt.vel[0] = last_gps_vel(0);
                tx_pkt.vel[1] = last_gps_vel(1);
                tx_pkt.vel[2] = last_gps_vel(2);
                tx_pkt.baro_altitude = noisy_baro_z;

                tx_pkt.sensor_mask = current_sensor_mask;
                tx_pkt.timestamp = (uint32_t)(current_time * 1000.0); // 换算为毫秒

                // 2. 填装人类遥控指令
                // (把原来写在循环里的键盘读取逻辑移到这里)
                double flight_speed_z = 3.0;
                if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) target_position(2) -= flight_speed_z * dt;
                if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) target_position(2) += flight_speed_z * dt;
                tx_pkt.target_z = target_position(2);

                double max_angle = 0.3;
                if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) tx_pkt.cmd_pitch = -max_angle;
                else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) tx_pkt.cmd_pitch = max_angle;
                else tx_pkt.cmd_pitch = 0.0;

                if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) tx_pkt.cmd_roll = max_angle;
                else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) tx_pkt.cmd_roll = -max_angle;
                else tx_pkt.cmd_roll = 0.0;

                if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) tx_pkt.cmd_yaw_rate = -1.0;
                else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) tx_pkt.cmd_yaw_rate = 1.0;
                else tx_pkt.cmd_yaw_rate = 0.0;

                // 计算校验和并发送
                tx_pkt.checksum = calculate_checksum(tx_pkt);
                write(serial_fd, &tx_pkt, sizeof(HitlSensorPacket));

                // ==========================================
                // [肉体行为 2]：死等大脑下达运动指令 (阻塞读取)
                // ==========================================
                HitlMotorPacket rx_pkt;
                int bytes_read = 0;
                // 这里用阻塞 read，或者用 select/poll 设置个微小的超时
                while (bytes_read < sizeof(HitlMotorPacket)) {
                    int n = read(serial_fd, ((uint8_t*)&rx_pkt) + bytes_read, sizeof(HitlMotorPacket) - bytes_read);
                    if (n > 0) bytes_read += n;
                }
                // 准备用来存 STM32 状态的变量
                Eigen::Vector3d stm32_ukf_mrp = Eigen::Vector3d::Zero();
                Eigen::Vector3d stm32_ukf_omega = Eigen::Vector3d::Zero();
                // 校验数据合法性
                if (rx_pkt.header1 == 0x55 && rx_pkt.header2 == 0xBB) {
                    if (calculate_checksum(rx_pkt)==rx_pkt.checksum)
                    {
                        current_motor_speeds[0] = rx_pkt.motor_speeds[0];
                        current_motor_speeds[1] = rx_pkt.motor_speeds[1];
                        current_motor_speeds[2] = rx_pkt.motor_speeds[2];
                        current_motor_speeds[3] = rx_pkt.motor_speeds[3];

                        // 2. ★ 提取 STM32 解算出来的真实飞控状态！
                        stm32_ukf_mrp(0) = rx_pkt.ukf_mrp[0];
                        stm32_ukf_mrp(1) = rx_pkt.ukf_mrp[1];
                        stm32_ukf_mrp(2) = rx_pkt.ukf_mrp[2];

                        stm32_ukf_omega(0) = rx_pkt.ukf_omega[0];
                        stm32_ukf_omega(1) = rx_pkt.ukf_omega[1];
                        stm32_ukf_omega(2) = rx_pkt.ukf_omega[2];
                    }
                    else
                    {
                        std::cout << "Motor指令接受失败！" << std::endl;
                    }
                }
                for (int i = 0; i < 4; ++i) {
                    // 1. 核心手术：一阶惯性环节离散化差分方程
                    // 物理含义：真实转速在努力地“追赶”飞控下达的期望转速 current_motor_speeds
                    actual_motor_speeds[i] = actual_motor_speeds[i] + alpha * (current_motor_speeds[i] - actual_motor_speeds[i]);

                    // 注意写入 XML 的是推力大小 (speeds^2 * k_f) 或者根据你 XML motor 的定义直接给 control 信号
                    // 如果你的 XML <motor> 标签自带了 gear，你可能需要根据配置给 speeds，
                    // 咱们这里延续你之前的物理公式：
                    d->ctrl[i] = actual_motor_speeds[i] * actual_motor_speeds[i] * k_f;
                    // 更新当前电机转速，供下一毫秒的传感器震动模型使用！
                }

                if (current_time >= 0.5 && current_time <= 0.6) {
                    // qfrc_applied 是 MuJoCo 里的外力数组。
                    // 对于 freejoint: [0-2] 是 XYZ 平移力, [3-5] 是 XYZ 旋转力矩
                    d->qfrc_applied[3] = 0.5; // 在 Roll 轴施加 0.5 Nm 的狂风力矩！
                } else {
                    d->qfrc_applied[3] = 0.0; // 风停
                }

                // ==========================================
                // [E] 记录日志：把 STM32 的状态写入 CSV！
                // ==========================================
                // ★ 注意：把这里写入的 ukf_mrp 和 ukf_omega 换成刚才从串口读出来的 stm32 变量！

                Eigen::Vector4d q_safe = current_quat;
                if (q_safe(0) < 0.0) { q_safe = -q_safe; }
                // Eigen::Vector3d current_mrp = ukf.get_mrp();
                // 提取物理引擎的上帝视角真值
                Eigen::Vector3d true_mrp = Eigen::Vector3d(q_safe(1), q_safe(2), q_safe(3)) / (1.0 + q_safe(0));

                // ==========================================
                // [E] 记录日志：把 STM32 的状态写入 CSV！
                // ==========================================
                // ★ 注意：把这里写入的 ukf_mrp 和 ukf_omega 换成刚才从串口读出来的 stm32 变量！
                log << current_time << ","
                    << -target_position(2) << "," << -current_pos(2) << ","
                    << true_mrp(0) << "," << 0 << "," << stm32_ukf_mrp(0) << ","
                    << true_mrp(1) << "," << 0 << "," << stm32_ukf_mrp(1) << ","
                    << true_mrp(2) << "," << 0 << "," << stm32_ukf_mrp(2) << ","

                    << gyro_frd(0) << "," << 0 << "," << stm32_ukf_omega(0) << ","
                    << gyro_frd(1) << "," << 0 << "," << stm32_ukf_omega(1) << ","
                    << gyro_frd(2) << "," << 0 << "," << stm32_ukf_omega(2) << ","
                    << current_motor_speeds[0] << "," << current_motor_speeds[1] << "," << current_motor_speeds[2] << "," << current_motor_speeds[3] << "\n";
            }
            else
            {
                // ==========================================
                // [B] 姿态解算滤波 (UKF)
                // ==========================================
                ukf.predict(dt);
                ukf.update_gyro(filtered_gyro);
                ukf.update_accel(filtered_acc);
                // 4. ★ 极其关键：喂给 UKF，让它终于能看见“北”了！
                ukf.update_mag(noisy_mag);

                // ==========================================
                // [C]  SMC飞控 (高度和姿态闭环控制)
                // ==========================================
                // 定义两个飞控输出变量，交给内环
                double dynamic_thrust = 0.0;

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
                // =====================================
                // [纯解析架构]：基于 Eq 3.147 的姿态指令重构
                // =====================================
                Eigen::Vector4d ukf_quat = ukf.get_quaternion();
                Eigen::Vector3d current_mrp = MathUtils::quaternion_to_mrp(ukf_quat);
                Eigen::Vector4d q_safe = current_quat;
                if (q_safe(0) < 0.0) { q_safe = -q_safe; }
                // Eigen::Vector3d current_mrp = ukf.get_mrp();
                // 提取物理引擎的上帝视角真值
                Eigen::Vector3d true_mrp = Eigen::Vector3d(q_safe(1), q_safe(2), q_safe(3)) / (1.0 + q_safe(0));
                // 1. 提取当前 Yaw，并构造出“基础 MRP” (sigma_prime)
                Eigen::Matrix3d R_NB_ukf = MathUtils::mrp_to_dcm(current_mrp).transpose();
                double current_yaw = std::atan2(R_NB_ukf(1, 0), R_NB_ukf(0, 0));
                // 纯 Yaw 旋转的 MRP 极其简单：[0, 0, tan(theta/4)]
                Eigen::Vector3d sigma_base(0, 0, std::tan(current_yaw / 4.0));

                // 2. 接收人类的相对指令
                double cmd_roll = 0.0, cmd_pitch = 0.0, yaw_rate_cmd = 0.0;
                double max_angle = 0.3;

                if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) cmd_pitch =  -max_angle;
                if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) cmd_pitch = max_angle;
                if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) cmd_roll  = max_angle;
                if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) cmd_roll  =  -max_angle;
                // if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) yaw   -= 0.001;
                // if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) yaw   +=  0.001;

                // 3. 将人类指令转化为“期望指令 MRP” (sigma_double_prime)
                // 因为 Roll 和 Pitch 是小角度独立指令，我们可以快速构造
                Eigen::Vector3d sigma_cmd;
                sigma_cmd=MathUtils::mrp_switchto_shadow(sigma_cmd);
                sigma_cmd(0) = std::tan(cmd_roll / 4.0);
                sigma_cmd(1) = std::tan(cmd_pitch / 4.0);
                sigma_cmd(2) = 0;

                // 4. ★ 圣经降临：用公式 3.147 瞬间完成非线性叠加！
                dynamic_target_sigma = MathUtils::mrp_add(sigma_base, sigma_cmd);


                // ==========================================
                // [内环] 直接接收 dynamic_thrust 和 dynamic_target_sigma
                // ==========================================
                Eigen::Vector3d omega_d(0, 0, 0);
                Eigen::Vector3d omega_d_dot(0, 0, 0);


                Eigen::Vector3d mrp_err = MathUtils::mrp_add(-dynamic_target_sigma, current_mrp);
                // 5. 永远不要忘了套上一层影子集结界
                mrp_err = MathUtils::mrp_switchto_shadow(mrp_err);
                // 姿态环和角速度环照常运作，只是外环指令的来源变了
                steering_real.compute_steering(mrp_err, omega_d, omega_d_dot);

                // 2. ★ 偏航轴夺权：剥夺 MRPSteering 对 Z 轴的控制权！
                // 直接把人类的摇杆输入（或高级算法）转化为期望偏航角速度
                if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) yaw_rate_cmd = -1.0; // 每秒转 1 rad
                if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) yaw_rate_cmd =  1.0;

                omega_d(2) = yaw_rate_cmd;     // 注入角速度指令
                // omega_d_dot(0) = 0.0;          // 匀速转弯，前馈角加速度为 0
                // omega_d_dot(1) = 0.0;          // 匀速转弯，前馈角加速度为 0
                omega_d_dot(2) = 0.0;          // 匀速转弯，前馈角加速度为 0

                Eigen::Vector3d current_omega = ukf.get_omega();
                Eigen::Vector3d tau_frd = smc.compute_torque(current_omega, omega_d, omega_d_dot, dt);
                // ==========================================
                // [D] 执行器海关：极其严谨的神经重接！
                // ==========================================
                // 按照 Thrust, Roll, Pitch, Yaw 顺序传入期望力矩
                current_motor_speeds = mixer.allocate(dynamic_thrust, tau_frd(0), tau_frd(1), tau_frd(2));


                // 直接按顺序写入 MuJoCo！再也不用在外部做恶心的索引映射了
                for (int i = 0; i < 4; ++i) {
                    // 1. 核心手术：一阶惯性环节离散化差分方程
                    // 物理含义：真实转速在努力地“追赶”飞控下达的期望转速 current_motor_speeds
                    actual_motor_speeds[i] = actual_motor_speeds[i] + alpha * (current_motor_speeds[i] - actual_motor_speeds[i]);
                    // 注意写入 XML 的是推力大小 (speeds^2 * k_f) 或者根据你 XML motor 的定义直接给 control 信号
                    // 如果你的 XML <motor> 标签自带了 gear，你可能需要根据配置给 speeds，
                    // 咱们这里延续你之前的物理公式：
                    d->ctrl[i] = actual_motor_speeds[i] * actual_motor_speeds[i] * k_f;
                    // 更新当前电机转速，供下一毫秒的传感器震动模型使用！
                }


                if (current_time >= 0.5 && current_time <= 0.6) {
                    // qfrc_applied 是 MuJoCo 里的外力数组。
                    // 对于 freejoint: [0-2] 是 XYZ 平移力, [3-5] 是 XYZ 旋转力矩
                    d->qfrc_applied[3] = 0.5; // 在 Roll 轴施加 0.5 Nm 的狂风力矩！
                } else {
                    d->qfrc_applied[3] = 0.0; // 风停
                }

    // ==========================================
                // [E] 记录日志：全维度姿态真值与 UKF 估计提取
                // ==========================================

                // 提取飞控大脑的 UKF 估计值
                Eigen::Vector3d ukf_mrp = current_mrp;
                Eigen::Vector3d ukf_omega = ukf.get_omega();
                // 写入日志
                log << current_time << ","
                    << -target_position(2) << "," << -current_pos(2) << ","
                    << true_mrp(0) << "," << mrp_err(0) << "," << ukf_mrp(0) << "," // Roll 组
                    << true_mrp(1) << "," << mrp_err(1) << "," << ukf_mrp(1) << "," // Pitch 组
                    << true_mrp(2) << "," << mrp_err(2) << "," << ukf_mrp(2) << "," // Yaw 组

                    << gyro_frd(0) << "," << omega_d(0) << "," << ukf_omega(0) << "," // Roll 组
                    << gyro_frd(1) << "," << omega_d(1) << "," << ukf_omega(1) << "," // Pitch 组
                    << gyro_frd(2) << "," << omega_d(2) << "," << ukf_omega(2) << "," // Yaw 组
                    << current_motor_speeds[0] << "," << current_motor_speeds[1] << "," << current_motor_speeds[2] << "," << current_motor_speeds[3] << "\n";
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

        // ==========================================
        // ★ 视觉发送时刻 (2.0s 触发，且必须连上 Python)
        // ==========================================
        static double last_fpv_time = 0.0;
        if (use_vision && (d->time - last_fpv_time >= 2.0)) {
            last_fpv_time = d->time;

            mjrRect fpv_viewport = {0, 0, 256, 256};
            mjvCamera fpv_cam;
            mjv_defaultCamera(&fpv_cam);
            fpv_cam.type = mjCAMERA_FIXED;
            fpv_cam.fixedcamid = mj_name2id(m, mjOBJ_CAMERA, "fpv_cam");

            mjv_updateScene(m, d, &opt, NULL, &fpv_cam, mjCAT_ALL, &scn);
            mjr_render(fpv_viewport, &scn, &con);

            unsigned char rgb[256 * 256 * 3];
            mjr_readPixels(rgb, NULL, fpv_viewport, &con);

            // ★ 终极防炸机补丁：MSG_NOSIGNAL！
            // 把你原来的 sendto 换成标准的 TCP send，并加上 MSG_NOSIGNAL。
            // 这样就算 Python 突然被关掉，C++ 也绝不崩溃，只会返回一个负数错误码！
            ssize_t bytes_sent = send(sockfd, rgb, sizeof(rgb), MSG_NOSIGNAL);

            if (bytes_sent < 0) {
                std::cout << "❌ 视觉链路意外断开 (Python 掉线)！已自动切断图传。" << std::endl;
                use_vision = false; // 掐断标志位，以后不再白白浪费算力渲染
                close(sockfd);      // 优雅关闭文件描述符
            } else {
                std::cout << "[" << d->time << "s] FPV 画面已发送给神经网络！" << std::endl;
            }
        }

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