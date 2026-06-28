# Quad-SITL-GNC: Aerospace-Grade Quadcopter Simulation

[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![MuJoCo](https://img.shields.io/badge/Physics-MuJoCo_3.9.0-black.svg)](https://mujoco.org/)
[![Eigen3](https://img.shields.io/badge/Math-Eigen3-red.svg)](https://eigen.tuxfamily.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

基于 MuJoCo 物理引擎构建的多旋翼无人机 SITL（软件在环）仿真环境。本项目在底层控制架构中引入了深空探测器级别的非线性姿态动力学算法，旨在提供一个**彻底免疫万向节死锁**、**完美解决 180° 偏航控制反转**的纯正 C++ 飞控开发基石。

## ✨ 核心算法特性 (Core Features)

* **9 维 SR-UKF 状态估计器 (Square-Root Unscented Kalman Filter)**
    * 基于 MRP (修正罗德里格斯参数) 的空间折叠流形，彻底免疫 360° 奇异点。
    * 包含精确的协方差雅可比伴随映射 (Jacobian Shadow Set Mapping)，在跨越姿态边界时确保误差椭球不发散。
    * 引入动力学先验约束，完美规避由于非线性陀螺耦合力矩引起的相位估计翻转。

* **非线性滑模控制器 (Sliding Mode Control, SMC)**
    * 极其暴力的底层闭环镇压能力，无惧外部强风扰动。
    * 包含完整的 $\omega \times (I\omega)$ 陀螺力矩前馈补偿 (Gyroscopic Coupling Compensation)。

* **基于当前航向系的期望姿态重构 (Attitude Target Reconstruction)**
    * 运用 Schaub & Junkins 航天动力学中的 MRP 连续旋转定理 (Eq 3.147)。
    * 完美打通机体坐标系与全局 NED 坐标系，实现 FPV 视角的极其丝滑跟手，无论偏航角度多大，推杆始终顺应机头方向。

* **真实物理传感器仿真**
    * 模拟 BMI088 加速度计/陀螺仪的游走零偏与高频机械震动，并集成全局低通滤波器 (LPF)。
    * 融合罗盘 (Magnetometer) 观测，打破偏航轴不可观测诅咒。

## 🛠️ 依赖环境 (Dependencies)

* **Ubuntu / Linux** (推荐开发环境)
* **CMake** (3.10+)
* **MuJoCo** (已测试版本 3.9.0)
* **Eigen3** (纯头文件矩阵运算库)
* **GLFW3** (用于 3D 渲染窗口和键鼠交互)

```bash
# Ubuntu 环境下快速安装依赖
sudo apt update
sudo apt install cmake build-essential libglfw3-dev
```
## 编译与运行 (Build & Run)
### 编译项目
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```
### 运行
``` bash
./aircraft_sim
```
## 飞行控制 (Flight Controls)
程序运行后，会弹出一个 60Hz 渲染的 3D 物理引擎窗口。请在窗口处于激活状态时，使用键盘进行人类试飞员接管：

W : 压低机头 (向前飞)

S : 抬高机头 (向后飞)

A : 向左横滚

D : 向右横滚

Q : 偏航轴左转 (-1.0 rad/s)

E : 偏航轴右转 (+1.0 rad/s)

↑ (Up) : 增加目标高度

↓ (Down) : 降低目标高度

鼠标视角控制：

左键拖拽：旋转视角

右键拖拽：平移视角

滚轮：缩放

## 数据遥测与可视化 (Telemetry & Plotting)
每次运行结束后，系统会在目录下自动生成极其详尽的黑匣子日志文件 mujoco_sitl_log.csv，记录了 1000Hz 刷新率下的物理真值、指令目标与 UKF 估计状态。
本项目提供了一个科研级的 Python 绘图脚本，用于检阅 GNC 系统的全链路性能：
```bash
# 需要安装 pandas 和 matplotlib
pip install pandas matplotlib

# 运行绘图脚本，生成 3D 姿态追踪性能对比图
python plot_attitude.py
```

