import pandas as pd
import matplotlib.pyplot as plt

# 读取 SITL 仿真产生的数据日志
df = pd.read_csv('mujoco_sitl_log.csv')

plt.figure(figsize=(12, 8))

# 子图 1：姿态追踪 (上帝视角的物理真值 vs UKF 滤波解算值)
plt.subplot(2, 1, 1)
plt.plot(df['time'], df['true_yaw'], label='true_yaw', linewidth=2, color='black')
plt.plot(df['time'], df['target_yaw'], label='target_yaw', linestyle='--', color='red', alpha=0.8)
plt.title('SITL Simulation: mrp_roll Tracking (MuJoCo vs SR-UKF)')
plt.ylabel('mrp')
plt.grid(True)
plt.legend()

# 子图 2：底层执行器响应 (4个电机的实时转速)
plt.subplot(2, 1, 2)
plt.plot(df['time'], df['motor1'], label='Motor 1 (Front-Right)', alpha=0.8)
plt.plot(df['time'], df['motor2'], label='Motor 2 (Rear-Right)', alpha=0.8)
plt.plot(df['time'], df['motor3'], label='Motor 3 (Rear-Left)', alpha=0.8)
plt.plot(df['time'], df['motor4'], label='Motor 4 (Front-Left)', alpha=0.8)
# 画一条悬停基准线 (约 495 rad/s)
plt.axhline(y=495.2, color='black', linestyle=':', label='Theoretical Hover Speed')
plt.title('SITL Simulation: Motor RPM Output from SMC + Mixer')
plt.xlabel('Time (s)')
plt.ylabel('Rotor Speed (rad/s)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()