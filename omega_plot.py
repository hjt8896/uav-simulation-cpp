import pandas as pd
import matplotlib.pyplot as plt

# 1. 读取数据
try:
    df = pd.read_csv('mujoco_sitl_log.csv')
except FileNotFoundError:
    print("找不到 mujoco_sitl_log.csv，请先运行 C++ 仿真。")
    exit()

df = df[df['time'] > 1.0]

# 2. 设置绘图
plt.style.use('bmh')
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle('SITL GNC Performance: Target vs True vs UKF (MRP)', fontsize=16, fontweight='bold')

# ==========================================
# 轴 1: Roll (横滚)
# ==========================================
# ax1.plot(df['time'], df['target_roll'], 'r--', linewidth=2, label='Control Error')
ax1.plot(df['time'], df['true_roll'], 'k-', linewidth=1.5, label='True (MuJoCo Ground Truth)')
ax1.plot(df['time'], df['ukf_roll'], 'b-.', linewidth=1.5, alpha=0.8, label='UKF (Estimated)')
ax1.set_ylabel('Sigma(1)')
ax1.legend(loc='upper right')
ax1.grid(True)

# ==========================================
# 轴 2: Pitch (俯仰)
# ==========================================
# ax2.plot(df['time'], df['target_pitch'], 'r--', linewidth=2, label='Control Error')
ax2.plot(df['time'], df['true_pitch'], 'k-', linewidth=1.5, label='True (MuJoCo Ground Truth)')
ax2.plot(df['time'], df['ukf_pitch'], 'b-.', linewidth=1.5, alpha=0.8, label='UKF (Estimated)')
ax2.set_ylabel('Sigma(2)')
ax2.legend(loc='upper right')
ax2.grid(True)

# ==========================================
# 轴 3: Yaw (偏航)
# ==========================================
# ax3.plot(df['time'], df['target_yaw'], 'r--', linewidth=2, label='Control Error')
ax3.plot(df['time'], df['true_yaw'], 'k-', linewidth=1.5, label='True (MuJoCo Ground Truth)')
ax3.plot(df['time'], df['ukf_yaw'], 'b-.', linewidth=1.5, alpha=0.8, label='UKF (Estimated)')
ax3.set_ylabel('Sigma(3)')
ax3.set_xlabel('Time (s)')
ax3.legend(loc='upper right')
ax3.grid(True)

plt.tight_layout()
plt.subplots_adjust(top=0.93)
plt.show()