import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. 读取数据
try:
    df = pd.read_csv('mujoco_sitl_log.csv')
except FileNotFoundError:
    print("找不到 mujoco_sitl_log.csv，请确认仿真是否成功运行并生成了日志。")
    exit()

# 过滤掉起飞阶段，只截取进入 8 字航线后的平稳数据 (比如 3 秒以后)
df = df[df['time'] > 3.0]

# 2. 设置科研级画图风格
plt.style.use('bmh')
fig = plt.figure(figsize=(14, 10))

# ==========================================
# 子图 1: 上帝视角的 2D 水平轨迹 (XY平面)
# ==========================================
ax1 = plt.subplot(2, 2, (1, 2))
ax1.plot(df['target_x'], df['target_y'], 'r--', linewidth=2, label='Target Trajectory (Figure-8)')
ax1.plot(df['pos_x'], df['pos_y'], 'k-', linewidth=1.5, label='Actual Trajectory (SITL)')
ax1.set_title('SITL Simulation: 2D Horizontal Trajectory Tracking (Top-Down View)', fontsize=14)
ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')
ax1.axis('equal')  # 保证 XY 比例尺一致，8字不会变形
ax1.legend()
ax1.grid(True)

# 为了展示飞行方向，可以在实际轨迹上加几个箭头
# step = len(df) // 20
# ax1.quiver(df['pos_x'].iloc[::step], df['pos_y'].iloc[::step],
#            np.gradient(df['pos_x'].iloc[::step]), np.gradient(df['pos_y'].iloc[::step]),
#            angles='xy', scale_units='xy', scale=0.5, color='blue', alpha=0.6, width=0.003)

# ==========================================
# 子图 2: X 轴独立跟踪曲线
# ==========================================
ax2 = plt.subplot(2, 2, 3)
ax2.plot(df['time'], df['target_x'], 'r--', label='Target X')
ax2.plot(df['time'], df['pos_x'], 'k-', label='Actual X')
ax2.set_title('X-Axis Tracking', fontsize=12)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Position (m)')
ax2.legend()

# ==========================================
# 子图 3: Y 轴独立跟踪曲线
# ==========================================
ax3 = plt.subplot(2, 2, 4)
ax3.plot(df['time'], df['target_y'], 'r--', label='Target Y')
ax3.plot(df['time'], df['pos_y'], 'k-', label='Actual Y')
ax3.set_title('Y-Axis Tracking', fontsize=12)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Position (m)')
ax3.legend()

plt.tight_layout()
plt.show()