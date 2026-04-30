import numpy as np
import matplotlib.pyplot as plt
import math

# 1. 地形高度函數
def terrain_z(x, slope):
    return slope * x

# 2. 多點 Bézier 曲線高度計算
def bezier_height(control_z, u):
    """
    control_z: array of control points' z-values
    u: array of parameter [0,1]
    returns z at each u
    """
    n = len(control_z) - 1
    C = [math.comb(n, i) for i in range(n+1)]
    z = np.zeros_like(u)
    for i in range(n+1):
        z += C[i] * (u**i) * ((1-u)**(n-i)) * control_z[i]
    return z

# 3. 參數設定
SL = 0.2  # 步長 (m)
slope_deg = 20
slope = math.tan(math.radians(slope_deg))
step_height = 0.15  # 中間最大抬升

# 4. 起點與終點設定 (可隨意調整)
start = np.array([-0.05, terrain_z(-0.05, slope) + 0.05])  # 起點不一定在地面上
touchdown = np.array([SL, terrain_z(SL, slope)])

# 5. 中間控制點設計 (可靈活調整)
fractions = [0.2, 0.5, 0.8]  # 比例位置
lifts =      [0.05, step_height, 0.05]  # 抬升高度
midpoints = []
for f, h in zip(fractions, lifts):
    x_mid = start[0] + f * (touchdown[0] - start[0])
    z_mid = terrain_z(x_mid, slope) + h
    midpoints.append([x_mid, z_mid])
# 組合控制點
control_points = np.vstack([start, midpoints, touchdown])
control_z = control_points[:,1]

# 6. 時間參數與時間縮放 (速度分佈)
num = 300
t = np.linspace(0, 1, num)
# 半波余弦速度分佈：v = 1 - cos(2πt), s = ∫v dt
s = t - np.sin(2*math.pi*t)/(2*math.pi)

# 7. 足尖世界軌跡
x_path = start[0] + s * (touchdown[0] - start[0])
z_path = bezier_height(control_z, s)

# 8. 著地角度設計
theta0 = math.radians(20)   # 起始腳尖角度 (rad)
thetaf = math.radians(-5)   # 著地腳尖角度 (rad)
# 線性插值
theta = theta0 + s * (thetaf - theta0)

# 9. 繪圖
fig, axs = plt.subplots(3, 1, figsize=(8, 12))

# (a) 足尖軌跡與控制點
ax = axs[0]
# 地形
x_ter = np.linspace(start[0]-0.1, SL+0.1, 200)
z_ter = terrain_z(x_ter, slope)
ax.plot(x_ter, z_ter, 'k--', label='Terrain')
ax.plot(x_path, z_path, 'b-', label='Swing Path')
ax.scatter(control_points[:,0], control_points[:,1], c='red', marker='o', label='Control Points')
ax.set_title('Terrain-aware Quintic+Bezier Swing Trajectory')
ax.set_xlabel('X (m)'); ax.set_ylabel('Z (m)')
ax.grid(); ax.legend(); ax.axis('equal')

# (b) 速度分佈
ax = axs[1]
v = np.gradient(s, t)  # normalized speed
ax.plot(t, v, 'g-')
ax.set_title('Velocity Profile (Half-cosine)')
ax.set_xlabel('Normalized Time t'); ax.set_ylabel('Normalized Speed v')
ax.grid()

# (c) 著地角度
ax = axs[2]
ax.plot(t, np.degrees(theta), 'm-')
ax.set_title('Foot Orientation Angle')
ax.set_xlabel('Normalized Time t'); ax.set_ylabel('Angle (deg)')
ax.grid()

plt.tight_layout()
plt.show()
