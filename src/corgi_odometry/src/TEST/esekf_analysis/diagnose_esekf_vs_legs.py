#!/usr/bin/env python3
"""
diagnose_esekf_vs_legs.py

Compare ESEKF velocity estimate vs pure leg odometry vs ground truth.
Key question: Why is ESEKF (96 mm/s RMSE) worse than pure legs (4.5 mm/s)?

Diagnostics:
1. Body-frame velocity: est_vel vs z_avg vs GT_body
2. Attitude comparison: est_q vs GT_q (IMU)
3. Yaw drift analysis
4. Bias evolution
5. Pure leg integration baseline

Author: debug helper for corgi_odometry (Step 3)
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path


# =====================================================================
# Quaternion helpers (avoid scipy dependency)
# =====================================================================
def quat_to_rotmat(qw, qx, qy, qz):
    """Quaternion (w,x,y,z) → 3×3 rotation matrix."""
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),      1 - 2*(qx*qx + qy*qy)],
    ])
    return R


def quat_to_euler_deg(qw, qx, qy, qz):
    """Quaternion (w,x,y,z) → roll, pitch, yaw in degrees (XYZ convention)."""
    # Roll (x-axis)
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr, cosr)
    # Pitch (y-axis)
    sinp = 2 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # Yaw (z-axis)
    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny, cosy)
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

DT = 0.001

# =====================================================================
# Load data
# =====================================================================
ws = Path(__file__).resolve().parents[2]
output_dir = ws / "output_data"
esekf_file = output_dir / "walk_3m_01m_esekf.csv"

df = pd.read_csv(esekf_file)
N = len(df)
t = np.arange(N) * DT
print(f"Loaded {N} samples ({t[-1]:.1f}s)")

# =====================================================================
# 1. Attitude analysis: extract Euler angles from both quaternions
# =====================================================================
est_euler = np.zeros((N, 3))
gt_euler = np.zeros((N, 3))
for i in range(N):
    est_euler[i] = quat_to_euler_deg(df.est_qw.iloc[i], df.est_qx.iloc[i],
                                      df.est_qy.iloc[i], df.est_qz.iloc[i])
    gt_euler[i]  = quat_to_euler_deg(df.gt_qw.iloc[i], df.gt_qx.iloc[i],
                                      df.gt_qy.iloc[i], df.gt_qz.iloc[i])

print(f"\n--- Attitude (Euler degrees, roll/pitch/yaw) ---")
print(f"  EST roll  range: [{est_euler[:,0].min():.2f}, {est_euler[:,0].max():.2f}]")
print(f"  GT  roll  range: [{gt_euler[:,0].min():.2f},  {gt_euler[:,0].max():.2f}]")
print(f"  EST pitch range: [{est_euler[:,1].min():.2f}, {est_euler[:,1].max():.2f}]")
print(f"  GT  pitch range: [{gt_euler[:,1].min():.2f},  {gt_euler[:,1].max():.2f}]")
print(f"  EST yaw   range: [{est_euler[:,2].min():.2f}, {est_euler[:,2].max():.2f}]")
print(f"  GT  yaw   range: [{gt_euler[:,2].min():.2f},  {gt_euler[:,2].max():.2f}]")
print(f"  EST yaw   final: {est_euler[-1,2]:.2f}°")
print(f"  GT  yaw   final: {gt_euler[-1,2]:.2f}°")

# =====================================================================
# 2. GT velocity in WORLD frame (finite difference)
# =====================================================================
gt_vx_world = np.gradient(df.sim_pos_x.values, DT)
gt_vy_world = np.gradient(df.sim_pos_y.values, DT)
gt_vz_world = np.gradient(df.sim_pos_z.values, DT)

# Low-pass filter (10 Hz)
alpha_lpf = 1.0 - np.exp(-2.0 * np.pi * 10.0 * DT)
for i in range(1, N):
    gt_vx_world[i] = (1 - alpha_lpf) * gt_vx_world[i-1] + alpha_lpf * gt_vx_world[i]
    gt_vy_world[i] = (1 - alpha_lpf) * gt_vy_world[i-1] + alpha_lpf * gt_vy_world[i]
    gt_vz_world[i] = (1 - alpha_lpf) * gt_vz_world[i-1] + alpha_lpf * gt_vz_world[i]

# =====================================================================
# 3. GT velocity in BODY frame: v_body = R_GT^T * v_world
# =====================================================================
gt_v_body = np.zeros((N, 3))
for i in range(N):
    R_gt = quat_to_rotmat(df.gt_qw.iloc[i], df.gt_qx.iloc[i],
                           df.gt_qy.iloc[i], df.gt_qz.iloc[i])
    gt_v_body[i] = R_gt.T @ np.array([gt_vx_world[i], gt_vy_world[i], gt_vz_world[i]])

print(f"\n--- GT Velocity ---")
print(f"  World vx mean: {gt_vx_world.mean():.4f} m/s")
print(f"  Body  vx mean: {gt_v_body[:,0].mean():.4f} m/s")
print(f"  Body  vy mean: {gt_v_body[:,1].mean():.4f} m/s")
print(f"  Body  vz mean: {gt_v_body[:,2].mean():.4f} m/s")

# =====================================================================
# 4. ESEKF velocity in WORLD frame: v_world = R_est * v_body
# =====================================================================
est_v_body = np.column_stack([df.est_vel_x, df.est_vel_y, df.est_vel_z])
est_v_world = np.zeros((N, 3))
for i in range(N):
    R_est = quat_to_rotmat(df.est_qw.iloc[i], df.est_qx.iloc[i],
                            df.est_qy.iloc[i], df.est_qz.iloc[i])
    est_v_world[i] = R_est @ est_v_body[i]

# z_avg (leg observation, body frame)
z_avg = np.column_stack([df.z_avg_x, df.z_avg_y, df.z_avg_z])
# z_avg projected to world using GT attitude (best-case projection)
z_avg_world_gt = np.zeros((N, 3))
z_avg_world_est = np.zeros((N, 3))
for i in range(N):
    R_gt = quat_to_rotmat(df.gt_qw.iloc[i], df.gt_qx.iloc[i],
                           df.gt_qy.iloc[i], df.gt_qz.iloc[i])
    R_est = quat_to_rotmat(df.est_qw.iloc[i], df.est_qx.iloc[i],
                            df.est_qy.iloc[i], df.est_qz.iloc[i])
    z_avg_world_gt[i] = R_gt @ z_avg[i]
    z_avg_world_est[i] = R_est @ z_avg[i]

# =====================================================================
# 5. Body-frame velocity RMSEs (skip warmup)
# =====================================================================
SKIP = 2000  # 2 seconds warmup

def rmse(a, b, skip=SKIP):
    return np.sqrt(np.mean((a[skip:] - b[skip:])**2))

print(f"\n--- Body-frame Velocity RMSE (skip {SKIP} samples) ---")
print(f"  est_vel_x vs GT_body_vx:  {rmse(est_v_body[:,0], gt_v_body[:,0])*1000:.1f} mm/s")
print(f"  est_vel_y vs GT_body_vy:  {rmse(est_v_body[:,1], gt_v_body[:,1])*1000:.1f} mm/s")
print(f"  est_vel_z vs GT_body_vz:  {rmse(est_v_body[:,2], gt_v_body[:,2])*1000:.1f} mm/s")
print(f"  z_avg_x   vs GT_body_vx:  {rmse(z_avg[:,0], gt_v_body[:,0])*1000:.1f} mm/s")
print(f"  z_avg_y   vs GT_body_vy:  {rmse(z_avg[:,1], gt_v_body[:,1])*1000:.1f} mm/s")
print(f"  z_avg_z   vs GT_body_vz:  {rmse(z_avg[:,2], gt_v_body[:,2])*1000:.1f} mm/s")

print(f"\n--- World-frame Velocity RMSE ---")
print(f"  est_v_world_x vs GT:      {rmse(est_v_world[:,0], gt_vx_world)*1000:.1f} mm/s")
print(f"  est_v_world_y vs GT:      {rmse(est_v_world[:,1], gt_vy_world)*1000:.1f} mm/s")
print(f"  est_v_world_z vs GT:      {rmse(est_v_world[:,2], gt_vz_world)*1000:.1f} mm/s")
print(f"  z_avg_world(GT_q) vx:     {rmse(z_avg_world_gt[:,0], gt_vx_world)*1000:.1f} mm/s")
print(f"  z_avg_world(est_q) vx:    {rmse(z_avg_world_est[:,0], gt_vx_world)*1000:.1f} mm/s")

# =====================================================================
# 6. Pure leg integration position (using z_avg + GT attitude)
# =====================================================================
pure_leg_pos = np.zeros((N, 3))
pure_leg_pos_est_q = np.zeros((N, 3))
for i in range(1, N):
    # Using GT attitude
    R_gt = quat_to_rotmat(df.gt_qw.iloc[i], df.gt_qx.iloc[i],
                           df.gt_qy.iloc[i], df.gt_qz.iloc[i])
    pure_leg_pos[i] = pure_leg_pos[i-1] + R_gt @ z_avg[i] * DT
    # Using EST attitude
    R_est = quat_to_rotmat(df.est_qw.iloc[i], df.est_qx.iloc[i],
                            df.est_qy.iloc[i], df.est_qz.iloc[i])
    pure_leg_pos_est_q[i] = pure_leg_pos_est_q[i-1] + R_est @ z_avg[i] * DT

gt_pos = np.column_stack([df.sim_pos_x - df.sim_pos_x.iloc[0],
                           df.sim_pos_y - df.sim_pos_y.iloc[0],
                           df.sim_pos_z - df.sim_pos_z.iloc[0]])
est_pos = np.column_stack([df.est_pos_x, df.est_pos_y, df.est_pos_z])

print(f"\n--- Position (final) ---")
print(f"  GT:                      [{gt_pos[-1,0]:.3f}, {gt_pos[-1,1]:.3f}, {gt_pos[-1,2]:.3f}]")
print(f"  ESEKF:                   [{est_pos[-1,0]:.3f}, {est_pos[-1,1]:.3f}, {est_pos[-1,2]:.3f}]")
print(f"  Pure leg (GT attitude):  [{pure_leg_pos[-1,0]:.3f}, {pure_leg_pos[-1,1]:.3f}, {pure_leg_pos[-1,2]:.3f}]")
print(f"  Pure leg (EST attitude): [{pure_leg_pos_est_q[-1,0]:.3f}, {pure_leg_pos_est_q[-1,1]:.3f}, {pure_leg_pos_est_q[-1,2]:.3f}]")

print(f"\n--- Position RMSE ---")
print(f"  ESEKF x:             {rmse(est_pos[:,0], gt_pos[:,0])*1000:.0f} mm")
print(f"  Pure leg (GT q) x:   {rmse(pure_leg_pos[:,0], gt_pos[:,0])*1000:.0f} mm")
print(f"  Pure leg (EST q) x:  {rmse(pure_leg_pos_est_q[:,0], gt_pos[:,0])*1000:.0f} mm")

# =====================================================================
# 7. Yaw drift impact analysis
# =====================================================================
yaw_err_deg = est_euler[:, 2] - gt_euler[:, 2]
# Wrap to [-180, 180]
yaw_err_deg = (yaw_err_deg + 180) % 360 - 180

print(f"\n--- Yaw Error ---")
print(f"  Mean:  {yaw_err_deg[SKIP:].mean():.2f}°")
print(f"  Max:   {np.abs(yaw_err_deg[SKIP:]).max():.2f}°")
print(f"  Final: {yaw_err_deg[-1]:.2f}°")

# =====================================================================
# PLOTS
# =====================================================================
fig, axes = plt.subplots(4, 2, figsize=(18, 16))
fig.suptitle("ESEKF vs Pure Leg Odometry Diagnosis", fontsize=14)

# --- Body-frame velocity X ---
ax = axes[0, 0]
ax.plot(t, gt_v_body[:, 0], 'k-', lw=0.5, alpha=0.5, label='GT body vx')
ax.plot(t, z_avg[:, 0], 'b-', lw=0.3, alpha=0.5, label='z_avg_x (leg obs)')
ax.plot(t, est_v_body[:, 0], 'r-', lw=0.3, alpha=0.6, label='ESEKF est_vel_x')
ax.set_ylabel('vx [m/s]')
ax.set_title('Body-frame Velocity X')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# --- Body-frame velocity error X ---
ax = axes[0, 1]
ax.plot(t, (est_v_body[:, 0] - gt_v_body[:, 0]) * 1000, 'r-', lw=0.3, alpha=0.6, label='ESEKF - GT')
ax.plot(t, (z_avg[:, 0] - gt_v_body[:, 0]) * 1000, 'b-', lw=0.3, alpha=0.5, label='z_avg - GT')
ax.axhline(0, color='k', lw=0.5)
ax.set_ylabel('error [mm/s]')
ax.set_title('Body-frame Velocity X Error')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# --- Attitude (pitch & yaw) ---
ax = axes[1, 0]
ax.plot(t, gt_euler[:, 1], 'k-', lw=0.5, label='GT pitch')
ax.plot(t, est_euler[:, 1], 'r--', lw=0.5, label='EST pitch')
ax.set_ylabel('Pitch [°]')
ax.set_title('Pitch Comparison')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

ax = axes[1, 1]
ax.plot(t, gt_euler[:, 2], 'k-', lw=0.5, label='GT yaw')
ax.plot(t, est_euler[:, 2], 'r--', lw=0.5, label='EST yaw')
ax.set_ylabel('Yaw [°]')
ax.set_title('Yaw Comparison (unobservable)')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# --- Position X comparison ---
ax = axes[2, 0]
ax.plot(t, gt_pos[:, 0], 'k-', lw=1, label='GT')
ax.plot(t, est_pos[:, 0], 'r--', lw=0.8, label='ESEKF')
ax.plot(t, pure_leg_pos[:, 0], 'b--', lw=0.8, label='Pure leg (GT q)')
ax.plot(t, pure_leg_pos_est_q[:, 0], 'g--', lw=0.8, label='Pure leg (EST q)')
ax.set_ylabel('X [m]')
ax.set_title('Position X')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# --- Position Y comparison ---
ax = axes[2, 1]
ax.plot(t, gt_pos[:, 1], 'k-', lw=1, label='GT')
ax.plot(t, est_pos[:, 1], 'r--', lw=0.8, label='ESEKF')
ax.plot(t, pure_leg_pos[:, 1], 'b--', lw=0.8, label='Pure leg (GT q)')
ax.plot(t, pure_leg_pos_est_q[:, 1], 'g--', lw=0.8, label='Pure leg (EST q)')
ax.set_ylabel('Y [m]')
ax.set_title('Position Y')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# --- Biases ---
ax = axes[3, 0]
ax.plot(t, df.ba_x, 'r-', lw=0.5, label='ba_x')
ax.plot(t, df.ba_y, 'g-', lw=0.5, label='ba_y')
ax.plot(t, df.ba_z, 'b-', lw=0.5, label='ba_z')
ax.set_ylabel('ba [m/s²]')
ax.set_xlabel('Time [s]')
ax.set_title('Accelerometer Bias')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

ax = axes[3, 1]
ax.plot(t, df.bw_x, 'r-', lw=0.5, label='bw_x')
ax.plot(t, df.bw_y, 'g-', lw=0.5, label='bw_y')
ax.plot(t, df.bw_z, 'b-', lw=0.5, label='bw_z')
ax.set_ylabel('bw [rad/s]')
ax.set_xlabel('Time [s]')
ax.set_title('Gyroscope Bias')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out_fig = Path(__file__).resolve().parent / "esekf_vs_legs_diagnosis.png"
plt.savefig(out_fig, dpi=150)
print(f"\nPlot saved to {out_fig}")
plt.close()

# =====================================================================
# Summary
# =====================================================================
print(f"\n{'='*70}")
print(f"DIAGNOSIS SUMMARY")
print(f"{'='*70}")
print(f"Pure leg odometry (z_avg) body-frame vx RMSE : "
      f"{rmse(z_avg[:,0], gt_v_body[:,0])*1000:.1f} mm/s")
print(f"ESEKF body-frame vx RMSE                     : "
      f"{rmse(est_v_body[:,0], gt_v_body[:,0])*1000:.1f} mm/s")
print(f"ESEKF world-frame vx RMSE                    : "
      f"{rmse(est_v_world[:,0], gt_vx_world)*1000:.1f} mm/s")
print(f"")
print(f"Pure leg position (GT q) final x              : {pure_leg_pos[-1,0]:.3f} m")
print(f"Pure leg position (EST q) final x             : {pure_leg_pos_est_q[-1,0]:.3f} m")
print(f"ESEKF final x                                 : {est_pos[-1,0]:.3f} m")
print(f"GT final x                                    : {gt_pos[-1,0]:.3f} m")
print(f"")
print(f"Yaw drift (EST - GT) final                    : {yaw_err_deg[-1]:.2f}°")
print(f"ba_x final                                    : {df.ba_x.iloc[-1]:.4f} m/s²")
print(f"bw_z final (yaw bias)                         : {df.bw_z.iloc[-1]:.6f} rad/s")
print(f"{'='*70}")
