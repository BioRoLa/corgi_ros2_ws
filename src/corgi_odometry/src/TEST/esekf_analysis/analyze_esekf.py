"""
Analyze ESEKF offline test results vs ground truth.
Usage:  python3 analyze_esekf.py
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Locate output
ws = Path(__file__).resolve().parents[2]  # corgi_ros2_ws/src -> corgi_ros2_ws
output_dir = ws / "output_data"
esekf_file = output_dir / "walk_3m_01m_esekf.csv"

if not esekf_file.exists():
    print(f"File not found: {esekf_file}")
    sys.exit(1)

df = pd.read_csv(esekf_file)
N = len(df)
dt = 0.001
t = np.arange(N) * dt

print(f"Loaded {N} samples ({t[-1]:.1f}s)")
print(f"\n--- Ground Truth ---")
print(f"  Start: ({df.sim_pos_x.iloc[0]:.4f}, {df.sim_pos_y.iloc[0]:.4f}, {df.sim_pos_z.iloc[0]:.4f})")
print(f"  End:   ({df.sim_pos_x.iloc[-1]:.4f}, {df.sim_pos_y.iloc[-1]:.4f}, {df.sim_pos_z.iloc[-1]:.4f})")
print(f"\n--- ESEKF Estimate ---")
print(f"  Start: ({df.est_pos_x.iloc[0]:.4f}, {df.est_pos_y.iloc[0]:.4f}, {df.est_pos_z.iloc[0]:.4f})")
print(f"  End:   ({df.est_pos_x.iloc[-1]:.4f}, {df.est_pos_y.iloc[-1]:.4f}, {df.est_pos_z.iloc[-1]:.4f})")

# Position errors
err_x = df.est_pos_x.values - df.sim_pos_x.values
err_y = df.est_pos_y.values - df.sim_pos_y.values
err_z = df.est_pos_z.values - df.sim_pos_z.values
err_norm = np.sqrt(err_x**2 + err_y**2 + err_z**2)
print(f"\n--- Position Error ---")
print(f"  Final |e|:     {err_norm[-1]:.4f} m")
print(f"  Max   |e|:     {err_norm.max():.4f} m")
print(f"  RMSE  (3D):    {np.sqrt(np.mean(err_norm**2)):.4f} m")
print(f"  RMSE  x:       {np.sqrt(np.mean(err_x**2)):.4f} m")
print(f"  RMSE  y:       {np.sqrt(np.mean(err_y**2)):.4f} m")
print(f"  RMSE  z:       {np.sqrt(np.mean(err_z**2)):.4f} m")

# Velocity stats
print(f"\n--- Velocity Stats ---")
print(f"  Mean vx:  {df.est_vel_x.mean():.6f} m/s")
print(f"  Mean vy:  {df.est_vel_y.mean():.6f} m/s")
print(f"  Mean vz:  {df.est_vel_z.mean():.6f} m/s")

# Contact stats
total_contacts = df[['contact_a','contact_b','contact_c','contact_d']].sum(axis=1)
print(f"\n--- Contact Stats ---")
print(f"  Mean legs in contact: {total_contacts.mean():.2f}")
print(f"  All 4 in contact: {(total_contacts==4).mean()*100:.1f}%")
print(f"  0 in contact: {(total_contacts==0).mean()*100:.1f}%")

# ============================================================
# Plots
# ============================================================
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('ESEKF Offline Test Results', fontsize=14)

# Position X
ax = axes[0, 0]
ax.plot(t, df.sim_pos_x, 'b-', label='Ground Truth', alpha=0.8)
ax.plot(t, df.est_pos_x, 'r--', label='ESEKF', alpha=0.8)
ax.set_ylabel('Position X [m]')
ax.legend()
ax.set_title('Position X')
ax.grid(True, alpha=0.3)

# Position Y
ax = axes[1, 0]
ax.plot(t, df.sim_pos_y, 'b-', label='Ground Truth', alpha=0.8)
ax.plot(t, df.est_pos_y, 'r--', label='ESEKF', alpha=0.8)
ax.set_ylabel('Position Y [m]')
ax.legend()
ax.set_title('Position Y')
ax.grid(True, alpha=0.3)

# Position Z
ax = axes[2, 0]
ax.plot(t, df.sim_pos_z, 'b-', label='Ground Truth', alpha=0.8)
ax.plot(t, df.est_pos_z, 'r--', label='ESEKF', alpha=0.8)
ax.set_ylabel('Position Z [m]')
ax.set_xlabel('Time [s]')
ax.legend()
ax.set_title('Position Z')
ax.grid(True, alpha=0.3)

# Position Error
ax = axes[0, 1]
ax.plot(t, err_x, label='err_x', alpha=0.7)
ax.plot(t, err_y, label='err_y', alpha=0.7)
ax.plot(t, err_z, label='err_z', alpha=0.7)
ax.plot(t, err_norm, 'k-', label='|err|', alpha=0.5)
ax.set_ylabel('Error [m]')
ax.legend()
ax.set_title('Position Error')
ax.grid(True, alpha=0.3)

# Velocity
ax = axes[1, 1]
ax.plot(t, df.est_vel_x, label='vx', alpha=0.7)
ax.plot(t, df.est_vel_y, label='vy', alpha=0.7)
ax.plot(t, df.est_vel_z, label='vz', alpha=0.7)
ax.set_ylabel('Body Velocity [m/s]')
ax.legend()
ax.set_title('Estimated Velocity (body frame)')
ax.grid(True, alpha=0.3)

# Contact
ax = axes[2, 1]
for idx, leg in enumerate(['a', 'b', 'c', 'd']):
    ax.fill_between(t, idx, idx + df[f'contact_{leg}'].values * 0.8,
                    alpha=0.5, label=f'Leg {leg.upper()}')
ax.set_yticks([0.4, 1.4, 2.4, 3.4])
ax.set_yticklabels(['LF', 'RF', 'RH', 'LH'])
ax.set_xlabel('Time [s]')
ax.set_title('Contact Detection')
ax.grid(True, alpha=0.3)

plt.tight_layout()
out_fig = output_dir / "esekf_analysis.png"
plt.savefig(out_fig, dpi=150)
print(f"\nPlot saved to {out_fig}")
plt.close()
