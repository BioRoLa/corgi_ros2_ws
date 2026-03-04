#!/usr/bin/env python3
"""
gt_vs_esekf.py
==============
9-panel comparison: Ground Truth vs ESEKF

  Row 1: Position   X, Y, Z
  Row 2: Velocity   X, Y, Z  (body frame)
  Row 3: Roll, Yaw, Pitch    (degrees)

Data source: output_data/walk_3m_01m_esekf.csv
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

# =====================================================================
# Configuration
# =====================================================================
DT          = 0.001   # 1 kHz
GT_LPF_HZ  = 10.0    # GT velocity low-pass cutoff (Hz)

# =====================================================================
# Quaternion helpers
# =====================================================================
def quat_to_rotmat(qw, qx, qy, qz):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),     1-2*(qx*qx+qy*qy)]])

def quat_to_euler_deg(qw, qx, qy, qz):
    """Return (roll, yaw, pitch) in degrees – vectorised over N-element arrays."""
    qw, qx, qy, qz = map(np.asarray, (qw, qx, qy, qz))
    roll  = np.degrees(np.arctan2(2*(qw*qx + qy*qz),
                                   1 - 2*(qx*qx + qy*qy)))
    sinp  = np.clip(2*(qw*qy - qz*qx), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))
    yaw   = np.degrees(np.arctan2(2*(qw*qz + qx*qy),
                                   1 - 2*(qy*qy + qz*qz)))
    return roll, yaw, pitch


# =====================================================================
# Main
# =====================================================================
def main():
    ws         = Path(__file__).resolve().parents[5]
    output_dir = ws / "output_data"
    fig_dir    = Path(__file__).resolve().parent

    esekf_file = output_dir / "walk_3m_01m_esekf.csv"
    print(f"Loading {esekf_file} …")
    df = pd.read_csv(esekf_file)
    N  = len(df)
    t  = np.arange(N) * DT
    print(f"  {N} samples  ({t[-1]:.1f} s)")

    # ------------------------------------------------------------------
    # Position  (offset-correct GT so both start at origin)
    # ------------------------------------------------------------------
    gt_off = [df.sim_pos_x.iloc[0], df.sim_pos_y.iloc[0], df.sim_pos_z.iloc[0]]
    gt_px = df.sim_pos_x.values - gt_off[0]
    gt_py = df.sim_pos_y.values - gt_off[1]
    gt_pz = df.sim_pos_z.values - gt_off[2]
    es_px = df.est_pos_x.values
    es_py = df.est_pos_y.values
    es_pz = df.est_pos_z.values

    # ------------------------------------------------------------------
    # GT velocity: world-frame FD → 10 Hz LPF → rotate to body frame
    # ------------------------------------------------------------------
    alpha = 1.0 - np.exp(-2 * np.pi * GT_LPF_HZ * DT)
    vw = np.column_stack([
        np.gradient(df.sim_pos_x.values, DT),
        np.gradient(df.sim_pos_y.values, DT),
        np.gradient(df.sim_pos_z.values, DT),
    ])
    for i in range(1, N):
        vw[i] = (1 - alpha) * vw[i-1] + alpha * vw[i]

    gt_vb = np.zeros((N, 3))
    for i in range(N):
        R = quat_to_rotmat(df.gt_qw.iloc[i], df.gt_qx.iloc[i],
                           df.gt_qy.iloc[i], df.gt_qz.iloc[i])
        gt_vb[i] = R.T @ vw[i]

    es_vx = df.est_vel_x.values
    es_vy = df.est_vel_y.values
    es_vz = df.est_vel_z.values

    # ------------------------------------------------------------------
    # Euler angles (degrees)
    # ------------------------------------------------------------------
    gt_roll, gt_yaw, gt_pitch = quat_to_euler_deg(df.gt_qw.values, df.gt_qx.values,
                                                    df.gt_qy.values, df.gt_qz.values)
    es_roll, es_yaw, es_pitch = quat_to_euler_deg(df.est_qw.values, df.est_qx.values,
                                                    df.est_qy.values, df.est_qz.values)

    # ------------------------------------------------------------------
    # Position RMSE (for titles)
    # ------------------------------------------------------------------
    rmse_px = np.sqrt(np.mean((es_px - gt_px)**2)) * 1000
    rmse_py = np.sqrt(np.mean((es_py - gt_py)**2)) * 1000
    rmse_pz = np.sqrt(np.mean((es_pz - gt_pz)**2)) * 1000

    warmup = 2000   # skip first 2 s for LPF settle (velocity eval)
    rmse_vx = np.sqrt(np.mean((es_vx[warmup:] - gt_vb[warmup:, 0])**2)) * 1000
    rmse_vy = np.sqrt(np.mean((es_vy[warmup:] - gt_vb[warmup:, 1])**2)) * 1000
    rmse_vz = np.sqrt(np.mean((es_vz[warmup:] - gt_vb[warmup:, 2])**2)) * 1000

    rmse_roll  = np.sqrt(np.mean((es_roll  - gt_roll )**2))
    rmse_yaw   = np.sqrt(np.mean((es_yaw   - gt_yaw  )**2))
    rmse_pitch = np.sqrt(np.mean((es_pitch - gt_pitch)**2))

    # ------------------------------------------------------------------
    # Output directory
    # ------------------------------------------------------------------
    out_dir = fig_dir / "gt_vs_esekf"
    out_dir.mkdir(parents=True, exist_ok=True)

    GT_STYLE  = dict(color='steelblue', lw=1.2, label='GT',    zorder=3)
    EST_STYLE = dict(color='tomato',    lw=0.8, label='ESEKF', linestyle='--', zorder=2, alpha=0.9)

    def _save(filename, x, gt, est, ylabel, title, rmse_str):
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.plot(x, gt,  **GT_STYLE)
        ax.plot(x, est, **EST_STYLE)
        ax.set_title(f"{title}  (RMSE={rmse_str})", fontsize=11)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_xlabel('Time [s]', fontsize=10)
        ax.legend(fontsize=9, loc='upper left')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(x[0], x[-1])
        plt.tight_layout()
        path = out_dir / filename
        plt.savefig(path, dpi=150)
        plt.close(fig)
        print(f"  Saved: {path.name}")

    # Position
    _save("pos_x.png", t, gt_px,      es_px,  'X [m]',    'Position X',        f'{rmse_px:.1f} mm')
    _save("pos_y.png", t, gt_py,      es_py,  'Y [m]',    'Position Y',        f'{rmse_py:.1f} mm')
    _save("pos_z.png", t, gt_pz,      es_pz,  'Z [m]',    'Position Z',        f'{rmse_pz:.1f} mm')

    # Velocity (body frame)
    _save("vel_x.png", t, gt_vb[:,0], es_vx,  'Vx [m/s]', 'Velocity X (body)', f'{rmse_vx:.1f} mm/s')
    _save("vel_y.png", t, gt_vb[:,1], es_vy,  'Vy [m/s]', 'Velocity Y (body)', f'{rmse_vy:.1f} mm/s')
    _save("vel_z.png", t, gt_vb[:,2], es_vz,  'Vz [m/s]', 'Velocity Z (body)', f'{rmse_vz:.1f} mm/s')

    # Euler angles
    _save("roll.png",  t, gt_roll,    es_roll,  'Roll [°]',  'Roll',  f'{rmse_roll:.2f}°')
    _save("yaw.png",   t, gt_yaw,     es_yaw,   'Yaw [°]',   'Yaw',   f'{rmse_yaw:.2f}°')
    _save("pitch.png", t, gt_pitch,   es_pitch, 'Pitch [°]', 'Pitch', f'{rmse_pitch:.2f}°')

    print(f"\nAll 9 plots saved to: {out_dir}/")

    # ------------------------------------------------------------------
    # Console summary
    # ------------------------------------------------------------------
    print("\n" + "="*55)
    print(f"{'Metric':<28} {'RMSE':>10}")
    print("-"*55)
    print(f"  Position X                 {rmse_px:>8.1f} mm")
    print(f"  Position Y                 {rmse_py:>8.1f} mm")
    print(f"  Position Z                 {rmse_pz:>8.1f} mm")
    print(f"  Velocity X (body, >2s)     {rmse_vx:>8.1f} mm/s")
    print(f"  Velocity Y (body, >2s)     {rmse_vy:>8.1f} mm/s")
    print(f"  Velocity Z (body, >2s)     {rmse_vz:>8.1f} mm/s")
    print(f"  Roll                       {rmse_roll:>8.2f} °")
    print(f"  Yaw                        {rmse_yaw:>8.2f} °")
    print(f"  Pitch                      {rmse_pitch:>8.2f} °")
    print("="*55)


if __name__ == "__main__":
    main()
