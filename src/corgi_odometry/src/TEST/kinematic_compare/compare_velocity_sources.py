#!/usr/bin/env python3
"""
compare_velocity_sources.py

Compare two approaches for computing theta_d / beta_d:
  A) Motor velocities: theta_d = (-vel_r + vel_l)/2, beta_d = (vel_r + vel_l)/2
  B) Position differentiation: theta_d = d/dt(state_theta), beta_d = d/dt(state_beta) + 30Hz LPF

Then compute z_leg using both and compare with ground truth velocity.
This identifies whether the velocity source is the root cause of ESEKF z_leg errors.
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add parent for shared code
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "contact_detection"))
from tune_schmitt_trigger import (
    linkleg_calculate, leg_calculate, contact_lookup,
    contact_velocity_noslip, schmitt_trigger,
    DT, START_INDEX, R_wheel, r_tire,
    OFFSET_X, OFFSET_Y, LEG_NAMES, LEG_SIGNS, LEG_LABELS,
)

def main():
    base = Path(__file__).resolve().parents[5]
    out_dir = Path(__file__).resolve().parent

    raw_csv  = base / "output_data" / "walk_3m_01m.csv"
    dist_csv = base / "output_data" / "walk_3m_01m_result.csv"

    print("Loading data...")
    raw  = pd.read_csv(raw_csv)
    dist = pd.read_csv(dist_csv)

    raw_aligned = raw.iloc[START_INDEX : START_INDEX + len(dist)].reset_index(drop=True)
    SKIP = 500
    raw_aligned = raw_aligned.iloc[SKIP:].reset_index(drop=True)
    dist        = dist.iloc[SKIP:].reset_index(drop=True)
    N = len(dist)
    t = np.arange(N) * DT
    print(f"  Aligned: {N} samples ({N*DT:.1f}s)")

    # ---- Ground truth velocity ----
    sim_x = raw_aligned["sim_pos_x"].values
    gt_vx = np.gradient(sim_x, DT)
    alpha_lpf = 1.0 - np.exp(-2.0 * np.pi * 5.0 * DT)
    for i in range(1, N):
        gt_vx[i] = (1 - alpha_lpf) * gt_vx[i-1] + alpha_lpf * gt_vx[i]

    # ---- IMU angular velocity ----
    w_body_y = raw_aligned["imu_ang_vel_y"].values

    # ---- Contact detection (use best config: OR rm=25, beta=10) ----
    rm_signals   = {n: dist[f"estimated_disturbance_rm_{n}"].values   for n in LEG_NAMES}
    beta_signals = {n: dist[f"estimated_disturbance_beta_{n}"].values for n in LEG_NAMES}
    leg_contacts = {}
    for name in LEG_NAMES:
        leg_contacts[name] = schmitt_trigger(
            rm_signals[name], beta_signals[name], 25, 15, 10, 1, use_and=False)

    # =====================================================================
    # Prepare two velocity sources per leg
    # =====================================================================
    alpha_enc = 1.0 - np.exp(-2.0 * np.pi * 30.0 * DT)

    leg_data_A = {}  # Method A: motor velocities
    leg_data_B = {}  # Method B: position differentiation + LPF

    for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS):
        theta = raw_aligned[f"state_theta_{name}"].values
        beta  = raw_aligned[f"state_beta_{name}"].values
        vel_r = raw_aligned[f"state_vel_r_{name}"].values
        vel_l = raw_aligned[f"state_vel_l_{name}"].values

        is_right = name in ["b", "c"]
        if is_right:
            beta = -beta

        offset = np.array([xs * OFFSET_X, ys * OFFSET_Y, 0.0])
        y_off  = ys * OFFSET_Y

        # ---- Method A: motor velocities (matching C++ offline_test) ----
        theta_d_A = (-vel_r + vel_l) / 2.0
        beta_d_A  = ( vel_r + vel_l) / 2.0
        if is_right:
            beta_d_A = -beta_d_A

        # ---- Method B: position differentiation + 30Hz LPF (matching Python) ----
        theta_d_B = np.gradient(theta, DT)
        beta_d_B  = np.gradient(beta, DT)
        for i in range(1, N):
            theta_d_B[i] = (1 - alpha_enc) * theta_d_B[i-1] + alpha_enc * theta_d_B[i]
            beta_d_B[i]  = (1 - alpha_enc) * beta_d_B[i-1]  + alpha_enc * beta_d_B[i]

        leg_data_A[name] = (theta, beta, theta_d_A, beta_d_A, offset, y_off)
        leg_data_B[name] = (theta, beta, theta_d_B, beta_d_B, offset, y_off)

    # =====================================================================
    # Compute z_leg with both methods
    # =====================================================================
    print("\nComputing z_leg for both velocity sources...")

    def compute_velocity(leg_data_dict, label):
        vx_est = np.zeros(N)
        n_contact = np.zeros(N)
        per_leg_vx = {name: np.full(N, np.nan) for name in LEG_NAMES}
        for i in range(N):
            vx_sum, cnt = 0.0, 0
            w = np.array([0.0, w_body_y[i], 0.0])
            for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS):
                if not leg_contacts[name][i]:
                    continue
                theta_arr, beta_arr, theta_d_arr, beta_d_arr, offset, y_off = leg_data_dict[name]
                try:
                    fk = leg_calculate(theta_arr[i], theta_d_arr[i], beta_arr[i], beta_d_arr[i])
                    rim = contact_lookup(theta_arr[i], beta_arr[i])
                    if rim == "NO_CONTACT":
                        continue
                    v_est = contact_velocity_noslip(fk, rim, 0.0, w, offset, y_off)
                    per_leg_vx[name][i] = v_est[0]
                    vx_sum += v_est[0]
                    cnt += 1
                except:
                    continue
            if cnt > 0:
                vx_est[i] = vx_sum / cnt
            n_contact[i] = cnt
        # LPF
        vx_filt = vx_est.copy()
        alpha_v = 1.0 - np.exp(-2.0 * np.pi * 5.0 * DT)
        for i in range(1, N):
            vx_filt[i] = (1 - alpha_v) * vx_filt[i-1] + alpha_v * vx_filt[i]
        return vx_filt, per_leg_vx, n_contact

    vx_A, per_leg_A, nA = compute_velocity(leg_data_A, "Motor vel")
    vx_B, per_leg_B, nB = compute_velocity(leg_data_B, "Pos diff")

    # =====================================================================
    # Statistics
    # =====================================================================
    eval_start = 2000
    err_A = vx_A[eval_start:] - gt_vx[eval_start:]
    err_B = vx_B[eval_start:] - gt_vx[eval_start:]
    rmse_A = np.sqrt(np.mean(err_A**2))
    rmse_B = np.sqrt(np.mean(err_B**2))
    mean_A = np.mean(vx_A[eval_start:])
    mean_B = np.mean(vx_B[eval_start:])
    mean_GT = np.mean(gt_vx[eval_start:])

    print(f"\n{'='*60}")
    print(f"  Method A (motor vel):   RMSE={rmse_A*1000:.1f} mm/s   mean={mean_A:.4f} m/s")
    print(f"  Method B (pos diff):    RMSE={rmse_B*1000:.1f} mm/s   mean={mean_B:.4f} m/s")
    print(f"  Ground truth:                              mean={mean_GT:.4f} m/s")
    print(f"{'='*60}")

    # ---- Per-leg velocity comparison at sample 0 (= START_INDEX+SKIP) ----
    sample_idx = 0
    print(f"\nPer-leg comparison at sample {sample_idx} (global idx {START_INDEX+SKIP}):")
    for name, (xs, ys) in zip(LEG_NAMES, LEG_SIGNS):
        _, _, td_A, bd_A, _, _ = leg_data_A[name]
        _, _, td_B, bd_B, _, _ = leg_data_B[name]
        theta_arr = leg_data_A[name][0]
        beta_arr  = leg_data_A[name][1]
        print(f"  Leg {name}: theta={theta_arr[sample_idx]:.4f}  beta={beta_arr[sample_idx]:.4f}")
        print(f"    theta_d: motor={td_A[sample_idx]:+.4f}  pos_diff={td_B[sample_idx]:+.4f}  diff={td_A[sample_idx]-td_B[sample_idx]:+.6f}")
        print(f"    beta_d:  motor={bd_A[sample_idx]:+.4f}  pos_diff={bd_B[sample_idx]:+.4f}  diff={bd_A[sample_idx]-bd_B[sample_idx]:+.6f}")

    # ---- Per-leg theta_d comparison statistics ----
    print(f"\nPer-leg theta_d / beta_d RMSE (motor_vel vs pos_diff):")
    for name in LEG_NAMES:
        _, _, td_A, bd_A, _, _ = leg_data_A[name]
        _, _, td_B, bd_B, _, _ = leg_data_B[name]
        td_rmse = np.sqrt(np.mean((td_A[eval_start:] - td_B[eval_start:])**2))
        bd_rmse = np.sqrt(np.mean((bd_A[eval_start:] - bd_B[eval_start:])**2))
        td_corr = np.corrcoef(td_A[eval_start:], td_B[eval_start:])[0,1]
        bd_corr = np.corrcoef(bd_A[eval_start:], bd_B[eval_start:])[0,1]
        print(f"  Leg {name}: theta_d RMSE={td_rmse:.4f} corr={td_corr:.4f}  "
              f"beta_d RMSE={bd_rmse:.4f} corr={bd_corr:.4f}")

    # ---- Check sign: scatter plot theta_d_A vs theta_d_B ----
    print(f"\nSign check: linear regression of theta_d_motor vs theta_d_pos_diff:")
    for name in LEG_NAMES:
        _, _, td_A, bd_A, _, _ = leg_data_A[name]
        _, _, td_B, bd_B, _, _ = leg_data_B[name]
        # Fit y = mx+b
        m_td, b_td = np.polyfit(td_B[eval_start:], td_A[eval_start:], 1)
        m_bd, b_bd = np.polyfit(bd_B[eval_start:], bd_A[eval_start:], 1)
        print(f"  Leg {name}: theta_d slope={m_td:+.4f} int={b_td:+.4f}  "
              f"beta_d slope={m_bd:+.4f} int={b_bd:+.4f}")

    # =====================================================================
    # Plot
    # =====================================================================
    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)
    fig.suptitle("z_leg velocity: Motor Velocities vs Position Differentiation", fontsize=14)

    ax = axes[0]
    ax.plot(t, gt_vx, "k-", linewidth=1.5, label="GT vx", alpha=0.7)
    ax.plot(t, vx_A, "r-", linewidth=0.8, label=f"Method A (motor vel) RMSE={rmse_A*1000:.1f}mm/s", alpha=0.7)
    ax.plot(t, vx_B, "b-", linewidth=0.8, label=f"Method B (pos diff) RMSE={rmse_B*1000:.1f}mm/s", alpha=0.7)
    ax.set_ylabel("Velocity x (m/s)")
    ax.legend(fontsize=9)
    ax.set_title("Filtered average z_leg_x estimate")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, (vx_A - gt_vx)*1000, "r-", linewidth=0.5, label="Method A error", alpha=0.6)
    ax.plot(t, (vx_B - gt_vx)*1000, "b-", linewidth=0.5, label="Method B error", alpha=0.6)
    ax.set_ylabel("Error (mm/s)")
    ax.legend(fontsize=9)
    ax.set_title("Error vs ground truth")
    ax.grid(True, alpha=0.3)

    # Per-leg raw vx for method A vs B (first contacted leg as example)
    ax = axes[2]
    for name, color in zip(LEG_NAMES, ["green", "red", "blue", "orange"]):
        valid = ~np.isnan(per_leg_A[name])
        if np.any(valid):
            ax.scatter(t[valid], per_leg_A[name][valid], s=0.2, c=color, alpha=0.3, label=f"{name} (motor)")
        valid_B = ~np.isnan(per_leg_B[name])
        if np.any(valid_B):
            ax.scatter(t[valid_B], per_leg_B[name][valid_B], s=0.2, c=color, alpha=0.3, marker="x")
    ax.plot(t, gt_vx, "k-", linewidth=1, alpha=0.5, label="GT")
    ax.set_ylabel("Per-leg vx (m/s)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Per-leg z_leg_x (circles=motor vel, x=pos diff)")
    ax.legend(fontsize=8, markerscale=5)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "compare_velocity_sources.png", dpi=150)
    print(f"\nPlot saved: {out_dir / 'compare_velocity_sources.png'}")

    # =====================================================================
    # Additional: theta_d time series comparison for one leg
    # =====================================================================
    fig2, axes2 = plt.subplots(4, 2, figsize=(18, 14), sharex=True)
    fig2.suptitle("theta_d and beta_d: Motor Vel vs Pos Diff per leg", fontsize=14)

    plot_range = slice(0, min(5000, N))
    tp = t[plot_range]

    for idx, (name, label) in enumerate(zip(LEG_NAMES, LEG_LABELS)):
        _, _, td_A, bd_A, _, _ = leg_data_A[name]
        _, _, td_B, bd_B, _, _ = leg_data_B[name]

        ax_td = axes2[idx, 0]
        ax_td.plot(tp, td_A[plot_range], "r-", linewidth=0.5, alpha=0.7, label="motor vel")
        ax_td.plot(tp, td_B[plot_range], "b-", linewidth=0.5, alpha=0.7, label="pos diff")
        ax_td.set_ylabel(f"{label}\ntheta_d (rad/s)")
        ax_td.legend(fontsize=7)
        ax_td.grid(True, alpha=0.3)

        ax_bd = axes2[idx, 1]
        ax_bd.plot(tp, bd_A[plot_range], "r-", linewidth=0.5, alpha=0.7, label="motor vel")
        ax_bd.plot(tp, bd_B[plot_range], "b-", linewidth=0.5, alpha=0.7, label="pos diff")
        ax_bd.set_ylabel(f"beta_d (rad/s)")
        ax_bd.legend(fontsize=7)
        ax_bd.grid(True, alpha=0.3)

    axes2[3, 0].set_xlabel("Time (s)")
    axes2[3, 1].set_xlabel("Time (s)")
    fig2.tight_layout()
    fig2.savefig(out_dir / "theta_beta_d_comparison.png", dpi=150)
    print(f"Plot saved: {out_dir / 'theta_beta_d_comparison.png'}")


if __name__ == "__main__":
    main()
