#!/usr/bin/env python3
"""
Plot kinematic_test results: PointVelocity vs DP::z()-style velocity.

Usage:
    python3 plot_kinematic_test.py [result_csv_path]

Default: ../../../output_data/kinematic_result_a.csv
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

def main():
    default_csv = os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..", "output_data", "kinematic_result_a.csv"
    )
    csv_path = sys.argv[1] if len(sys.argv) >= 2 else default_csv
    csv_path = os.path.abspath(csv_path)

    if not os.path.isfile(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    print(f"Loading: {csv_path}")
    data = np.genfromtxt(csv_path, delimiter=",", names=True)

    row   = data["row"]
    dt    = 0.001  # Config::DT
    time  = (row - row[0]) * dt  # approximate time axis

    pv_x  = data["pv_x"]
    pv_y  = data["pv_y"]
    pv_z  = data["pv_z"]
    dpz_x = data["dpz_x"]
    dpz_y = data["dpz_y"]
    dpz_z = data["dpz_z"]
    rim   = data["rim"]

    # ============================================================
    # Figure 1: Velocity comparison (x, y, z)
    # ============================================================
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    labels = ["x (forward)", "y (lateral)", "z (vertical)"]
    pv_vals  = [pv_x, pv_y, pv_z]
    dpz_vals = [dpz_x, dpz_y, dpz_z]

    for i, ax in enumerate(axes):
        ax.plot(time, pv_vals[i],  label="PointVelocity", alpha=0.8, linewidth=0.6)
        ax.plot(time, dpz_vals[i], label="DP::z()",       alpha=0.8, linewidth=0.6)
        ax.set_ylabel(f"vel {labels[i]} [m/s]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("PointVelocity vs DP::z() Velocity", fontsize=13)
    fig.tight_layout()

    # ============================================================
    # Figure 2: Difference (PV - DPz)
    # ============================================================
    fig2, axes2 = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    diff = [pv_x - dpz_x, pv_y - dpz_y, pv_z - dpz_z]

    for i, ax in enumerate(axes2):
        ax.plot(time, diff[i], color="red", alpha=0.7, linewidth=0.6)
        ax.set_ylabel(f"Δ {labels[i]} [m/s]")
        ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
        ax.grid(True, alpha=0.3)
        # Show mean ± std
        mu = np.mean(diff[i])
        sigma = np.std(diff[i])
        ax.set_title(f"mean={mu:.6f}, std={sigma:.6f}", fontsize=9, loc="right")

    axes2[-1].set_xlabel("Time [s]")
    fig2.suptitle("Difference (PointVelocity − DP::z())", fontsize=13)
    fig2.tight_layout()

    # ============================================================
    # Figure 3: RIM state + velocity magnitude
    # ============================================================
    fig3, axes3 = plt.subplots(2, 1, figsize=(14, 6), sharex=True,
                               gridspec_kw={"height_ratios": [1, 2]})

    # RIM state
    rim_names = {0: "NO_CT", 1: "UP_L", 2: "LO_L", 3: "G_PNT", 4: "LO_R", 5: "UP_R"}
    axes3[0].plot(time, rim, drawstyle="steps-post", color="purple", linewidth=0.8)
    axes3[0].set_ylabel("RIM")
    axes3[0].set_yticks(list(rim_names.keys()))
    axes3[0].set_yticklabels(list(rim_names.values()), fontsize=7)
    axes3[0].grid(True, alpha=0.3)

    # Velocity magnitude
    pv_mag  = np.sqrt(pv_x**2 + pv_y**2 + pv_z**2)
    dpz_mag = np.sqrt(dpz_x**2 + dpz_y**2 + dpz_z**2)
    axes3[1].plot(time, pv_mag,  label="|PointVelocity|", alpha=0.8, linewidth=0.6)
    axes3[1].plot(time, dpz_mag, label="|DP::z()|",       alpha=0.8, linewidth=0.6)
    axes3[1].set_ylabel("Speed [m/s]")
    axes3[1].set_xlabel("Time [s]")
    axes3[1].legend(loc="upper right", fontsize=8)
    axes3[1].grid(True, alpha=0.3)

    fig3.suptitle("RIM State & Velocity Magnitude", fontsize=13)
    fig3.tight_layout()

    # ============================================================
    # Figure 4: Scatter PV vs DPz for each axis
    # ============================================================
    fig4, axes4 = plt.subplots(1, 3, figsize=(14, 4.5))
    for i, ax in enumerate(axes4):
        ax.scatter(dpz_vals[i], pv_vals[i], s=0.3, alpha=0.3)
        lims = [min(ax.get_xlim()[0], ax.get_ylim()[0]),
                max(ax.get_xlim()[1], ax.get_ylim()[1])]
        ax.plot(lims, lims, "r--", linewidth=0.8, label="y=x")
        ax.set_xlabel(f"DP::z() {labels[i]}")
        ax.set_ylabel(f"PV {labels[i]}")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig4.suptitle("PointVelocity vs DP::z() Scatter", fontsize=13)
    fig4.tight_layout()

    # Save figures
    out_dir = os.path.dirname(csv_path)
    base = os.path.splitext(os.path.basename(csv_path))[0]
    for i, fig_obj in enumerate([fig, fig2, fig3, fig4], 1):
        out_path = os.path.join(out_dir, f"{base}_fig{i}.png")
        fig_obj.savefig(out_path, dpi=150)
        print(f"Saved: {out_path}")

    plt.show()


if __name__ == "__main__":
    main()
