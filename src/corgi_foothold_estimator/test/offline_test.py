#!/usr/bin/env python3
"""
offline_test.py — Offline replay of FootholdDetector using recorded CSV data.

Replays each row of the CSV as if it were a real-time message stream, then
visualises per-leg contact state, swing phase, and kinematic void-detection
flags on a shared time-axis plot.

Usage:
    python3 offline_test.py [path/to/csv]

Defaults to:
    ~/corgi_ws/corgi_ros2_ws/output_data/walk_on_unflat_h25_v30_sl30_sh4.csv

Parameters (edit the PARAMS dict below):
    kinematic_delay_threshold  (s)      – post-landing contact timeout

Output:
    <csv_dir>/<csv_stem>_detection.png
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")           # headless; switch to "TkAgg" for an interactive window
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# =============================================================================
# Detection parameters  (mirror the ROS 2 node defaults)
# =============================================================================
PARAMS = {
    "kinematic_delay_threshold": 0.005,   # seconds
}

# Plot only the first PLOT_DURATION seconds of data (set to None to plot all).
PLOT_DURATION: float | None = 5.0        # seconds

LEG_NAMES = ["A (LF)", "B (RF)", "C (RH)", "D (LH)"]
LEG_KEYS  = ["a",      "b",      "c",      "d"]

# =============================================================================
# Python mirror of C++ FootholdDetector
# =============================================================================

class FootholdDetector:
    """
    Pure-Python replica of the C++ FootholdDetector class.
    Kinematic detection only (timing + contact latch).
    """

    def __init__(
        self,
        kinematic_delay_threshold: float = PARAMS["kinematic_delay_threshold"],
    ) -> None:
        self.kin_thr   = kinematic_delay_threshold
        self._contact_confirmed = False

    def notify_swing_started(self) -> None:
        """Reset the contact-confirmed latch; call on every swing rising edge."""
        self._contact_confirmed = False

    def evaluate_kinematic_only(self, tse: float, is_contact: bool) -> bool:
        """Kinematic void check — active only until first confirmed contact after landing."""
        if self._contact_confirmed:
            return False
        if tse <= 0.0:
            return False
        if is_contact:
            self._contact_confirmed = True
            return False
        return tse > self.kin_thr


# =============================================================================
# Simulation driver
# =============================================================================

def run_simulation(
    df: pd.DataFrame,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Replay the DataFrame row-by-row through the FootholdDetector logic.

    Returns
    -------
    time          : (N,)   float  – simulation time in seconds
    swing_phase   : (N, 4) int    – per-leg swing phase (0=stance, 1=swing)
    contact       : (N, 4) int    – per-leg physical contact (1=contact, 0=none)
    flag_kinematic: (N, 4) int    – per-leg kinematic void flag
    """
    N = len(df)

    time        = df["Time"].to_numpy(dtype=float)
    swing_phase = np.zeros((N, 4), dtype=int)
    contact     = np.zeros((N, 4), dtype=int)
    flag_kin    = np.zeros((N, 4), dtype=int)

    for i, key in enumerate(LEG_KEYS):
        swing_phase[:, i] = df[f"swing_phase_{key}"].to_numpy(dtype=int)
        contact[:, i]     = df[f"sim_contact_state_{key}"].to_numpy(dtype=int)

    # One independent detector instance per leg (kinematic only).
    det_kin = [FootholdDetector() for _ in range(4)]

    # Per-leg bookkeeping (mirrors the ROS callback logic).
    last_swing_end_time = [time[0]] * 4
    prev_swing          = [0] * 4
    is_active           = False

    for row in range(N):
        t = time[row]

        # --- Swing phase update (mirrors swing_phase_sub_ callback) ---
        for i in range(4):
            cur = swing_phase[row, i]
            if cur == 1:
                is_active = True
            # Rising edge: stance (0) → swing (1): arm kinematic detection.
            if prev_swing[i] == 0 and cur == 1:
                det_kin[i].notify_swing_started()
            # Falling edge: swing (1) → stance (0): record landing time.
            if prev_swing[i] == 1 and cur == 0:
                last_swing_end_time[i] = t
            prev_swing[i] = cur

        if not is_active:
            continue    # hold flags at 0 until walking starts

        # --- Per-leg evaluation (mirrors timer_callback) ---
        for i in range(4):
            tse = t - last_swing_end_time[i]
            if swing_phase[row, i] == 1:
                tse = 0.0       # leg is still airborne

            is_contact = bool(contact[row, i])

            # Kinematic
            flag_kin[row, i] = int(
                det_kin[i].evaluate_kinematic_only(tse, is_contact))

    return time, swing_phase, contact, flag_kin


# =============================================================================
# Plotting
# =============================================================================

# Visual lane layout for binary signals inside each leg subplot.
SIGNAL_COLORS = {
    "contact":   "#2196F3",   # blue
    "kinematic": "#FF9800",   # orange
}
LANE_GAP    = 1.3   # vertical distance between lane baselines
LANE_HEIGHT = 0.9   # height of a filled binary pulse (0→1)


def plot_results(
    time:           np.ndarray,
    swing_phase:    np.ndarray,
    contact:        np.ndarray,
    flag_kin:       np.ndarray,
    title:          str,
    output_path:    str,
) -> None:
    fig, axs = plt.subplots(4, 1, figsize=(20, 11), sharex=True)
    fig.suptitle(title, fontsize=13, fontweight="bold")

    n_lanes = len(SIGNAL_COLORS)
    max_y   = (n_lanes - 1) * LANE_GAP + LANE_HEIGHT + 0.3

    for leg_idx, ax in enumerate(axs):
        sp  = swing_phase[:, leg_idx]
        sig = [
            contact  [:, leg_idx],
            flag_kin [:, leg_idx],
        ]

        # ── Background: stance (swing=0) = light blue, swing (swing=1) = light orange ──
        ax.fill_between(
            time, 0, max_y, where=(sp == 0),
            step="post", alpha=0.10, color="royalblue")
        ax.fill_between(
            time, 0, max_y, where=(sp == 1),
            step="post", alpha=0.10, color="tomato")

        # ── Binary signal lanes ──
        for lane, (sig_arr, (name, color)) in enumerate(
                zip(sig, SIGNAL_COLORS.items())):
            baseline = lane * LANE_GAP
            vals     = sig_arr.astype(float) * LANE_HEIGHT

            ax.fill_between(
                time, baseline, baseline + vals,
                step="post", color=color, alpha=0.70, linewidth=0)
            ax.step(
                time, baseline + vals,
                color=color, linewidth=0.8, where="post")
            # Zero-level reference line
            ax.axhline(baseline, color=color, linewidth=0.35, alpha=0.35, linestyle="--")

        # ── Axes formatting ──
        tick_pos = [lane * LANE_GAP + LANE_HEIGHT * 0.5
                    for lane in range(n_lanes)]
        ax.set_yticks(tick_pos)
        ax.set_yticklabels(list(SIGNAL_COLORS.keys()), fontsize=8)
        ax.set_ylim(-0.15, max_y)
        ax.set_ylabel(f"Leg {LEG_NAMES[leg_idx]}", fontsize=9, labelpad=4)
        ax.grid(axis="x", alpha=0.25, linestyle="--")
        ax.tick_params(axis="x", labelbottom=(leg_idx == 3))

    axs[-1].set_xlabel("Time (s)", fontsize=10)

    # ── Legend ──
    signal_patches = [
        mpatches.Patch(color=c, alpha=0.75, label=name)
        for name, c in SIGNAL_COLORS.items()
    ]
    bg_patches = [
        mpatches.Patch(color="royalblue", alpha=0.30, label="stance phase (bg)"),
        mpatches.Patch(color="tomato",    alpha=0.30, label="swing phase (bg)"),
    ]
    axs[0].legend(
        handles=signal_patches + bg_patches,
        loc="upper right", fontsize=7.5, ncol=3, framealpha=0.85)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"[offline_test] Figure saved → {output_path}")


# =============================================================================
# Entry point
# =============================================================================

DEFAULT_CSV = os.path.join(
    os.path.expanduser("~"),
    "corgi_ws", "corgi_ros2_ws", "output_data",
    "walk_on_flat_h25_v30_sl30_sh4.csv",
)

REQUIRED_COLS = [
    "Time",
    *[f"sim_contact_state_{k}" for k in LEG_KEYS],
    *[f"swing_phase_{k}"       for k in LEG_KEYS],
]


def main() -> None:
    csv_path = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV)

    if not os.path.isfile(csv_path):
        print(f"[offline_test] ERROR: CSV not found: {csv_path}")
        sys.exit(1)

    print(f"[offline_test] Loading: {csv_path}")
    df = pd.read_csv(csv_path)

    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        print(f"[offline_test] ERROR: Missing columns: {missing}")
        sys.exit(1)

    # Forward-fill any gaps in required columns, then default remaining NaN to 0.
    df[REQUIRED_COLS] = df[REQUIRED_COLS].ffill().fillna(0)

    # Trim to the first PLOT_DURATION seconds of the recording.
    if PLOT_DURATION is not None:
        t_start = df["Time"].iloc[0]
        df = df[df["Time"] <= t_start + PLOT_DURATION].reset_index(drop=True)

    t0, t1 = df["Time"].iloc[0], df["Time"].iloc[-1]
    print(f"[offline_test] Rows: {len(df)},  Time: [{t0:.3f}, {t1:.3f}] s")
    print(f"[offline_test] Parameters: {PARAMS}")

    time, swing_phase, contact, flag_kin = run_simulation(df)

    # Convert to relative time (t=0 at the first row).
    time = time - time[0]

    csv_stem   = Path(csv_path).stem
    output_dir = Path(csv_path).parent
    output_png = str(output_dir / f"{csv_stem}_detection.png")

    plot_results(
        time, swing_phase, contact, flag_kin,
        title=f"Per-Leg Foothold Void Detection — {csv_stem}",
        output_path=output_png,
    )


if __name__ == "__main__":
    main()
