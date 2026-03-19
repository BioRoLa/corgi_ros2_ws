#!/usr/bin/env python3
"""
Parameter sweep for ESEKF offline_test.

Sweeps sigma_a_x, sigma_leg_x, sigma_leg_z, and mahalanobis_threshold.
Generates a markdown report with results ranked by position RMSE.

Usage:
    python3 param_sweep.py
"""

import subprocess
import re
import itertools
import math
from pathlib import Path
from datetime import datetime

# ─── Configuration ───────────────────────────────────────────────────
EXECUTABLE = Path(__file__).parent / "build" / "corgi_odometry" / "src" / "offline_test"

# Parameter grids (each sweep varies ONE parameter, others at baseline)
BASELINE = {
    "sigma_a_x": 1.0,
    "sigma_leg_x": 8.0,
    "sigma_leg_z": 1.2,
    "threshold": 16.27,
}

SWEEPS = {
    "sigma_a_x":   [0.01, 0.05, 0.1, 0.5, 1.0, 2.0, 5.0],
    "sigma_leg_x": [0.01, 0.05, 0.1, 0.5, 1.0, 2.0, 4.0, 8.0],
    "sigma_leg_z": [0.01, 0.05, 0.1, 0.5, 1.0, 1.2, 2.0, 4.0],
    "threshold":   [0.5, 1.0, 2.0, 4.0, 8.0, 16.27, 1e9],
}

# Output
OUTPUT_DIR = Path(__file__).parent / "output_data"
REPORT_PATH = OUTPUT_DIR / "param_sweep_report.md"

# ─── Helpers ─────────────────────────────────────────────────────────
POS_RE = re.compile(r"Position RMSE total:\s+([\d.]+)")
VEL_RE = re.compile(r"Velocity RMSE total:\s+([\d.]+)")
POS_VEC_RE = re.compile(r"Position RMSE \(m\):\s+\[([\d.]+),\s+([\d.]+),\s+([\d.]+)\]")
VEL_VEC_RE = re.compile(r"Velocity RMSE body\(m/s\):\[([\d.]+),\s+([\d.]+),\s+([\d.]+)\]")


def run_experiment(sigma_a_x, sigma_leg_x, sigma_leg_z, threshold):
    """Run offline_test with the given parameters. Returns parsed RMSE dict or None."""
    thr_arg = f"{threshold:.6f}" if threshold < 1e8 else "1e30"
    cmd = [
        str(EXECUTABLE),
        str(sigma_a_x),
        str(sigma_leg_x),
        str(sigma_leg_z),
        thr_arg,
        "1",  # quiet
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        stdout = result.stdout
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  ERROR: {e}")
        return None

    pos_m = POS_RE.search(stdout)
    vel_m = VEL_RE.search(stdout)
    pos_v = POS_VEC_RE.search(stdout)
    vel_v = VEL_VEC_RE.search(stdout)

    if not pos_m or not vel_m:
        print(f"  WARNING: Could not parse RMSE from output (exit={result.returncode})")
        if result.stderr:
            print(f"  stderr: {result.stderr[:200]}")
        return None

    return {
        "pos_rmse": float(pos_m.group(1)),
        "vel_rmse": float(vel_m.group(1)),
        "pos_x": float(pos_v.group(1)) if pos_v else float("nan"),
        "pos_y": float(pos_v.group(2)) if pos_v else float("nan"),
        "pos_z": float(pos_v.group(3)) if pos_v else float("nan"),
        "vel_x": float(vel_v.group(1)) if vel_v else float("nan"),
        "vel_y": float(vel_v.group(2)) if vel_v else float("nan"),
        "vel_z": float(vel_v.group(3)) if vel_v else float("nan"),
    }


def fmt_thr(v):
    """Format threshold for display."""
    return "∞" if v > 1e8 else f"{v:.2f}"


def main():
    if not EXECUTABLE.exists():
        print(f"ERROR: Executable not found: {EXECUTABLE}")
        print("Run: colcon build --packages-select corgi_odometry")
        return

    all_results = []  # list of (param_dict, rmse_dict)

    # ── 1. Individual parameter sweeps ──────────────────
    for param_name, values in SWEEPS.items():
        print(f"\n{'='*60}")
        print(f"Sweeping: {param_name}")
        print(f"{'='*60}")
        for val in values:
            params = dict(BASELINE)
            params[param_name] = val
            label = f"{param_name}={val}"
            print(f"  Running {label} ...", end=" ", flush=True)
            rmse = run_experiment(
                params["sigma_a_x"],
                params["sigma_leg_x"],
                params["sigma_leg_z"],
                params["threshold"],
            )
            if rmse:
                print(f"pos={rmse['pos_rmse']:.6f} m, vel={rmse['vel_rmse']:.6f} m/s")
                all_results.append((dict(params), rmse, param_name))
            else:
                print("FAILED")

    # ── 2. Grid search around promising regions ─────────
    # (top-2 sigma_leg_x) × (top-2 sigma_leg_z) × (top-2 sigma_a_x)
    def best_values(param_name, n=2):
        sub = [(p, r) for p, r, sweep in all_results if sweep == param_name]
        sub.sort(key=lambda x: x[1]["pos_rmse"])
        return list(dict.fromkeys(p[param_name] for p, _ in sub[:n]))

    best_leg_x = best_values("sigma_leg_x", 3)
    best_leg_z = best_values("sigma_leg_z", 3)
    best_a_x   = best_values("sigma_a_x", 2)

    print(f"\n{'='*60}")
    print(f"Grid search: sigma_leg_x={best_leg_x}, sigma_leg_z={best_leg_z}, sigma_a_x={best_a_x}")
    print(f"{'='*60}")

    for lx, lz, ax in itertools.product(best_leg_x, best_leg_z, best_a_x):
        params = dict(BASELINE)
        params["sigma_leg_x"] = lx
        params["sigma_leg_z"] = lz
        params["sigma_a_x"] = ax
        # Skip if already tested
        already = any(
            p["sigma_leg_x"] == lx and p["sigma_leg_z"] == lz and p["sigma_a_x"] == ax
            for p, _, _ in all_results
        )
        if already:
            continue
        label = f"ax={ax}, lx={lx}, lz={lz}"
        print(f"  Running {label} ...", end=" ", flush=True)
        rmse = run_experiment(ax, lx, lz, params["threshold"])
        if rmse:
            print(f"pos={rmse['pos_rmse']:.6f} m, vel={rmse['vel_rmse']:.6f} m/s")
            all_results.append((dict(params), rmse, "grid"))
        else:
            print("FAILED")

    # ── 3. Generate report ──────────────────────────────
    # Deduplicate by parameter tuple
    seen = set()
    unique_results = []
    for params, rmse, sweep in all_results:
        key = (params["sigma_a_x"], params["sigma_leg_x"], params["sigma_leg_z"], params["threshold"])
        if key not in seen:
            seen.add(key)
            unique_results.append((params, rmse, sweep))

    # Sort by pos_rmse
    unique_results.sort(key=lambda x: x[1]["pos_rmse"])

    # Find baseline result
    baseline_rmse = None
    for p, r, _ in unique_results:
        if (p["sigma_a_x"] == BASELINE["sigma_a_x"] and
            p["sigma_leg_x"] == BASELINE["sigma_leg_x"] and
            p["sigma_leg_z"] == BASELINE["sigma_leg_z"] and
            p["threshold"] == BASELINE["threshold"]):
            baseline_rmse = r
            break

    with open(REPORT_PATH, "w") as f:
        f.write("# ESEKF 參數掃描結果報告\n\n")
        f.write(f"生成時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("## 基準參數\n\n")
        f.write(f"| 參數 | 值 |\n|------|----|\n")
        f.write(f"| sigma_a_x | {BASELINE['sigma_a_x']} |\n")
        f.write(f"| sigma_leg_x | {BASELINE['sigma_leg_x']} |\n")
        f.write(f"| sigma_leg_z | {BASELINE['sigma_leg_z']} |\n")
        f.write(f"| mahalanobis_threshold | {BASELINE['threshold']} |\n\n")

        if baseline_rmse:
            f.write(f"基準 RMSE: 位置 = {baseline_rmse['pos_rmse']:.6f} m, "
                    f"速度 = {baseline_rmse['vel_rmse']:.6f} m/s\n\n")

        f.write("---\n\n")

        # ── Per-parameter sweep tables ──
        for param_name in SWEEPS:
            sub = [(p, r) for p, r, s in unique_results if s == param_name]
            if not sub:
                continue
            sub.sort(key=lambda x: x[0][param_name])
            f.write(f"## {param_name} 掃描\n\n")
            f.write(f"固定: " + ", ".join(
                f"{k}={v}" for k, v in BASELINE.items() if k != param_name) + "\n\n")

            header_val = "threshold" if param_name == "threshold" else param_name
            f.write(f"| {header_val} | Pos RMSE (m) | Vel RMSE (m/s) | Pos X | Pos Y | Pos Z | Vel X | Vel Y | Vel Z |\n")
            f.write(f"|---|---|---|---|---|---|---|---|---|\n")
            for p, r in sub:
                val = fmt_thr(p[param_name]) if param_name == "threshold" else f"{p[param_name]}"
                f.write(f"| {val} | {r['pos_rmse']:.6f} | {r['vel_rmse']:.6f} "
                        f"| {r['pos_x']:.6f} | {r['pos_y']:.6f} | {r['pos_z']:.6f} "
                        f"| {r['vel_x']:.6f} | {r['vel_y']:.6f} | {r['vel_z']:.6f} |\n")
            f.write("\n")

        # ── Grid search results ──
        grid_results = [(p, r) for p, r, s in unique_results if s == "grid"]
        if grid_results:
            f.write("## Grid Search 組合結果\n\n")
            f.write("| sigma_a_x | sigma_leg_x | sigma_leg_z | Pos RMSE (m) | Vel RMSE (m/s) |\n")
            f.write("|---|---|---|---|---|\n")
            grid_results.sort(key=lambda x: x[1]["pos_rmse"])
            for p, r in grid_results:
                f.write(f"| {p['sigma_a_x']} | {p['sigma_leg_x']} | {p['sigma_leg_z']} "
                        f"| {r['pos_rmse']:.6f} | {r['vel_rmse']:.6f} |\n")
            f.write("\n")

        # ── Overall Top-10 ──
        f.write("## 總排名（位置 RMSE 最佳前 10）\n\n")
        f.write("| Rank | sigma_a_x | sigma_leg_x | sigma_leg_z | threshold | Pos RMSE (m) | Vel RMSE (m/s) | Δ Pos% |\n")
        f.write("|------|-----------|-------------|-------------|-----------|-------------|----------------|--------|\n")
        for rank, (p, r, _) in enumerate(unique_results[:10], 1):
            delta = ""
            if baseline_rmse and baseline_rmse["pos_rmse"] > 0:
                pct = (r["pos_rmse"] - baseline_rmse["pos_rmse"]) / baseline_rmse["pos_rmse"] * 100
                delta = f"{pct:+.1f}%"
            f.write(f"| {rank} | {p['sigma_a_x']} | {p['sigma_leg_x']} | {p['sigma_leg_z']} "
                    f"| {fmt_thr(p['threshold'])} | {r['pos_rmse']:.6f} | {r['vel_rmse']:.6f} | {delta} |\n")

        # ── Impact summary ──
        f.write("\n## 參數影響程度分析\n\n")
        for param_name in SWEEPS:
            sub = [(p, r) for p, r, s in unique_results if s == param_name]
            if len(sub) < 2:
                continue
            pos_vals = [r["pos_rmse"] for _, r in sub]
            f.write(f"### {param_name}\n\n")
            f.write(f"- RMSE 範圍: {min(pos_vals):.6f} ~ {max(pos_vals):.6f} m\n")
            f.write(f"- 最大變化幅度: {max(pos_vals) - min(pos_vals):.6f} m ({(max(pos_vals)/min(pos_vals) - 1)*100:.1f}%)\n")
            best_p = min(sub, key=lambda x: x[1]["pos_rmse"])[0][param_name]
            f.write(f"- 最佳值: {fmt_thr(best_p) if param_name == 'threshold' else best_p}\n\n")

        f.write("---\n\n")
        f.write("## 結論\n\n")
        if unique_results:
            best = unique_results[0]
            f.write(f"**最佳參數組合**:\n")
            f.write(f"- sigma_a_x = {best[0]['sigma_a_x']}\n")
            f.write(f"- sigma_leg_x = {best[0]['sigma_leg_x']}\n")
            f.write(f"- sigma_leg_z = {best[0]['sigma_leg_z']}\n")
            f.write(f"- threshold = {fmt_thr(best[0]['threshold'])}\n")
            f.write(f"- **位置 RMSE = {best[1]['pos_rmse']:.6f} m**\n")
            f.write(f"- **速度 RMSE = {best[1]['vel_rmse']:.6f} m/s**\n")

    print(f"\n✓ Report saved to: {REPORT_PATH}")
    print(f"  Total experiments: {len(unique_results)}")
    if unique_results:
        best = unique_results[0]
        print(f"  Best: pos_rmse={best[1]['pos_rmse']:.6f} m "
              f"(σ_a_x={best[0]['sigma_a_x']}, σ_leg_x={best[0]['sigma_leg_x']}, "
              f"σ_leg_z={best[0]['sigma_leg_z']}, thr={fmt_thr(best[0]['threshold'])})")


if __name__ == "__main__":
    main()
