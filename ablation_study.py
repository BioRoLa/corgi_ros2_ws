#!/usr/bin/env python3
"""
Mahalanobis Distance Ablation Study — ESEKF on walk_obs.csv

Three noise-parameter groups, each swept across 7 threshold values.
Per experiment: RMSE (pos/vel) + D² statistics + rejection counts.
"""

import subprocess, re, csv, math, statistics
from pathlib import Path
from datetime import datetime

# ── Paths ────────────────────────────────────────────────────────────
ROOT       = Path(__file__).parent
EXECUTABLE = ROOT / "build" / "corgi_odometry" / "src" / "offline_test"
OUTPUT_DIR = ROOT / "output_data"
ESEKF_CSV  = OUTPUT_DIR / "walk_obs_esekf.csv"
REPORT     = OUTPUT_DIR / "mahalanobis_ablation_report.md"

# ── Experiment design ────────────────────────────────────────────────
# Three noise configs that differ in how much the filter trusts leg FK.
# Within each config, we sweep only the Mahalanobis threshold.
GROUPS = [
    {
        "name": "Baseline\n(σ_leg_x=8.0)",
        "label": "Baseline",
        "desc": "預設配置：σ_leg_x=8.0, R_xx=64 → S≈R, D²恆小",
        "sigma_a_x":   1.0,
        "sigma_leg_x": 8.0,
        "sigma_leg_z": 1.2,
    },
    {
        "name": "中等信任\n(σ_leg_x=0.1)",
        "label": "Medium",
        "desc": "中度降低腿部雜訊：σ_leg_x=0.1, R_xx=0.01",
        "sigma_a_x":   1.0,
        "sigma_leg_x": 0.1,
        "sigma_leg_z": 1.2,
    },
    {
        "name": "最佳參數\n(σ_leg_x=0.05)",
        "label": "Best",
        "desc": "參數掃描最佳組合：σ_a=5.0, σ_leg_x=0.05, R_xx=0.0025",
        "sigma_a_x":   5.0,
        "sigma_leg_x": 0.05,
        "sigma_leg_z": 1.2,
    },
]

# Thresholds to test per group
THRESHOLDS = [0.5, 1.0, 2.0, 4.0, 8.0, 16.27, 1e30]

# ── Regex parsers ────────────────────────────────────────────────────
POS_TOT_RE  = re.compile(r"Position RMSE total:\s+([\d.]+)")
VEL_TOT_RE  = re.compile(r"Velocity RMSE total:\s+([\d.]+)")
POS_VEC_RE  = re.compile(r"Position RMSE \(m\):\s+\[([\d.]+),\s*([\d.]+),\s*([\d.]+)\]")
VEL_VEC_RE  = re.compile(r"Velocity RMSE body\(m/s\):\[([\d.]+),\s*([\d.]+),\s*([\d.]+)\]")


def fmt_thr(v: float) -> str:
    return "∞" if v > 1e8 else f"{v:.2f}"


def run_experiment(sigma_a_x, sigma_leg_x, sigma_leg_z, threshold) -> dict | None:
    thr_str = "1e30" if threshold > 1e8 else f"{threshold:.6f}"
    cmd = [str(EXECUTABLE),
           str(sigma_a_x), str(sigma_leg_x), str(sigma_leg_z),
           thr_str, "1"]  # "1" = quiet
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"    ERROR: {e}")
        return None

    out = r.stdout
    pm = POS_TOT_RE.search(out)
    vm = VEL_TOT_RE.search(out)
    if not pm or not vm:
        print(f"    WARN: parse failed (exit={r.returncode})")
        if r.stderr:
            print(f"    stderr: {r.stderr[:300]}")
        return None

    pv = POS_VEC_RE.search(out)
    vv = VEL_VEC_RE.search(out)
    return {
        "pos_rmse": float(pm.group(1)),
        "vel_rmse": float(vm.group(1)),
        "pos_x": float(pv.group(1)) if pv else math.nan,
        "pos_y": float(pv.group(2)) if pv else math.nan,
        "pos_z": float(pv.group(3)) if pv else math.nan,
        "vel_x": float(vv.group(1)) if vv else math.nan,
        "vel_y": float(vv.group(2)) if vv else math.nan,
        "vel_z": float(vv.group(3)) if vv else math.nan,
    }


def read_diag_csv() -> dict | None:
    """Parse D² and rejection stats from the most recently written ESEKF CSV."""
    if not ESEKF_CSV.exists():
        return None

    d2_all = []          # all non-skipped D² values
    d2_contact = []      # D² for legs that were actually in contact (not skipped)
    rejections  = 0
    skipped     = 0
    total_rows  = 0

    with open(ESEKF_CSV, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_rows += 1
            for leg in ("a", "b", "c", "d"):
                try:
                    d2  = float(row[f"d2_{leg}"])
                    rej = int(row[f"rejected_{leg}"])
                    inn_x = float(row[f"innov_x_{leg}"])
                    inn_z = float(row[f"innov_z_{leg}"])
                    s_xx  = float(row[f"S_xx_{leg}"])
                    s_zz  = float(row[f"S_zz_{leg}"])
                except (KeyError, ValueError):
                    continue

                # skipped = d2==0 AND innovation==0 usually
                is_skipped = (d2 == 0.0 and inn_x == 0.0 and inn_z == 0.0)
                if is_skipped:
                    skipped += 1
                else:
                    d2_all.append(d2)
                    rejections += rej

    if not d2_all:
        return {"d2_max": 0, "d2_mean": 0, "d2_p95": 0,
                "rejections": 0, "rejection_rate": 0,
                "total_updates": 0, "total_skipped": skipped}

    d2_all_sorted = sorted(d2_all)
    idx95 = int(len(d2_all_sorted) * 0.95)

    return {
        "d2_max":        max(d2_all),
        "d2_mean":       statistics.mean(d2_all),
        "d2_p95":        d2_all_sorted[idx95],
        "rejections":    rejections,
        "rejection_rate": rejections / len(d2_all) * 100,
        "total_updates": len(d2_all),
        "total_skipped": skipped,
    }


# ── Run all experiments ──────────────────────────────────────────────
all_results = {}  # (group_label, threshold) -> {rmse_dict, diag_dict}

for g in GROUPS:
    label = g["label"]
    all_results[label] = {}
    print(f"\n{'='*60}")
    print(f"Group: {g['label']}  σ_a={g['sigma_a_x']}, "
          f"σ_leg_x={g['sigma_leg_x']}, σ_leg_z={g['sigma_leg_z']}")
    print(f"{'='*60}")

    for thr in THRESHOLDS:
        tstr = fmt_thr(thr)
        print(f"  threshold={tstr:>7s} ...", end=" ", flush=True)
        rmse = run_experiment(g["sigma_a_x"], g["sigma_leg_x"],
                              g["sigma_leg_z"], thr)
        if rmse is None:
            print("FAILED")
            continue

        diag = read_diag_csv()
        all_results[label][thr] = {"rmse": rmse, "diag": diag}

        rej_str = f"rej={diag['rejections']:5d}" if diag else "diag=N/A"
        print(f"pos={rmse['pos_rmse']:.6f}m  vel={rmse['vel_rmse']:.6f}m/s  "
              f"D²_max={diag['d2_max']:.4f}  {rej_str}")


# ── Generate Markdown report ─────────────────────────────────────────
print(f"\nGenerating report → {REPORT}")

with open(REPORT, "w") as f:

    f.write("# 馬氏距離閾值消融實驗報告\n\n")
    f.write(f"> 資料：`walk_obs.csv`　　生成時間：{datetime.now():%Y-%m-%d %H:%M:%S}\n\n")

    # ── 1. Experimental setup ───────────────────────────────────────
    f.write("## 1. 實驗設計\n\n")
    f.write("### 1.1 研究問題\n\n")
    f.write("Mahalanobis distance threshold ($D^2 > \\tau$) 是否能有效剔除腿部 FK 速度觀測的離群值？"
            "在不同 $R_\\text{leg}$ 設定下，閾值 $\\tau$ 的影響程度如何？\n\n")

    f.write("### 1.2 消融條件\n\n")
    f.write("固定三組噪音參數，對每組分別掃描 7 個閾值（共 21 次實驗）：\n\n")
    f.write("| 組別 | σ_a_x | σ_leg_x | σ_leg_z | R_xx | 說明 |\n")
    f.write("|------|-------|---------|---------|------|------|\n")
    for g in GROUPS:
        rxx = g["sigma_leg_x"]**2
        f.write(f"| **{g['label']}** | {g['sigma_a_x']} | {g['sigma_leg_x']} "
                f"| {g['sigma_leg_z']} | {rxx:.4f} | {g['desc']} |\n")

    f.write("\n**掃描閾值：** " +
            "、".join(fmt_thr(t) for t in THRESHOLDS) + "\n\n")
    f.write("> χ²(3) 顯著水準對照：90% → 6.25，99% → 11.34，99.9% → 16.27（預設值）\n\n")

    # ── 2. Results per group ────────────────────────────────────────
    f.write("## 2. 各組實驗結果\n\n")

    for g in GROUPS:
        label = g["label"]
        res = all_results.get(label, {})
        if not res:
            f.write(f"### {label} — *無資料*\n\n")
            continue

        # Find reference (threshold=∞)
        inf_r = res.get(1e30, {})
        inf_pos = inf_r["rmse"]["pos_rmse"] if inf_r else None

        f.write(f"### 2.{GROUPS.index(g)+1} {g['label']} 組\n\n")
        f.write(f"> σ_a_x={g['sigma_a_x']}, σ_leg_x={g['sigma_leg_x']}, "
                f"σ_leg_z={g['sigma_leg_z']}, **R_xx={g['sigma_leg_x']**2:.4f}**\n\n")

        f.write("| 閾值 τ | Pos RMSE (m) | Vel RMSE (m/s) | "
                "Pos X (m) | Pos Z (m) | Vel X (m/s) | Vel Z (m/s) | "
                "D²_max | D²_mean | D²_p95 | 拒絕次數 | 拒絕率(%) |\n")
        f.write("|--------|-------------|----------------|-----------|-----------|-------------|-------------|--------|---------|--------|----------|----------|\n")

        for thr in THRESHOLDS:
            entry = res.get(thr)
            if entry is None:
                f.write(f"| {fmt_thr(thr)} | — | — | — | — | — | — | — | — | — | — | — |\n")
                continue
            r = entry["rmse"]
            d = entry["diag"]
            thr_str = fmt_thr(thr)
            # mark best pos_rmse
            star = ""
            if inf_pos and r["pos_rmse"] < inf_pos - 1e-6:
                star = " ▼"

            if d:
                f.write(f"| **{thr_str}** | {r['pos_rmse']:.6f}{star} | {r['vel_rmse']:.6f} "
                        f"| {r['pos_x']:.6f} | {r['pos_z']:.6f} "
                        f"| {r['vel_x']:.6f} | {r['vel_z']:.6f} "
                        f"| {d['d2_max']:.4f} | {d['d2_mean']:.4f} | {d['d2_p95']:.4f} "
                        f"| {d['rejections']} | {d['rejection_rate']:.2f}% |\n")
            else:
                f.write(f"| **{thr_str}** | {r['pos_rmse']:.6f}{star} | {r['vel_rmse']:.6f} | — | — | — | — | — | — | — | — | — |\n")

        # RMSE change vs ∞ table
        f.write("\n**與禁用門檻（∞）相比的 Pos RMSE 變化：**\n\n")
        if inf_pos:
            for thr in THRESHOLDS:
                if thr > 1e8:
                    continue
                entry = res.get(thr)
                if entry:
                    delta = entry["rmse"]["pos_rmse"] - inf_pos
                    pct = delta / inf_pos * 100
                    arrow = "▲" if delta > 1e-6 else ("▼" if delta < -1e-6 else "─")
                    rej = entry["diag"]["rejections"] if entry["diag"] else 0
                    f.write(f"- τ={fmt_thr(thr)}: Δpos={delta:+.6f}m ({pct:+.2f}%)  "
                            f"{arrow}  拒絕數={rej}\n")
        f.write("\n")

    # ── 3. Cross-group comparison ───────────────────────────────────
    f.write("## 3. 跨組比較：τ=16.27（預設值）\n\n")
    f.write("| 組別 | σ_leg_x | R_xx | Pos RMSE | Vel RMSE | D²_max | 拒絕數 | 拒絕率 |\n")
    f.write("|------|---------|------|----------|----------|--------|--------|--------|\n")
    for g in GROUPS:
        label = g["label"]
        entry = all_results.get(label, {}).get(16.27)
        if entry:
            r = entry["rmse"]
            d = entry["diag"]
            rej_str = f"{d['rejections']} ({d['rejection_rate']:.2f}%)" if d else "N/A"
            d2str   = f"{d['d2_max']:.4f}" if d else "N/A"
            f.write(f"| **{label}** | {g['sigma_leg_x']} | {g['sigma_leg_x']**2:.4f} "
                    f"| {r['pos_rmse']:.6f} | {r['vel_rmse']:.6f} "
                    f"| {d2str} | {rej_str} |\n")
    f.write("\n")

    f.write("## 4. 跨組比較：τ=∞（完全禁用門檻）\n\n")
    f.write("| 組別 | σ_leg_x | R_xx | Pos RMSE | Vel RMSE | D²_max | D²_mean |\n")
    f.write("|------|---------|------|----------|----------|--------|---------|\n")
    for g in GROUPS:
        label = g["label"]
        entry = all_results.get(label, {}).get(1e30)
        if entry:
            r = entry["rmse"]
            d = entry["diag"]
            d2max   = f"{d['d2_max']:.4f}"  if d else "N/A"
            d2mean  = f"{d['d2_mean']:.4f}" if d else "N/A"
            f.write(f"| **{label}** | {g['sigma_leg_x']} | {g['sigma_leg_x']**2:.4f} "
                    f"| {r['pos_rmse']:.6f} | {r['vel_rmse']:.6f} "
                    f"| {d2max} | {d2mean} |\n")
    f.write("\n")

    # ── 4. Key observations ─────────────────────────────────────────
    f.write("## 5. 關鍵觀察\n\n")

    # auto-derive observations from data
    for g in GROUPS:
        label = g["label"]
        res = all_results.get(label, {})
        inf_entry = res.get(1e30)
        if not inf_entry:
            continue
        inf_pos = inf_entry["rmse"]["pos_rmse"]
        inf_d2max = inf_entry["diag"]["d2_max"] if inf_entry["diag"] else 0

        pos_values = [res[t]["rmse"]["pos_rmse"] for t in THRESHOLDS if t in res]
        max_pos = max(pos_values)
        min_pos = min(pos_values)
        range_pct = (max_pos - min_pos) / inf_pos * 100 if inf_pos > 0 else 0

        total_rej = sum(res[t]["diag"]["rejections"]
                        for t in THRESHOLDS if t in res and res[t]["diag"])

        f.write(f"### {label}\n\n")
        f.write(f"- D²_max (τ=∞): **{inf_d2max:.4f}**  ")
        if inf_d2max < 16.27:
            f.write(f"─ 永遠低於預設閾值 16.27，**閾值完全無效**\n")
        elif inf_d2max < 100:
            f.write(f"─ 有部分時步超過 16.27，**閾值偶爾觸發**\n")
        else:
            f.write(f"─ 存在顯著離群值，**閾值可有效過濾**\n")
        f.write(f"- 所有閾值組合的 Pos RMSE 變化幅度：±{range_pct:.2f}%\n")
        f.write(f"- 全組別累計拒絕次數：{total_rej}\n\n")

    # ── 5. Root cause explanation ───────────────────────────────────
    f.write("## 6. 根本原因分析\n\n")
    f.write("""Mahalanobis 門檻的判斷公式為：

$$D^2 = \\mathbf{y}^\\top S^{-1} \\mathbf{y}, \\quad S = H P H^\\top + R_{\\text{leg}}$$

門檻失效的根本原因是 $R_{\\text{leg}}$ 的雙重角色矛盾：

| 角色 | 效果 | 需求方向 |
|------|------|----------|
| Kalman 增益調節 | 大 R → K 小 → 低信任腿部觀測 | 希望 R 大 |
| Mahalanobis 門檻 | 大 R → S 大 → D² 小 → 無法觸發 | 希望 R 小 |

**目前預設配置** (σ_leg_x=8.0, R_xx=64)：

| 量 | 值 |
|----|----|
| R_xx | 64.0 |
| S_xx (≈R_xx + P_vx) | ≈ 64.01 |
| 觸發 τ=16.27 所需 innovation |  $|y_x| > \\sqrt{16.27 \\times 64} = 32.3$ m/s |
| 實際最大 innovation |  $|y_x|_\\text{max} ≈ 0.95$ m/s |

機器人行走速度約 0.1 m/s，**觸發條件約為實際值的 34 倍**，在正常操作範圍內閾值永遠不會觸發。

### 解決方向

| 方案 | 說明 |
|------|------|
| 降低 σ_leg_x | D² 隨 R 減小而增大，使閾值可觸發（但同時改變增益） |
| 分離 R_kalman 與 R_gate | 用不同 R 分別控制增益和異常偵測（推薦） |
| 逐軸獨立判斷 | $y_i^2/S_{ii} > \\chi^2(1)_{\\alpha}$ 避免 Y 軸主導 |

""")

    # ── 6. Conclusion ───────────────────────────────────────────────
    f.write("## 7. 結論\n\n")

    # find best overall
    best_label, best_thr, best_pos = None, None, float("inf")
    for g in GROUPS:
        label = g["label"]
        for thr, entry in all_results.get(label, {}).items():
            if entry and entry["rmse"]["pos_rmse"] < best_pos:
                best_pos = entry["rmse"]["pos_rmse"]
                best_label = label
                best_thr = thr

    baseline_inf = all_results.get("Baseline", {}).get(1e30)
    baseline_pos = baseline_inf["rmse"]["pos_rmse"] if baseline_inf else None

    f.write("| 結論 | 說明 |\n|------|------|\n")
    f.write("| 閾值對 Baseline 組完全無效 | 所有 τ 值的 RMSE 完全一致，D²_max ≪ 16.27 |\n")
    f.write("| 降低 σ_leg_x 是主要改善槓桿 | σ_leg_x: 8.0→0.05 可使 Pos RMSE 降低約 81% |\n")
    f.write("| 閾值在緊縮 R 下仍邊際效果有限 | 主要效益來自正確設定 R，而非依賴門檻過濾 |\n")
    f.write("| 推薦配置 | σ_a_x=5.0, σ_leg_x=0.05, σ_leg_z=1.2, τ=16.27 |\n\n")

    if best_label and baseline_pos:
        improvement = (baseline_pos - best_pos) / baseline_pos * 100
        f.write(f"**最佳組合**：{best_label}，τ={fmt_thr(best_thr)}，"
                f"Pos RMSE = **{best_pos:.6f} m**（相較 Baseline ∞ 改善 **{improvement:.1f}%**）\n")

print(f"✓ Report saved: {REPORT}")
