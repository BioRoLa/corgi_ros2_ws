"""Scratch probe: is the 'lands later so less sweep remains' story right?

Read-only. Not a registered analyser. Written 2026-08-22 for the clock_ff
adversarial review.
"""
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE  # noqa: E402

LEGS = "ABCD"
DIR_BETA = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}
TAIL_S = 20.0
TPL_TD = -0.1614
TPL_LO = +0.1609
TPL_PERIOD = 0.2662
TPL_ROWS = 265
TPL_STANCE_ROWS = 108


def load(path):
    """-> {leg: dict of arrays}, common-mode (beta-axis) signals."""
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            m = r["motor"]
            if m not in ("L_Motor", "R_Motor"):
                continue
            try:
                key = (r["leg"], float(r["t"]))
                d = per.setdefault(key, {})
                d["c"] = int(r["in_contact"])
                d["b"] = float(r["beta"])
                d["th"] = float(r["theta"])
                s = m[0]
                d["e" + s] = float(r["pos_error"])
                d["ff" + s] = float(r["t_ff"])
                d["st" + s] = float(r["t_stiff"])
                d["dp" + s] = float(r["t_damp"])
                d["td" + s] = float(r["tau_demand"])
                d["kp" + s] = float(r["kp"])
                d["kd" + s] = float(r["kd"])
            except (ValueError, KeyError):
                continue
    byleg = {}
    for (leg, t), d in per.items():
        if "eL" not in d or "eR" not in d:
            continue
        sgn = DIR_BETA[leg]
        byleg.setdefault(leg, []).append((
            t, d["c"], d["b"], d["th"],
            sgn * 0.5 * (d["eL"] + d["eR"]),      # beta-axis pos error
            sgn * 0.5 * (d["ffL"] + d["ffR"]),    # beta-axis ff (per motor)
            sgn * 0.5 * (d["stL"] + d["stR"]),    # beta-axis stiffness torque
            sgn * 0.5 * (d["dpL"] + d["dpR"]),    # beta-axis damping torque
            sgn * 0.5 * (d["tdL"] + d["tdR"]),    # beta-axis demand
            0.5 * (d["kpL"] + d["kpR"]),
            0.5 * (d["kdL"] + d["kdR"]),
        ))
    out = {}
    names = ["t", "c", "b", "th", "e", "ff", "st", "dp", "td", "kp", "kd"]
    for leg, rows in byleg.items():
        rows.sort()
        a = np.array(rows)
        if len(a) < 500:
            continue
        out[leg] = {n: a[:, i] for i, n in enumerate(names)}
        out[leg]["c"] = a[:, 1].astype(bool)
    return out


def episodes(t, c):
    d = debounce(c, DEBOUNCE)
    chg = np.diff(d.astype(int))
    rise = np.flatnonzero(chg > 0) + 1
    fall = np.flatnonzero(chg < 0) + 1
    eps = []
    for r0 in rise:
        nxt = fall[fall > r0]
        if not len(nxt):
            continue
        f0 = nxt[0]
        if f0 - r0 < 3:
            continue
        eps.append((r0, f0))
    return eps, d


def analyse(path):
    legs = load(path)
    out = {k: [] for k in (
        "stance_s", "flight_s", "stride_s", "duty", "beta_TD", "beta_LO",
        "sweep", "dbeta_stance", "trk_TD", "trk_pre20", "trk_mid",
        "ff_TD", "ff_pre20", "ff_mid", "st_TD", "st_mid", "td_TD", "td_mid",
        "kp_TD", "kp_mid", "kd_TD", "cmd_TD", "beta_amp", "beta_min",
        "beta_max", "clockfrac_TD", "airborne_clock_stance")}
    dt = None
    for leg, a in legs.items():
        t, c = a["t"], a["c"]
        m = t >= (t.max() - TAIL_S)
        a = {k: v[m] for k, v in a.items()}
        t, c, b = a["t"], a["c"], a["b"]
        if len(t) < 200:
            continue
        dt = float(np.median(np.diff(t)))
        pre20 = max(1, int(round(0.020 / dt)))
        eps, d = episodes(t, c)
        if len(eps) < 5:
            continue
        out["beta_amp"].append(float(np.percentile(b, 95) - np.percentile(b, 5)))
        out["beta_min"].append(float(np.percentile(b, 2)))
        out["beta_max"].append(float(np.percentile(b, 98)))
        prev_r0 = None
        for r0, f0 in eps:
            st = (f0 - r0) * dt
            out["stance_s"].append(st)
            out["beta_TD"].append(b[r0])
            out["beta_LO"].append(b[f0])
            out["sweep"].append(b[f0] - b[r0])
            out["dbeta_stance"].append((b[f0] - b[r0]) / st if st > 0 else np.nan)
            mid = (r0 + f0) // 2
            out["trk_TD"].append(a["e"][r0])
            out["trk_mid"].append(a["e"][mid])
            out["ff_TD"].append(a["ff"][r0])
            out["ff_mid"].append(a["ff"][mid])
            out["st_TD"].append(a["st"][r0])
            out["st_mid"].append(a["st"][mid])
            out["td_TD"].append(a["td"][r0])
            out["td_mid"].append(a["td"][mid])
            out["kp_TD"].append(a["kp"][r0])
            out["kp_mid"].append(a["kp"][mid])
            out["kd_TD"].append(a["kd"][r0])
            cmd = b[r0] + a["e"][r0]
            out["cmd_TD"].append(cmd)
            out["clockfrac_TD"].append((cmd - TPL_TD) / (TPL_LO - TPL_TD))
            if r0 - pre20 >= 0:
                out["trk_pre20"].append(a["e"][r0 - pre20])
                out["ff_pre20"].append(a["ff"][r0 - pre20])
            if prev_r0 is not None:
                stride = (r0 - prev_r0) * dt
                if 0.15 < stride < 0.60:
                    out["stride_s"].append(stride)
                    out["duty"].append(st / stride)
                    out["flight_s"].append(stride - st)
            prev_r0 = r0
    res = {k: (float(np.median(v)) if len(v) else float("nan"))
           for k, v in out.items()}
    res["n_ep"] = len(out["stance_s"])
    res["dt"] = dt
    # per-stride track_err (median of differences) vs difference of medians
    res["trk_TD_mean"] = float(np.mean(out["trk_TD"])) if out["trk_TD"] else np.nan
    return res


def ff_step_phase(path):
    """When does the CLOCK think stance starts, relative to physical contact?

    In the on/both cells the clock term is a step in t_ff at the clock's
    stance boundary. Report, per leg, the beta-axis ff level while the foot
    is physically airborne vs in contact.
    """
    legs = load(path)
    rows = []
    for leg, a in legs.items():
        t = a["t"]
        m = t >= (t.max() - TAIL_S)
        a = {k: v[m] for k, v in a.items()}
        _, d = episodes(a["t"], a["c"])
        d = d[:len(a["ff"])]
        rows.append((leg,
                     float(np.median(a["ff"][d])),
                     float(np.median(a["ff"][~d])),
                     float(np.percentile(a["ff"][~d], 90)),
                     float(np.percentile(a["ff"][~d], 10))))
    return rows


def main():
    base = os.path.expanduser("~/corgi_runs/clock_ff")
    cells = ["off", "on", "both"]
    per_cell = {}
    for cell in cells:
        runs = []
        for p in sorted(glob.glob(os.path.join(base, cell, "run[0-9].csv"))):
            runs.append(analyse(p))
        per_cell[cell] = runs

    def med(cell, k):
        v = [r[k] for r in per_cell[cell] if not np.isnan(r[k])]
        return float(np.median(v)) if v else float("nan")

    print("dt = %.4f s\n" % per_cell["off"][0]["dt"])
    print("=== A. CONTACT TIMING (per-stride medians, pooled 3 runs) ===")
    print("%-6s %9s %9s %9s %8s %7s" % (
        "cell", "stance_s", "flight_s", "stride_s", "duty", "n_ep"))
    for c in cells:
        print("%-6s %9.4f %9.4f %9.4f %8.3f %7d" % (
            c, med(c, "stance_s"), med(c, "flight_s"), med(c, "stride_s"),
            med(c, "duty"), sum(r["n_ep"] for r in per_cell[c])))

    print("\n=== B. BETA KINEMATICS ===")
    print("%-6s %9s %9s %9s %11s %9s %9s %9s" % (
        "cell", "beta_TD", "beta_LO", "sweep", "dbeta/dt", "amp(p5-95)",
        "beta_min", "beta_max"))
    for c in cells:
        print("%-6s %+9.4f %+9.4f %+9.4f %+11.4f %9.4f %+9.4f %+9.4f" % (
            c, med(c, "beta_TD"), med(c, "beta_LO"), med(c, "sweep"),
            med(c, "dbeta_stance"), med(c, "beta_amp"),
            med(c, "beta_min"), med(c, "beta_max")))

    print("\n=== C. CLOCK ALIGNMENT AT PHYSICAL TOUCHDOWN ===")
    print("template stance window: beta_cmd %+.4f -> %+.4f over %d/%d rows"
          " = %.4f s" % (TPL_TD, TPL_LO, TPL_STANCE_ROWS, TPL_ROWS,
                         TPL_PERIOD * TPL_STANCE_ROWS / TPL_ROWS))
    tstance = TPL_PERIOD * TPL_STANCE_ROWS / TPL_ROWS
    print("%-6s %9s %11s %14s" % (
        "cell", "cmd_TD", "clockfrac", "airborne_ms"))
    for c in cells:
        fr = med(c, "clockfrac_TD")
        print("%-6s %+9.4f %11.3f %14.1f" % (
            c, med(c, "cmd_TD"), fr, 1000.0 * fr * tstance))

    print("\n=== D. TRACK ERROR AND TORQUE AT TOUCHDOWN (beta-axis, per motor) ===")
    print("%-6s %9s %9s %9s %9s %9s %9s %8s" % (
        "cell", "trk_pre20", "trk_TD", "trk_mid", "ff_pre20", "ff_TD",
        "ff_mid", "kp_TD"))
    for c in cells:
        print("%-6s %+9.4f %+9.4f %+9.4f %+9.3f %+9.3f %+9.3f %8.1f" % (
            c, med(c, "trk_pre20"), med(c, "trk_TD"), med(c, "trk_mid"),
            med(c, "ff_pre20"), med(c, "ff_TD"), med(c, "ff_mid"),
            med(c, "kp_TD")))
    print("%-6s %9s %9s %9s %9s" % ("cell", "st_TD", "st_mid", "td_TD", "td_mid"))
    for c in cells:
        print("%-6s %+9.3f %+9.3f %+9.3f %+9.3f" % (
            c, med(c, "st_TD"), med(c, "st_mid"), med(c, "td_TD"),
            med(c, "td_mid")))

    print("\n=== E. DEFINITIONAL TEST: is d(pos_err)*kp explained by d(ff)? ===")
    for c in ("on", "both"):
        de = med(c, "trk_TD") - med("off", "trk_TD")
        kp = med(c, "kp_TD")
        dff = med(c, "ff_TD") - med("off", "ff_TD")
        dst = med(c, "st_TD") - med("off", "st_TD")
        dtd = med(c, "td_TD") - med("off", "td_TD")
        print("  %-5s d(trk)=%+.4f rad  kp=%.1f  kp*d(trk)=%+.2f N.m"
              "   d(t_ff)=%+.2f   d(t_stiff)=%+.2f   d(tau_dem)=%+.2f"
              % (c, de, kp, kp * de, dff, dst, dtd))

    print("\n=== F. WHERE IS THE FF APPLIED (beta-axis per-motor median) ===")
    print("%-6s %-4s %10s %10s %10s %10s" % (
        "cell", "leg", "ff_contact", "ff_air", "ff_air_p90", "ff_air_p10"))
    for c in cells:
        p = os.path.join(base, c, "run1.csv")
        for leg, fc, fa, f90, f10 in sorted(ff_step_phase(p)):
            print("%-6s %-4s %+10.3f %+10.3f %+10.3f %+10.3f" % (
                c, leg, fc, fa, f90, f10))

    print("\n=== G. PER-RUN SPREAD (sanity) ===")
    for c in cells:
        print("  %-5s stance_s %s  sweep %s  trk_TD %s" % (
            c,
            ["%.4f" % r["stance_s"] for r in per_cell[c]],
            ["%+.4f" % r["sweep"] for r in per_cell[c]],
            ["%+.4f" % r["trk_TD"] for r in per_cell[c]]))


if __name__ == "__main__":
    main()
