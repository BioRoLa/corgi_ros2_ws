#!/usr/bin/env python3
"""Three independent radius estimates per run, one table per cell. Log S228
showed the map's 10-deg point was ESTIMATOR-LIMITED: on arcs under 90 deg the
path estimators (Menger, Kasa) bias toward straight while the kinematic one
does not. S229 registers the decisive run (10 deg, GAIT_SIM 50) and scores it
with this; S230's turn_rate map uses the same three numbers.

  R_menger = 1 / median per-triple Menger kappa (1 m chords)   -- the map's
  R_fit    = Kasa circle fit over the band (cross_track.fit_circle)
  R_yaw    = v_fwd / |yaw rate|                                -- kinematic

Band is [t0+START, t0+END] of the odom file; END defaults to 30 (the banked
convention) and MUST be raised for long windows (--end 48 for GAIT_SIM 50).
A run is VALID if band v_fwd >= 0.10 m/s (S191) and |kappa| <= 1; pirouettes
are listed, not scored. Every row prints its fit arc: under 90 deg the two path
radii are flagged 'short' and should not be quoted alone.

S229 bars (10 deg, +1, n=3):
  P-L10-1  fit arc >= 90 deg on every valid run
  P-L10-2  cell-median R_fit and R_menger each within +-25% of R_yaw
  P-L10-3  cell-median R_yaw in [2.4, 3.1] m
"""
import argparse, glob, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cross_track as ct                                      # noqa: E402
import aggregate_menger as am                                 # noqa: E402
import speed_from_odom as sfo                                 # noqa: E402


def rows(d, end):
    """One row per run. The band [t0+START, t0+end] is applied to ALL THREE
    estimators explicitly -- cross_track.run_row leaves aggregate_menger's
    end=30 default in place, which would have silently kept the Menger band
    at 12-30 s on a 50 s window (the exact trap S228 is about)."""
    out = []
    for tq in sorted(glob.glob(os.path.join(d, "run[0-9]*.csv"))):
        b = os.path.basename(tq)[3:-4]
        if not b.isdigit():
            continue
        n = int(b)
        od = os.path.join(d, "odom_run%d.csv" % n)
        if not os.path.exists(od):
            continue
        ct.END = float(end)
        xy = ct.band_xy(od)
        if xy is None:
            continue
        cx, cy, R, res = ct.fit_circle(xy[:, 0], xy[:, 1])
        arc = ct.arc_deg(xy[:, 0], xy[:, 1], cx, cy)
        try:
            kap, _ = am.run_kappa(od, tq, ct.START, 1.0, end=float(end))
        except SystemExit:
            kap = float("nan")
        try:
            st = sfo.stats(od)
            v = float(st["v_fwd"])
            yr = abs(math.radians(st["yaw_fit_deg_s"]))
            R_yaw = v / yr if yr > 1e-3 else float("nan")
        except sfo.Unfit:
            v, R_yaw = float("nan"), float("nan")
        r = {"run": n, "R_fit": R, "rms_mm": 1000.0 * float(np.sqrt(np.mean(res * res))),
             "arc": arc, "kappa": kap,
             "R_menger": 1.0 / abs(kap) if not math.isnan(kap) and abs(kap) > 1e-6 else float("nan"),
             "v_fwd": v, "R_yaw": R_yaw}
        r["valid"] = bool(not math.isnan(kap) and abs(kap) <= 1 and not math.isnan(v) and v >= 0.10)
        out.append(r)
    return out


def report(d, end, bars=False):
    rs = rows(d, end)
    print("cell %s  band [%g, %g] s  n_files=%d" % (d, ct.START, end, len(rs)))
    print("%4s %8s %8s %8s %7s %6s %7s %s" % ("run", "R_menger", "R_fit", "R_yaw", "arc", "v_fwd", "kappa", ""))
    for r in rs:
        flag = "" if r["valid"] else "INVALID"
        if r["valid"] and r["arc"] < 90:
            flag = "short-arc"
        print("%4d %8.2f %8.2f %8.2f %7.0f %6.2f %7.3f %s" % (
            r["run"], r["R_menger"], r["R_fit"], r["R_yaw"], r["arc"], r["v_fwd"], r["kappa"], flag))
    v = [r for r in rs if r["valid"]]
    if not v:
        print("no valid runs"); return None
    med = {k: float(np.nanmedian([r[k] for r in v])) for k in ("R_menger", "R_fit", "R_yaw", "arc")}
    print("median over %d valid: R_menger %.2f  R_fit %.2f  R_yaw %.2f  arc %.0f" % (
        len(v), med["R_menger"], med["R_fit"], med["R_yaw"], med["arc"]))
    if bars:
        p1 = all(r["arc"] >= 90 for r in v)
        p2 = (abs(med["R_fit"] / med["R_yaw"] - 1) <= 0.25 and
              abs(med["R_menger"] / med["R_yaw"] - 1) <= 0.25)
        p3 = 2.4 <= med["R_yaw"] <= 3.1
        for nm, ok, txt in (("P-L10-1", p1, "arc >= 90 on every valid run"),
                            ("P-L10-2", p2, "R_fit, R_menger within +-25%% of R_yaw (%.2f, %.2f vs %.2f)" % (med["R_fit"], med["R_menger"], med["R_yaw"])),
                            ("P-L10-3", p3, "median R_yaw in [2.4, 3.1] (%.2f)" % med["R_yaw"])):
            print("  %s  %s  %s" % (nm, "PASS" if ok else "FAIL", txt))
    return med


def selftest():
    # a synthetic circle, R = 2.5 m, v = 0.3 m/s: all three must agree to 5%
    import tempfile, csv
    R, v, dt = 2.5, 0.3, 0.01
    t = np.arange(0, 50, dt); th = v / R * t
    x, y = R * np.sin(th), R * (1 - np.cos(th))
    tmp = tempfile.mkdtemp()
    with open(os.path.join(tmp, "odom_run1.csv"), "w", newline="") as f:
        w = csv.writer(f)   # base_odom --csv layout: sec,nsec,_,_,x,y,z,qx,qy,qz,qw
        for ti, xi, yi, h in zip(t, x, y, th):
            w.writerow([int(ti), int(round((ti - int(ti)) * 1e9)), 0, 0, xi, yi, 0.0,
                        0.0, 0.0, math.sin(h / 2), math.cos(h / 2)])
    # long-format torque capture, one row per (t, leg); leg A in contact for
    # 0.13 s of every 0.265 s stride -> Menger samples at its touchdowns
    with open(os.path.join(tmp, "run1.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "leg", "in_contact", "theta", "gamma"])
        for ti in t:
            c = 1 if (ti % 0.265) < 0.13 else 0
            for leg in "ABCD":
                w.writerow([round(ti, 3), leg, c, 0.0, 0.0])
    rs = rows(tmp, 48)
    assert rs and rs[0]["valid"], rs
    r = rs[0]
    for k in ("R_menger", "R_fit", "R_yaw"):
        assert abs(r[k] / R - 1) < 0.05, (k, r[k])
    print("selftest OK: synthetic R 2.5 -> menger %.2f fit %.2f yaw %.2f arc %.0f" % (
        r["R_menger"], r["R_fit"], r["R_yaw"], r["arc"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", action="append", default=[])
    ap.add_argument("--end", type=float, default=30.0)
    ap.add_argument("--bars", action="store_true", help="score S229's P-L10 bars on the (single) cell")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest(); return
    for c in a.cell:
        report(c, a.end, a.bars); print()


if __name__ == "__main__":
    main()
