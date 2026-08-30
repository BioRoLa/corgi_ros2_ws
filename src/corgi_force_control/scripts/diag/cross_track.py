#!/usr/bin/env python3
"""The FIRST cross-track measurement on the camber arm -- from banked runs.
Log S198.

WHY. Open Issue #25: the thesis's headline comparison names three axes
(cross-track error, energetic cost, trajectory repeatability) and cross-track
has NEVER been measured under camber -- all seven mentions in the log (25 mm
RMS) sit in SS19-20, the uncambered differential-steering thread. It is the
one axis that is UNKNOWN rather than doubted. This closes that gap offline,
from runs already on disk. Zero simulator time.

DEFINITIONAL CAVEAT, stated up front. The camber cells are OPEN-LOOP
(turn_rate = 0, heading hold off): there is no commanded circle, so the RMS
radial residual of the best-fit circle measures PATH REGULARITY -- how well the
robot holds its OWN circle -- not tracking error against a command. The
differential cells (turn_rep) do carry a commanded rate, so their RMS is closer
to true cross-track. Comparing regularity across arms is the honest first cut;
a tracking comparison needs a camber arm with a commanded turn, which no banked
tree has.

The uncambered, unturned lam0 cell is the noise floor: this plant yaws
-0.5..-2 deg/s on its own (S195), so even "straight" runs curl and a best-fit
circle finds SOMETHING. A camber cell is only showing real structure where it
beats that floor.

Everything here reuses shipped implementations -- one implementation per
quantity, the aggregate_menger discipline:
    fit_circle        check_turn.py (Kasa; its short-arc caution applies and
                      the covered arc is printed per run, flagged under 90 deg)
    kappa             aggregate_menger.run_kappa (Menger, S88 band)
    v_fwd             speed_from_odom.stats (heading-integrated)
    tau p99.5         tau_demand_window.stats (pre-clamp |tau_demand|)

R_fit vs 1/|kappa| is the built-in cross-check: two independent estimates of
the same radius from the same path (circle fit vs local curvature). Where they
disagree, believe neither without looking.

Usage:
    cross_track.py --selftest
    cross_track.py --dir <cell> --label <name> [--dir ...]
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import load_odom_csv                         # noqa: E402
from check_turn import fit_circle                              # noqa: E402
from aggregate_menger import run_kappa                         # noqa: E402
import speed_from_odom as sfo                                  # noqa: E402
import tau_demand_window as tdw                                # noqa: E402

START, END = 12.0, 30.0


def band_xy(od):
    t, xy, _q = load_odom_csv(od)
    a0, a1 = t[0] + START, min(t[-1], t[0] + END)
    m = (t >= a0) & (t <= a1)
    if int(m.sum()) < 100:
        return None
    return xy[m]


def arc_deg(x, y, cx, cy):
    """Angular span actually covered on the fitted circle, degrees."""
    th = np.unwrap(np.arctan2(y - cy, x - cx))
    return float(np.degrees(abs(th[-1] - th[0])))


def run_row(d, n):
    od = os.path.join(d, "odom_run%d.csv" % n)
    tq = os.path.join(d, "run%d.csv" % n)
    if not (os.path.exists(od) and os.path.exists(tq)):
        return None
    xy = band_xy(od)
    if xy is None:
        return None
    cx, cy, R, res = fit_circle(xy[:, 0], xy[:, 1])
    rms = float(np.sqrt(np.mean(res * res)))
    arc = arc_deg(xy[:, 0], xy[:, 1], cx, cy)
    try:
        kap, _ntr = run_kappa(od, tq, START, 1.0)
    except SystemExit:
        kap = float("nan")
    try:
        v = float(sfo.stats(od)["v_fwd"])
    except sfo.Unfit:
        v = float("nan")
    try:
        tau = float(tdw.stats(tq)["pooled_p995"])
    except Exception:                                          # noqa: BLE001
        tau = float("nan")
    return {"run": n, "R_fit": R, "rms_mm": 1000.0 * rms, "arc": arc,
            "kappa": kap, "R_menger": 1.0 / abs(kap) if kap and not
            math.isnan(kap) and abs(kap) > 1e-6 else float("nan"),
            "v_fwd": v, "tau": tau}


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    rng = np.random.RandomState(0)
    # 1. a noisy circle: recover R and the injected noise as RMS
    R0, sig = 2.0, 0.010
    th = np.linspace(0.0, 2.0 * np.pi, 2000)
    r = R0 + rng.normal(0.0, sig, th.size)
    x, y = 3.0 + r * np.cos(th), -1.0 + r * np.sin(th)
    cx, cy, R, res = fit_circle(x, y)
    rms = float(np.sqrt(np.mean(res * res)))
    chk("full circle R=2.000 recovered (got %.4f)" % R, abs(R - R0) < 2e-3)
    chk("injected 10.0 mm noise read back as RMS (got %.1f mm)" % (1e3 * rms),
        abs(1e3 * rms - 10.0) < 0.5)
    chk("centre recovered (%.3f, %.3f)" % (cx, cy),
        abs(cx - 3.0) < 1e-2 and abs(cy + 1.0) < 1e-2)
    # 2. arc coverage on a known half circle
    th2 = np.linspace(0.0, np.pi, 500)
    a = arc_deg(R0 * np.cos(th2), R0 * np.sin(th2), 0.0, 0.0)
    chk("half circle covers 180 deg (got %.1f)" % a, abs(a - 180.0) < 1.0)
    # 3. the short-arc bias check_turn warns about, demonstrated: a 45 deg arc
    #    with the same noise must fit WORSE on R than the full circle did
    th3 = np.linspace(0.0, np.pi / 4.0, 500)
    r3 = R0 + rng.normal(0.0, sig, th3.size)
    _cx, _cy, R3, _res = fit_circle(r3 * np.cos(th3), r3 * np.sin(th3))
    chk("45 deg arc misfits R by more than the full circle "
        "(|err| %.3f vs %.4f)" % (abs(R3 - R0), abs(R - R0)),
        abs(R3 - R0) > abs(R - R0))
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("cross_track.py selftest\n")
        good = selftest()
        print("\n  SELFTEST %s" % ("PASS" if good else "FAIL"))
        return 0 if good else 1
    if not a.dir:
        ap.error("need --dir, or --selftest")

    print("best-fit-circle residuals on banked runs, band [t0+%.0f, t0+%.0f]s"
          % (START, END))
    print("open-loop camber cells measure PATH REGULARITY, not tracking error;")
    print("arcs under 90 deg are flagged per check_turn's Kasa caution.\n")
    print("  %-18s %4s %8s %9s %7s %9s %8s %7s %8s" %
          ("cell", "run", "R_fit", "R_menger", "arc", "rms mm", "rms/R %",
           "v_fwd", "tau p995"))
    cells = {}
    for i, d in enumerate(a.dir):
        d = os.path.expanduser(d)
        name = a.label[i] if i < len(a.label) else os.path.basename(d.rstrip("/"))
        rows = []
        for n in range(1, 10):
            r = run_row(d, n)
            if r is None:
                continue
            rows.append(r)
            flag = " <90deg" if r["arc"] < 90.0 else ""
            print("  %-18s %4d %8.2f %9.2f %6.0f%s %9.1f %8.2f %7.3f %8.2f" %
                  (name, r["run"], r["R_fit"], r["R_menger"], r["arc"],
                   flag or " ", r["rms_mm"],
                   100.0 * r["rms_mm"] / 1000.0 / r["R_fit"],
                   r["v_fwd"], r["tau"]))
        if rows:
            cells[name] = rows

    print("\n  %-18s %3s %10s %12s %10s %10s" %
          ("cell", "n", "med rms mm", "med rms/R %", "med v_fwd", "med tau"))
    for name, rows in cells.items():
        rr = [r["rms_mm"] for r in rows]
        rel = [100.0 * r["rms_mm"] / 1000.0 / r["R_fit"] for r in rows]
        print("  %-18s %3d %10.1f %12.2f %10.3f %10.2f" %
              (name, len(rows), np.median(rr), np.median(rel),
               np.median([r["v_fwd"] for r in rows]),
               np.median([r["tau"] for r in rows])))
    print("\nNOT a scored verdict: unregistered, cross-campaign, and the two")
    print("arms differ in what their RMS means (see the docstring caveat).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
