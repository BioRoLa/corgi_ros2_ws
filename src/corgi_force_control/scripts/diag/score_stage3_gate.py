#!/usr/bin/env python3
"""Score the Stage 3 cambered gate -- sustained arcs matching the template's
sagittal state. Gate text: Thesis Timeline, Stage 3, "The gate, defined"
(rewritten 2026-08-23). DRAFT until the campaign that attempts it registers
its bars in the log.

THE GATE. >= 5 VALID arcs out of <= 8 attempts at one cell. A run is a valid
arc when EVERY clause holds (conjunctive, one consequence -- S172's lesson):

    arc        >= 180 deg of heading change over the band [t0+12, end], AND
               the Kasa fit covers >= 90 deg (check_turn's caution)
    collapse   band v_fwd >= 0.10 m/s in EVERY 4 s window (S191 threshold;
               the failure is absorbing and whole-band means hide it, S195)
    speed      band v_fwd >= 0.235 m/s (0.85 x the lambda=0 baseline)
    screen     beta_TD within -0.084 +- 0.006 rad; sweep_frac_fwd in
               [0.70, 0.80]; >= 8 touchdowns on every leg  (the S152 screen)
    stride     dominant theta period within +-5% of the template's 0.2642 s
               (this is ALSO the playback check, playback_ratio's own method)

Every quantity comes from an implementation that already has a track record;
this file combines, it does not compute:
    cross_track.run_row         R_fit, fit arc, kappa, v_fwd, tau, rms
    yaw_excursion.windows       4 s windows of v_fwd, from the start of odom
    touchdown_phase.stats       beta_TD, sweep_frac_fwd, n_TD_by_leg
    playback_ratio              _series + dominant_period on theta
    speed_from_odom.yaw_from_quat, check_menger.load_odom_csv  (heading)

REPORTED, NOT GATED (the Timeline says why for each): flight fraction (#22),
body roll (S186), kappa vs the Stage 2a model (S127), yaw-drift contamination.

Usage:
    score_stage3_gate.py --selftest
    score_stage3_gate.py --base ~/corgi_runs/stage3_gate
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import load_odom_csv                       # noqa: E402
from speed_from_odom import yaw_from_quat                    # noqa: E402
from cross_track import run_row                              # noqa: E402
from yaw_excursion import windows                            # noqa: E402
import touchdown_phase as tdp                                # noqa: E402
import playback_ratio as pbr                                 # noqa: E402

START = 12.0
TEMPLATE_PERIOD = 0.2642
B = {  # the bands, verbatim from the Timeline
    "arc_deg": 180.0, "fit_arc_deg": 90.0,
    "collapse_vfwd": 0.10, "speed_vfwd": 0.235,
    "beta_td": -0.084, "beta_td_tol": 0.006,
    "fwd_lo": 0.70, "fwd_hi": 0.80, "min_td": 8,
    "stride_tol": 0.05,
    "r_lo": None, "r_hi": None,   # optional RADIUS clause (S217): median R_fit window, m
}
NEED = 5
MAX_ATTEMPTS = 8


def heading_change_deg(od):
    """|delta yaw| over the band, from the odom quaternions."""
    t, _xy, q = load_odom_csv(od)
    m = t >= t[0] + START
    if int(m.sum()) < 50:
        return float("nan")
    yaw = np.unwrap(yaw_from_quat(q[m]))
    return float(abs(math.degrees(yaw[-1] - yaw[0])))


def run_clauses(d, n):
    """-> dict of measured values + per-clause booleans, or None if no run."""
    tq, od = os.path.join(d, "run%d.csv" % n), os.path.join(d, "odom_run%d.csv" % n)
    if not (os.path.exists(tq) and os.path.exists(od)):
        return None
    r = run_row(d, n)
    if r is None:
        return None
    v = {"run": n, "kappa": r["kappa"], "R_fit": r["R_fit"], "fit_arc": r["arc"],
         "v_fwd": r["v_fwd"], "rms_mm": r["rms_mm"], "tau": r["tau"]}
    v["heading_deg"] = heading_change_deg(od)
    w = [x for x in windows(tq, od) if x[0] >= START]
    v["min_window_vfwd"] = float(min(x[1] for x in w)) if w else float("nan")
    v["n_windows"] = len(w)
    try:
        s = tdp.stats(tq)
        v["beta_TD"], v["fwd_frac"] = float(s["beta_TD"]), float(s["sweep_frac_fwd"])
        v["min_td"] = int(min(s["n_TD_by_leg"].values()))
    except Exception as e:                                   # noqa: BLE001
        v["beta_TD"] = v["fwd_frac"] = float("nan"); v["min_td"] = 0
        v["screen_err"] = str(e)
    try:
        _leg, dt, ch = pbr._series(tq)
        p, _peak = pbr.dominant_period(ch["theta"], dt)
        v["stride_s"] = float(p)
    except Exception as e:                                   # noqa: BLE001
        v["stride_s"] = float("nan"); v["stride_err"] = str(e)
    return evaluate(v)


def evaluate(v):
    """Attach the clause booleans to a measured-values dict. Pure."""
    c = {}
    c["arc"] = (v["heading_deg"] >= B["arc_deg"]) and (v["fit_arc"] >= B["fit_arc_deg"])
    c["collapse"] = v["n_windows"] > 0 and v["min_window_vfwd"] >= B["collapse_vfwd"]
    c["speed"] = v["v_fwd"] >= B["speed_vfwd"]
    c["screen"] = (abs(v["beta_TD"] - B["beta_td"]) <= B["beta_td_tol"]
                   and B["fwd_lo"] <= v["fwd_frac"] <= B["fwd_hi"]
                   and v["min_td"] >= B["min_td"])
    c["stride"] = (not math.isnan(v["stride_s"])
                   and abs(v["stride_s"] / TEMPLATE_PERIOD - 1.0) <= B["stride_tol"])
    if B["r_lo"] is not None and B["r_hi"] is not None:
        # S216 met the gate at R 1.8-2.6 m against a NAMED cell of R ~3 m that
        # was never scored. For a campaign whose point is "the Stage 4 radius
        # has been run at", the radius is a clause, per run, on the Kasa fit.
        c["radius"] = B["r_lo"] <= v["R_fit"] <= B["r_hi"]
    v["clauses"] = c
    v["valid"] = all(c.values())
    return v


def report(rows):
    print("  %-4s %7s %7s %8s %7s %7s %8s %7s %7s %8s  %s" %
          ("run", "kappa", "R_fit", "heading", "fitarc", "v_fwd", "minwin_v",
           "betaTD", "fwd%", "stride", "verdict"))
    for v in rows:
        c = v["clauses"]
        failed = [k for k, ok in c.items() if not ok]
        print("  %-4d %+7.3f %7.2f %8.0f %7.0f %7.3f %8.3f %7.4f %7.1f %8.4f  %s" %
              (v["run"], v["kappa"], v["R_fit"], v["heading_deg"], v["fit_arc"],
               v["v_fwd"], v["min_window_vfwd"], v["beta_TD"], 100 * v["fwd_frac"],
               v["stride_s"], "VALID ARC" if v["valid"] else "fails: " + ",".join(failed)))
    n_valid = sum(1 for v in rows if v["valid"])
    print("\n  valid arcs: %d of %d attempts  (gate: >= %d of <= %d)"
          % (n_valid, len(rows), NEED, MAX_ATTEMPTS))
    if len(rows) > MAX_ATTEMPTS:
        print("  !! more than %d attempts on disk -- only the first %d count, as registered"
              % (MAX_ATTEMPTS, MAX_ATTEMPTS))
        n_valid = sum(1 for v in rows[:MAX_ATTEMPTS] if v["valid"])
    print()
    if n_valid >= NEED:
        print("  GATE MET -- Stage 3 cambered half closes (result note + Index row).")
    elif n_valid >= 3:
        from collections import Counter
        fails = Counter(k for v in rows if not v["valid"] for k, ok in v["clauses"].items() if not ok)
        print("  GATE NOT MET (3-4). Failing clause(s), most frequent first: %s"
              % ", ".join("%s x%d" % kv for kv in fails.most_common()))
        print("  -> the next campaign targets the named clause. No re-registration.")
    else:
        print("  GATE NOT MET (<= 2). The off-ramp question is asked NOW, whatever the date.")
    print("\n  Reported, not gated: flight fraction (#22), roll (S186), kappa vs model")
    print("  (S127), yaw-drift share of the turn (S195). Add them to the log entry.")
    return n_valid


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    good = {"run": 1, "kappa": 0.33, "R_fit": 3.0, "fit_arc": 130.0, "v_fwd": 0.32,
            "rms_mm": 60, "tau": 46, "heading_deg": 195.0, "min_window_vfwd": 0.25,
            "n_windows": 9, "beta_TD": -0.085, "fwd_frac": 0.75, "min_td": 30,
            "stride_s": 0.2650}
    chk("a textbook arc is VALID", evaluate(dict(good))["valid"])
    for key, val, clause in (("heading_deg", 150.0, "arc"),
                             ("fit_arc", 80.0, "arc"),
                             ("min_window_vfwd", 0.05, "collapse"),
                             ("v_fwd", 0.20, "speed"),
                             ("beta_TD", -0.095, "screen"),
                             ("fwd_frac", 0.60, "screen"),
                             ("min_td", 5, "screen"),
                             ("stride_s", 0.2850, "stride")):
        v = dict(good); v[key] = val
        e = evaluate(v)
        chk("%s=%s fails ONLY clause '%s'" % (key, val, clause),
            (not e["valid"]) and [k for k, o in e["clauses"].items() if not o] == [clause])
    # radius clause: inactive by default, active and independent when set
    e0 = evaluate(dict(good))
    chk("no radius clause unless set (clauses: %s)" % ",".join(e0["clauses"]), "radius" not in e0["clauses"])
    B["r_lo"], B["r_hi"] = 2.5, 3.6
    e1 = evaluate(dict(good)); v3 = dict(good); v3["R_fit"] = 2.2; e2 = evaluate(v3)
    chk("R_fit 3.0 passes the [2.5, 3.6] radius clause", e1["clauses"]["radius"] and e1["valid"])
    chk("R_fit 2.2 fails ONLY the radius clause",
        (not e2["valid"]) and [k for k, o in e2["clauses"].items() if not o] == ["radius"])
    B["r_lo"], B["r_hi"] = None, None
    # heading change on a synthetic 200 deg arc, written in load_odom_csv's
    # own column layout (sec, nsec, _, _, x, y, _, qx, qy, qz, qw): the helper
    # must read 200 over the band and IGNORE the 12 s settle before it.
    import tempfile, csv
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "odom_run1.csv")
        tt = np.linspace(0.0, 40.0, 4000)
        yaw = np.radians(200.0) * np.clip((tt - START) / (40.0 - START), 0.0, 1.0)
        yaw = yaw + np.radians(30.0) * (tt < START)       # pre-band junk
        with open(p, "w", newline="") as fh:
            w = csv.writer(fh)
            for t_, y_ in zip(tt, yaw):
                w.writerow([int(t_), int((t_ % 1) * 1e9), 0, 0, 0.0, 0.0, 0,
                            0.0, 0.0, math.sin(y_ / 2), math.cos(y_ / 2)])
        got = heading_change_deg(p)
        chk("synthetic 200 deg arc reads 200 over the band, pre-band ignored "
            "(got %.1f)" % got, abs(got - 200.0) < 1.0)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=os.path.expanduser("~/corgi_runs/stage3_gate"))
    ap.add_argument("--beta-td", type=float, help="S152 screen centre, rad, from S202 cam arm at this lambda")
    ap.add_argument("--beta-tol", type=float, help="half-width, rad (median +- 3 sd)")
    ap.add_argument("--fwd-lo", type=float)
    ap.add_argument("--fwd-hi", type=float)
    ap.add_argument("--r-lo", type=float, help="optional radius clause: R_fit >= this (m)")
    ap.add_argument("--r-hi", type=float, help="optional radius clause: R_fit <= this (m)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    # The screen bands are a REGISTRATION input. The defaults in B are the
    # lambda=10 values (S194) and are WRONG at lambda 15+: a dry run on banked
    # camber_lambda/lam15 put beta_TD at -0.087..-0.091 and fwd at 75-81%, so
    # 2 of 5 healthy cambered runs failed the screen for being cambered (#22
    # trap). Set them from S202 cam arm at the gate lambda, in the registration.
    for k, v in (("beta_td", a.beta_td), ("beta_td_tol", a.beta_tol),
                 ("fwd_lo", a.fwd_lo), ("fwd_hi", a.fwd_hi),
                 ("r_lo", a.r_lo), ("r_hi", a.r_hi)):
        if v is not None:
            B[k] = v
    if a.selftest:
        print("score_stage3_gate.py selftest\n")
        g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL"))
        return 0 if g else 1
    if None in (a.beta_td, a.beta_tol, a.fwd_lo, a.fwd_hi):
        print("!! the S152-screen bands must be passed explicitly (--beta-td --beta-tol "
              "--fwd-lo --fwd-hi), derived from S202 cam arm at this lambda. The "
              "built-in lambda=10 values fail cambered runs for being cambered.")
        return 1
    print("screen bands in use: beta_TD %.4f +- %.4f, fwd [%.2f, %.2f]"
          % (B["beta_td"], B["beta_td_tol"], B["fwd_lo"], B["fwd_hi"]))
    if B["r_lo"] is not None:
        print("radius clause in use: R_fit in [%.2f, %.2f] m" % (B["r_lo"], B["r_hi"]))
    print()
    d = os.path.join(os.path.expanduser(a.base), "cam")
    rows = [r for r in (run_clauses(d, n) for n in range(1, MAX_ATTEMPTS + 3)) if r]
    if not rows:
        print("no runs under %s" % d); return 1
    print("Stage 3 cambered gate -- sustained arcs matching the template (Timeline)\n")
    report(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
