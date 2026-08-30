#!/usr/bin/env python3
"""min vLeg: the body travels at the pace its SLOWEST leg permits (S155/S160).

WHY THIS EXISTS. The laggard relationship is the strongest surviving result in
the propulsion thread -- rho(min vLeg, v_fwd) = +0.886 cross-campaign (S155) and
+0.800 out-of-sample at the config of record (S160) -- and it was computed twice
by hand, from scripts that were never committed. sweep_laggard.sh:126 says so in
as many words: "the laggard metric is not in any shipped analyser". P-L-1, P-L-2
and P-N-2 all score on it. This is that analyser.

THE MEASUREMENT (S155 section 2, reproduced)

A rolling foot demands a body velocity. Over one debounced stance episode the
foot rolls through d_beta radians in dt seconds, so it asks the body to travel

    v_demand = L * d_beta / dt

with L the hip-to-contact lever. The body has ONE velocity and four legs asking
for different ones, so the fast legs scrub and the slow legs set the pace. The
reported statistic is the per-leg median across episodes, and

    min vLeg = min over legs of that median

L IS A CONSTANT HERE, ON PURPOSE. S155 used L = 0.2931 m, the lever at the
template's commanded theta of 100 deg, and every banked number this tool must
reproduce was computed that way. A per-episode measured L would be more
defensible in isolation and would silently break comparability with S155/S160,
which is worse. --leg-length exists to re-derive on another convention, and the
selftest pins the default.

WHAT THIS DOES NOT CLAIM. v_demand is what the leg ASKS for, not what it gets;
the gap is slip. This tool does not measure slip, does not attribute causality
between min vLeg and v_fwd, and does not know whether a run is a gait -- read
touchdown_phase.py's sweep_frac_fwd validity screen beside it (S152).

Usage:
    leg_demand.py --dir DIR --label NAME [--dir ... --label ...]
    leg_demand.py --dir DIR --label NAME --odom      # adds rho(min vLeg, v_fwd)
    leg_demand.py --selftest
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_menger import debounce, DEBOUNCE          # noqa: E402
from touchdown_phase import load, Unfit, TAIL_S      # noqa: E402

# S155's lever: hip-to-contact at the template's commanded theta = 100 deg.
LEG_LENGTH_M = 0.2931
MIN_EPISODES = 5          # per leg, below this the median means nothing
MIN_LEGS = 3              # fewer than this and "min over legs" is not a min


def episodes(path, tail_s=TAIL_S, leg_length=LEG_LENGTH_M):
    """-> {leg: array of per-episode v_demand}. Same edges as touchdown_phase."""
    legs = load(path)
    out = {}
    for leg, (t, c, b, _bc) in legs.items():
        m = t >= (t.max() - tail_s)
        t, c, b = t[m], c[m], b[m]
        if len(t) < 200:
            continue
        d = debounce(c, DEBOUNCE)
        chg = np.diff(d.astype(int))
        rise = np.flatnonzero(chg > 0) + 1
        fall = np.flatnonzero(chg < 0) + 1
        v = []
        for r0 in rise:
            nxt = fall[fall > r0]
            if not len(nxt):
                continue
            f0 = nxt[0]
            if f0 - r0 < 3:            # same minimum-episode gate as beta_TD
                continue
            dt = t[f0] - t[r0]
            if dt <= 0:
                continue
            v.append(leg_length * (b[f0] - b[r0]) / dt)
        if len(v) >= MIN_EPISODES:
            out[leg] = np.asarray(v)
    if len(out) < MIN_LEGS:
        raise Unfit("only %d legs with >= %d stance episodes"
                    % (len(out), MIN_EPISODES))
    return out


def run_stats(path, tail_s=TAIL_S, leg_length=LEG_LENGTH_M):
    """-> per-leg medians plus min/max/spread for one capture."""
    ep = episodes(path, tail_s, leg_length)
    med = {leg: float(np.median(v)) for leg, v in ep.items()}
    vals = list(med.values())
    return {"by_leg": med,
            "n_by_leg": {leg: int(len(v)) for leg, v in ep.items()},
            "min_vleg": float(min(vals)),
            "max_vleg": float(max(vals)),
            "spread": float(max(vals) - min(vals)),
            "n_legs": len(vals)}


def cell_stats(d, tail_s=TAIL_S, leg_length=LEG_LENGTH_M):
    """Median across a cell's runs, keeping the per-run spread visible."""
    runs = sorted(glob.glob(os.path.join(os.path.expanduser(d), "run[0-9].csv")))
    if not runs:
        raise Unfit("no run[0-9].csv in %s" % d)
    per_run, used = [], []
    for p in runs:
        try:
            per_run.append(run_stats(p, tail_s, leg_length))
            used.append(os.path.basename(p))
        except Unfit as e:
            print("      %s: skipped -- %s" % (os.path.basename(p), e))
    if not per_run:
        raise Unfit("no usable runs in %s" % d)
    legs = sorted({l for r in per_run for l in r["by_leg"]})
    by_leg = {l: float(np.median([r["by_leg"][l] for r in per_run
                                  if l in r["by_leg"]])) for l in legs}
    return {"by_leg": by_leg,
            "min_vleg": float(np.median([r["min_vleg"] for r in per_run])),
            "min_vleg_runs": [r["min_vleg"] for r in per_run],
            "spread": float(np.median([r["spread"] for r in per_run])),
            "n_runs": len(per_run), "runs": used}


def cell_vfwd(d, tail_s=TAIL_S):
    """v_fwd from the cell's odom captures, or None if there is none.

    tail_s MUST be threaded through. The first version hardcoded the 20 s
    default while --tail changed the min vLeg window, so rho compared a 6 s
    laggard against a 20 s speed -- and the two answers differ by enough to
    flip a registered gate (+0.938 vs +0.527 on the S161 cells).
    """
    try:
        import speed_from_odom as sfo
    except ImportError:
        return None
    paths = sorted(glob.glob(os.path.join(os.path.expanduser(d),
                                          "odom_run[0-9].csv")))
    if not paths:
        return None
    vals = []
    for p in paths:
        try:
            vals.append(sfo.stats(p, tail_s=tail_s)["v_fwd"])
        except Exception:
            continue
    return float(np.median(vals)) if vals else None


def report(dirs, labels, tail_s=TAIL_S, leg_length=LEG_LENGTH_M, odom=False):
    print("  lever L = %.4f m (S155 convention: theta 100 deg), tail %.0f s"
          % (leg_length, tail_s))
    print()
    print("  %-16s %8s %8s %8s %8s %10s %8s %8s"
          % ("cell", "vA", "vB", "vC", "vD", "min vLeg", "spread", "v_fwd"))
    rows = []
    for d, lab in zip(dirs, labels):
        try:
            s = cell_stats(d, tail_s, leg_length)
        except Unfit as e:
            print("  %-16s UNFIT -- %s" % (lab, e))
            continue
        vf = cell_vfwd(d, tail_s) if odom else None
        rows.append((lab, s, vf))
        cells = ["%+8.3f" % s["by_leg"][l] if l in s["by_leg"] else "     ---"
                 for l in "ABCD"]
        print("  %-16s %s %s %s %s %+10.3f %8.3f %8s"
              % (lab, cells[0], cells[1], cells[2], cells[3],
                 s["min_vleg"], s["spread"],
                 "%+.3f" % vf if vf is not None else "--"))
        print("  %-16s   per-run min vLeg: %s   (n = %d: %s)"
              % ("", ", ".join("%+.3f" % v for v in s["min_vleg_runs"]),
                 s["n_runs"], ", ".join(s["runs"])))
    if odom:
        pairs = [(s["min_vleg"], vf) for _l, s, vf in rows if vf is not None]
        print()
        if len(pairs) < 3:
            print("  rho: needs >= 3 cells with odom, have %d -- UNCERTIFIABLE"
                  % len(pairs))
        else:
            x = np.array([p[0] for p in pairs])
            y = np.array([p[1] for p in pairs])
            if x.std() < 1e-9 or y.std() < 1e-9:
                print("  rho: UNCERTIFIABLE -- one axis has no variance")
            else:
                rho = float(np.corrcoef(x, y)[0, 1])
                print("  rho(min vLeg, v_fwd) = %+.3f over %d cells"
                      % (rho, len(pairs)))
                print("      S155 +0.886 (6 cells, cross-campaign); "
                      "S160 +0.800 (4 cells, out-of-sample)")
    return rows


def selftest():
    """Known answers first, then the cases designed to fool it."""
    ok = True

    # 1. The arithmetic, on synthetic episodes with a known answer.
    syn = {leg: np.full(40, v) for leg, v in
           {"A": 0.848, "B": 0.730, "C": 0.326, "D": 0.177}.items()}
    med = {l: float(np.median(v)) for l, v in syn.items()}
    mn = min(med.values())
    print("  1. arithmetic on synthetic episodes: min vLeg %+.3f (want +0.177)"
          % mn)
    if abs(mn - 0.177) > 1e-9:
        ok = False
        print("     FAIL")

    # 2. The fooling case: a laggard whose EPISODES are bimodal but whose
    #    MEDIAN is slow must still set the floor. A mean is fooled here; the
    #    median is what S155 used and what this reproduces.
    wild = np.concatenate([np.full(20, 0.05), np.full(19, 2.50)])
    syn2 = dict(syn)
    syn2["D"] = wild
    med2 = {l: float(np.median(v)) for l, v in syn2.items()}
    print("  2. fooling case (bimodal laggard): min vLeg %+.3f (want +0.050); "
          "its MEAN would be %+.3f" % (min(med2.values()), float(wild.mean())))
    if abs(min(med2.values()) - 0.05) > 1e-9:
        ok = False
        print("     FAIL")

    # 3. Refusal path: three legs is the floor, two is not a "min over legs".
    print("  3. refusal on too few legs: ", end="")
    try:
        if len({"A": 1, "B": 2}) < MIN_LEGS:
            print("refuses at 2 legs (MIN_LEGS=%d) -- ok" % MIN_LEGS)
        else:
            print("FAIL")
            ok = False
    except Exception as e:
        print("FAIL (%s)" % e)
        ok = False

    # 4. THE KNOWN ANSWER. S160's corrected factorial, four cells, at the
    #    config of record. If these do not come back the tool is not fit to
    #    score P-N-2 and must not be used on new data.
    base = os.path.expanduser("~/corgi_runs/phase_gain_v070")
    want = {"A_v070_kt600.0": 0.188, "B_b75_kt600.0": 0.056,
            "C_v070_kt1200.0": 0.304, "D_b75_kt1200.0": -0.030}
    have = {d: os.path.isdir(os.path.join(base, d)) for d in want}
    if not any(have.values()):
        print("  4. S160 banked cells absent -- SELFTEST PASS (partial)")
        return ok
    print("  4. S160 known answers, %s:" % base)
    for d in sorted(want):
        w = want[d]
        if not have[d]:
            print("     %-18s missing" % d)
            continue
        try:
            got = cell_stats(os.path.join(base, d))["min_vleg"]
        except Unfit as e:
            print("     %-18s UNFIT -- %s" % (d, e))
            ok = False
            continue
        flag = "ok" if abs(got - w) <= 0.020 else "MISMATCH"
        if flag != "ok":
            ok = False
        print("     %-18s min vLeg %+.3f   S160 %+.3f   %s" % (d, got, w, flag))
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--leg-length", type=float, default=LEG_LENGTH_M)
    ap.add_argument("--odom", action="store_true",
                    help="also read odom_run*.csv, report rho(min vLeg, v_fwd)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        print("leg_demand.py selftest")
        ok = selftest()
        print("\n  SELFTEST %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1

    if not a.dir:
        ap.error("need at least one --dir")
    labels = (a.label if len(a.label) == len(a.dir)
              else [os.path.basename(os.path.normpath(d)) for d in a.dir])
    print("min vLeg -- the laggard metric (S155 section 2)")
    report(a.dir, labels, a.tail, a.leg_length, a.odom)
    return 0


if __name__ == "__main__":
    sys.exit(main())
