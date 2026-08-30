#!/usr/bin/env python3
"""Open Issue #15 step 1: what torque does the ABAD joint actually carry in the
pronk? Read from existing captures -- zero simulator time.

#15 predicts 24-50 N.m through stance on a 91.7 mm cantilever, against a
44.25 N.m clamp, and warns that setAvailableTorque clips SILENTLY -- a
saturated ABAD would be invisible in the motion. It is not invisible in the
capture: the torque CSV carries `tau_demand` (pre-clamp) and `tau_applied`
(post-clamp) for motor == ABAD on every leg. Demand above applied IS the clip.

Reads only rows with motor == ABAD. Reports, per run and per leg, over the
steady band [t0+12, end]:
    demand p99.5 / max      pre-clamp |tau_demand|, stance rows only
    applied p99.5 / max     post-clamp |tau_applied|
    clipped %               stance rows where |demand| > |applied| + 0.01
    demand p99.5, flight    for contrast -- the cantilever is loaded in stance

Usage:
    abad_torque.py --selftest
    abad_torque.py --dir <cell> --label <name> [--dir ...] [--clamp 44.25]
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

START = 12.0
CLAMP = 44.25


def load_abad(path):
    """-> dict leg -> (t, contact, demand, applied) for motor == ABAD."""
    out = {}
    with open(path, newline="") as fh:
        r = csv.DictReader(fh)
        for row in r:
            if row.get("motor") != "ABAD":
                continue
            try:
                out.setdefault(row["leg"], []).append(
                    (float(row["t"]), float(row["in_contact"]),
                     float(row["tau_demand"]), float(row["tau_applied"])))
            except (ValueError, KeyError):
                continue
    return {k: np.array(v) for k, v in out.items() if v}


def stats(path, clamp=CLAMP):
    legs = load_abad(path)
    if not legs:
        return None
    t0 = min(a[0, 0] for a in legs.values())
    per = {}
    for leg, a in sorted(legs.items()):
        m = a[:, 0] >= t0 + START
        a = a[m]
        if len(a) < 200:
            continue
        st = a[:, 1] > 0.5
        d, ap = np.abs(a[:, 2]), np.abs(a[:, 3])
        ds, aps = d[st], ap[st]
        if len(ds) < 50:
            continue
        per[leg] = {
            "d_p995": float(np.percentile(ds, 99.5)), "d_max": float(ds.max()),
            "a_p995": float(np.percentile(aps, 99.5)), "a_max": float(aps.max()),
            "clip_pct": 100.0 * float(np.mean(ds > aps + 0.01)),
            "over_clamp_pct": 100.0 * float(np.mean(ds > clamp)),
            "d_p995_flight": float(np.percentile(d[~st], 99.5)) if (~st).sum() > 50 else float("nan"),
            "n_stance": int(st.sum()),
        }
    if not per:
        return None
    pooled = np.concatenate([np.abs(a[a[:, 0] >= t0 + START][:, 2])
                             [a[a[:, 0] >= t0 + START][:, 1] > 0.5] for a in legs.values()])
    return {"per_leg": per, "pooled_d_p995": float(np.percentile(pooled, 99.5)),
            "pooled_d_max": float(pooled.max())}


def selftest():
    ok = True

    def chk(name, good):
        nonlocal ok
        ok = ok and good
        print("  %s %s" % ("ok " if good else "FAIL", name))

    import tempfile
    rng = np.random.RandomState(0)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "run1.csv")
        with open(p, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "leg", "motor", "in_contact", "t_stiff", "t_damp", "t_ff",
                        "tau_demand", "tau_applied", "pos_error", "kp", "kd",
                        "theta", "beta", "gamma"])
            t = np.arange(0.0, 30.0, 0.01)
            for i, ti in enumerate(t):
                st = 1.0 if (i % 26) < 16 else 0.0          # 62% stance
                # leg A: stance demand 30 +- noise, never clips; flight 5
                dA = (30.0 + rng.normal(0, 1.0)) if st else 5.0
                w.writerow([ti, "A", "ABAD", st, 0, 0, 0, dA, dA, 0, 0, 0, 0, 0, 0])
                # leg B: stance demand 60, applied clipped at 44.25 -> 100% clipped
                dB = 60.0 if st else 5.0
                w.writerow([ti, "B", "ABAD", st, 0, 0, 0, dB, min(dB, CLAMP), 0, 0, 0, 0, 0, 0])
                # a non-ABAD row that must be ignored even with a huge torque
                w.writerow([ti, "A", "L_Motor", st, 0, 0, 0, 500.0, 500.0, 0, 0, 0, 0, 0, 0])
        s = stats(p)
        A, B = s["per_leg"]["A"], s["per_leg"]["B"]
        chk("leg A stance demand p99.5 ~ 32.5 (30 + 2.6 sd), got %.1f" % A["d_p995"],
            31.0 < A["d_p995"] < 34.0)
        chk("leg A never clips (got %.1f%%)" % A["clip_pct"], A["clip_pct"] == 0.0)
        chk("leg A flight demand 5.0 (got %.1f)" % A["d_p995_flight"],
            abs(A["d_p995_flight"] - 5.0) < 1e-6)
        chk("leg B demand 60 but applied capped at %.2f (got %.2f)" % (CLAMP, B["a_max"]),
            abs(B["a_max"] - CLAMP) < 1e-6)
        chk("leg B clipped on 100%% of stance rows (got %.0f%%)" % B["clip_pct"],
            B["clip_pct"] == 100.0)
        chk("non-ABAD rows ignored (pooled max %.1f, not 500)" % s["pooled_d_max"],
            s["pooled_d_max"] < 100.0)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--clamp", type=float, default=CLAMP)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("abad_torque.py selftest\n")
        g = selftest()
        print("\n  SELFTEST %s" % ("PASS" if g else "FAIL"))
        return 0 if g else 1
    if not a.dir:
        ap.error("need --dir, or --selftest")
    print("ABAD joint torque in the pronk, stance rows, band [t0+%.0f, end]. Clamp %.2f N.m"
          % (START, a.clamp))
    print("#15 predicted 24-50 N.m through stance.\n")
    print("  %-16s %4s %4s %8s %7s %8s %7s %6s %7s %8s" %
          ("cell", "run", "leg", "dem p995", "dem max", "app p995", "app max",
           "clip%", ">clamp%", "flt p995"))
    for i, d in enumerate(a.dir):
        d = os.path.expanduser(d)
        name = a.label[i] if i < len(a.label) else os.path.basename(d.rstrip("/"))
        pooled = []
        for tq in sorted(glob.glob(os.path.join(d, "run[0-9].csv"))):
            s = stats(tq, a.clamp)
            if not s:
                continue
            n = os.path.basename(tq)[3:-4]
            for leg, v in s["per_leg"].items():
                print("  %-16s %4s %4s %8.2f %7.2f %8.2f %7.2f %6.1f %7.1f %8.2f" %
                      (name, n, leg, v["d_p995"], v["d_max"], v["a_p995"], v["a_max"],
                       v["clip_pct"], v["over_clamp_pct"], v["d_p995_flight"]))
            pooled.append(s["pooled_d_p995"])
        if pooled:
            print("  %-16s pooled demand p99.5 across runs: median %.2f  range %.2f-%.2f\n"
                  % (name, np.median(pooled), min(pooled), max(pooled)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
