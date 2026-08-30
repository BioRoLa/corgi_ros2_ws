#!/usr/bin/env python3
"""Did the controller actually play the template at the template's rate?

WHY THIS EXISTS -- the half GAIT_SIM does not cover. GAIT_SIM makes the
capture's LENGTH a property of the experiment: the run ends when the sim clock
has advanced N seconds, so CPU load costs wall time, not sim time. It does
nothing for the run's FIDELITY.

The gap is real. gslip_pronk's stride loop advances exactly one template row
per control tick and holds 1 ms of SIM time per tick by sleeping the wall-clock
difference:

    next_time = next_time + 1ms;
    rclcpp::sleep_for(max(0, next_time - now()));      // now() is the sim clock

If the controller is descheduled long enough that sim time overshoots
next_time, the sleep clamps to zero and the loop FREE-RUNS to catch up --
publishing a burst and playing the template FAST in sim time. The driver never
waits for the controller, so sim time advances whether or not commands arrive.
That changes the gait, and it produces a perfectly normal-length capture.

So: measure the period the plant actually ran at, and compare it to the
template's. S153 already did this once by hand and it settled an argument --
autocorrelation of measured beta, theta and contact all read 0.2200 s against
the template's 0.2662 s, which is what exposed that the k12000 leg was
torque-saturated and running its own limit cycle rather than the commanded
trajectory.

⚠ THIS MEASURES TWO THINGS AT ONCE and cannot separate them. A ratio away from
1.0 means the plant's rhythm is not the template's -- that can be controller
starvation (this tool's purpose) OR the plant refusing to follow, as at
k_flight 12000 where the leg is torque-saturated (S153, Open Issue #21). Read
it beside tau_demand_window.py: saturation shows there, starvation does not.

⚠ theta is the cleanest channel. It is a pure template command with no contact
physics in it, so a deviation is about tracking and timing rather than about
the ground. beta and contact are reported too because agreement across all
three is what makes the reading trustworthy.

Usage:
    playback_ratio.py CAPTURE.csv [...] [--period 0.2662] [--tol 0.05]
    playback_ratio.py --dir DIR
    playback_ratio.py --selftest

Exit 0 = within tolerance, 1 = outside it, 2 = could not measure.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

LEG_MOTORS = ("L_Motor", "R_Motor")
TAIL_S = 20.0
V070_STRIDE_S = 0.2662        # S153's figure for the config-of-record template
MIN_SAMPLES = 2000
MIN_LAG_S = 0.10              # ignore the zero-lag lobe and sub-stride ripple
MAX_LAG_S = 0.60


class Unfit(Exception):
    pass


def _series(path):
    """-> (dt, {channel: array}) for one leg's tail. Uses leg A, or whichever
    leg has the most samples -- the template clock is global, so one leg is
    enough and pooling four would smear any per-leg desync into the period."""
    rows = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("motor") != "L_Motor":       # one motor: theta is per leg
                continue
            try:
                rows.setdefault(r["leg"], []).append(
                    (float(r["t"]), float(r["theta"]), float(r["beta"]),
                     float(r["in_contact"])))
            except (ValueError, KeyError):
                continue
    if not rows:
        raise Unfit("no L_Motor rows in %s" % os.path.basename(path))
    leg = max(rows, key=lambda k: len(rows[k]))
    a = np.array(sorted(rows[leg]))
    a = a[a[:, 0] >= a[:, 0].max() - TAIL_S]
    if len(a) < MIN_SAMPLES:
        raise Unfit("only %d samples in the tail" % len(a))
    d = np.diff(a[:, 0])
    dt = float(np.median(d[d > 0])) if np.any(d > 0) else 0.0
    if dt <= 0:
        raise Unfit("non-monotonic or zero timestep")
    return leg, dt, {"theta": a[:, 1], "beta": a[:, 2], "contact": a[:, 3]}


def dominant_period(x, dt):
    """First autocorrelation peak between MIN_LAG_S and MAX_LAG_S, parabolically
    interpolated so the answer is not quantised to the sample interval."""
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    if np.allclose(x, 0.0):
        raise Unfit("channel is constant")
    ac = np.correlate(x, x, mode="full")[len(x) - 1:]
    ac = ac / ac[0]
    lo, hi = int(MIN_LAG_S / dt), min(int(MAX_LAG_S / dt), len(ac) - 2)
    if hi <= lo + 2:
        raise Unfit("window too short for a %.2f-%.2f s lag search"
                    % (MIN_LAG_S, MAX_LAG_S))
    k = lo + int(np.argmax(ac[lo:hi]))
    if k <= 0 or k >= len(ac) - 1:
        raise Unfit("peak at the edge of the lag search")
    y0, y1, y2 = ac[k - 1], ac[k], ac[k + 1]
    denom = y0 - 2.0 * y1 + y2
    shift = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
    return (k + shift) * dt, float(y1)


def run_report(path, period, tol):
    leg, dt, ch = _series(path)
    out, worst = {}, 0.0
    for name in ("theta", "beta", "contact"):
        try:
            p, peak = dominant_period(ch[name], dt)
        except Unfit as e:
            out[name] = (None, None, str(e))
            continue
        ratio = p / period
        out[name] = (p, peak, ratio)
        if name == "theta":
            worst = abs(ratio - 1.0)
    th = out.get("theta", (None, None, None))
    print("  %-14s leg %s, dt %.4f s" % (os.path.basename(path), leg, dt))
    for name in ("theta", "beta", "contact"):
        v = out[name]
        if v[0] is None:
            print("      %-8s unmeasurable -- %s" % (name, v[2]))
        else:
            print("      %-8s period %.4f s   ratio %.4f   ac peak %.2f"
                  % (name, v[0], v[2], v[1]))
    if th[0] is None:
        print("      -> UNMEASURABLE on theta, the channel that matters")
        return 2
    if worst > tol:
        print("      -> !! theta ratio %.4f is outside +-%.0f%%. The plant's"
              % (th[2], tol * 100))
        print("         rhythm is NOT the template's. Either the controller was")
        print("         starved (check the machine's load) or the leg cannot")
        print("         follow (check tau_demand_window.py for saturation).")
        return 1
    print("      -> playback OK (theta within +-%.0f%% of the template)"
          % (tol * 100))
    return 0


def selftest():
    """Recover a KNOWN period, and refuse a signal that has none."""
    ok = True
    dt = 0.001
    t = np.arange(0, 20.0, dt)
    for want in (0.2662, 0.2200, 0.3100):
        # Not a pure sine: the template's theta is a distorted periodic wave,
        # and a method that only works on sinusoids would flatter itself here.
        x = (np.sin(2 * np.pi * t / want)
             + 0.4 * np.sin(4 * np.pi * t / want + 0.7)
             + 0.1 * np.sin(6 * np.pi * t / want))
        got, _peak = dominant_period(x, dt)
        good = abs(got - want) < 5e-4
        ok = ok and good
        print("  period %.4f s -> recovered %.4f s   %s"
              % (want, got, "ok" if good else "FAIL"))
    # The fooling case: white noise has no period, and a tool that always
    # returns its argmax would confidently report one.
    rng = np.random.default_rng(0)
    got, peak = dominant_period(rng.standard_normal(len(t)), dt)
    print("  white noise -> period %.4f s at ac peak %.3f "
          "(peak must be small; the NUMBER is meaningless)" % (got, peak))
    if peak > 0.2:
        ok = False
        print("     FAIL: noise produced a strong spurious peak")
    # And a constant must refuse rather than return anything.
    try:
        dominant_period(np.ones(len(t)), dt)
        ok = False
        print("  constant signal -> FAIL, returned a period")
    except Unfit:
        print("  constant signal -> refuses, ok")
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--dir")
    ap.add_argument("--period", type=float, default=V070_STRIDE_S)
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        print("playback_ratio.py selftest")
        ok = selftest()
        print("\n  SELFTEST %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    paths = list(a.paths)
    if a.dir:
        paths += sorted(glob.glob(os.path.join(os.path.expanduser(a.dir),
                                               "run[0-9].csv")))
    if not paths:
        ap.error("need a capture, or --dir")
    print("playback ratio -- realised stride period vs the template's %.4f s"
          % a.period)
    rc = 0
    for p in paths:
        try:
            rc = max(rc, run_report(p, a.period, a.tol))
        except Unfit as e:
            print("  %s: UNMEASURABLE -- %s" % (os.path.basename(p), e))
            rc = max(rc, 2)
    return rc


if __name__ == "__main__":
    sys.exit(main())
