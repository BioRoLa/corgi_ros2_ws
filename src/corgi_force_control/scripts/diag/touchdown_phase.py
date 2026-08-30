"""At what point in the beta cycle does the foot actually land and leave?

S110 left one thing unexplained: the +65 ms shifted gait FLIES (36-41%
air) and still rolls backward in every stance bin, so its contact is
centred on the beta backswing -- but the label shift changes only gains,
never the trajectory. Something moves touchdown indirectly.

The template intends, per its own labels (v070, 265 rows, stance 0-107):
    touchdown at beta = -0.1614 rad  (stance onset row)
    liftoff   at beta = +0.1609 rad  (last stance row)
i.e. the foot lands at the BACK of the swing and rolls FORWARD through
stance. Measuring the beta actually held at each debounced rising and
falling edge says whether the real gait does that, and if not, where it
lands instead.

Reported per leg and pooled:
    beta_TD   beta at the debounced touchdown edge
    beta_LO   beta at the debounced liftoff edge
    sweep     beta_LO - beta_TD, signed: POSITIVE rolls the robot
              forward, negative rolls it backward
    on_fore   fraction of contact samples taken while beta is INCREASING
              (the forward sweep) -- 1.0 means the foot is only ever
              down during the forward stroke, 0.0 only during backswing

Read-only. Uses the same debounce as every touchdown detector here.
"""
import argparse
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
MIN_STRIDES_SD = 8      # a leg contributes to the sd only above this


class Unfit(Exception):
    pass


def _td_spread(td_by_leg):
    """-> (within_sd, {leg: sd}, between_sd) for per-leg beta_TD lists.

    WITHIN is the quantity that answers "how repeatable is touchdown": each
    leg's own sd, pooled across legs by degrees of freedom

        s_w = sqrt( sum_l (n_l - 1) s_l^2 / sum_l (n_l - 1) )

    i.e. the sd of the residual after removing each leg's own fixed point.
    df weighting rather than a plain mean of the four sds, because n_l is not
    equal (76-106 per leg per run on label_shift) and a mean of sds is biased
    low against the RMS.

    BETWEEN is the sd of the four per-leg means -- gait asymmetry. Returned
    separately so it can be seen, and NEVER added into s_w. This is the whole
    point: a flat-list sd over all four legs measures the asymmetry, not the
    repeatability, and on planted data it comes out 8.3x too large.

    Never raises: a run with no leg above MIN_STRIDES_SD returns nan rather
    than refusing, so this cannot reject input that stats() already accepted.
    """
    usable = {l: np.asarray(v) for l, v in td_by_leg.items()
              if len(v) >= max(2, MIN_STRIDES_SD)}
    if not usable:
        return float("nan"), {}, float("nan")
    sd_by_leg = {l: float(np.std(v, ddof=1)) for l, v in usable.items()}
    den = sum(len(v) - 1 for v in usable.values())
    num = sum((len(v) - 1) * sd_by_leg[l] ** 2 for l, v in usable.items())
    within = float(np.sqrt(num / den)) if den > 0 else float("nan")
    mus = [float(np.mean(v)) for v in usable.values()]
    between = float(np.std(mus, ddof=1)) if len(mus) > 1 else 0.0
    return within, sd_by_leg, between


def load(path):
    """-> {leg: (t, contact, beta_meas, beta_cmd)}.

    `beta` in the capture is leg.get_states(), i.e. MEASURED. The
    COMMANDED beta is recovered as beta_meas + 0.5*(e_L + e_R), where e
    is each hip motor's pos_error = phi_des - phi_fb: beta is the COMMON
    mode of the two hip motors (theta is the differential), so their
    mean error is the beta error. Same construction as
    report_kt_sweep.py's `dbeta = 0.5 * (eL + eR)`.

    This separates "the foot lands late in the beta cycle" (a timing
    fault, fixable by rotating beta) from "the leg has not reached its
    commanded beta yet" (a tracking fault, which rotating beta would
    make worse).

    ⚠ THE MIRROR IS NOT OPTIONAL. pos_error is in MOTOR coordinates and
    CORGI_DIRBETA_TRANSFORM mirrors beta on the right pair, so the raw
    common mode is sign-flipped on B and C. Measured 2026-08-20: the
    error at touchdown is {A,D} +0.13 and {B,C} -0.13 in every cell --
    pooling without the mirror averages them to ~0 and reports a
    confident "the leg tracks its command", which is false. dir_beta =
    {A:+1, B:-1, C:-1, D:+1}, the same partition as roll_sign/steer_sign.
    """
    per = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["motor"] not in ("L_Motor", "R_Motor"):
                continue
            try:
                d = per.setdefault((r["leg"], float(r["t"])), {})
                d["c"] = int(r["in_contact"])
                d["b"] = float(r["beta"])
                d["e" + r["motor"][0]] = float(r["pos_error"])
            except (ValueError, KeyError):
                continue
    byleg = {}
    for (leg, t), d in per.items():
        if "eL" not in d or "eR" not in d:
            continue
        bcmd = d["b"] + DIR_BETA[leg] * 0.5 * (d["eL"] + d["eR"])
        byleg.setdefault(leg, []).append((t, d["c"], d["b"], bcmd))
    out = {}
    for leg, rows in byleg.items():
        rows.sort()
        a = np.array(rows)
        if len(a) < 500:
            continue
        out[leg] = (a[:, 0], a[:, 1].astype(bool), a[:, 2], a[:, 3])
    if not out:
        raise Unfit("no usable legs")
    return out


def stats(path, tail_s=TAIL_S):
    legs = load(path)
    td, lo, sweep, fore, tdc = [], [], [], [], []
    td_by_leg = {}
    for leg, (t, c, b, bc) in legs.items():
        m = t >= (t.max() - tail_s)
        t, c, b, bc = t[m], c[m], b[m], bc[m]
        if len(t) < 200:
            continue
        d = debounce(c, DEBOUNCE)
        chg = np.diff(d.astype(int))
        rise = np.flatnonzero(chg > 0) + 1
        fall = np.flatnonzero(chg < 0) + 1
        td_leg = []
        for r0 in rise:
            nxt = fall[fall > r0]
            if not len(nxt):
                continue
            f0 = nxt[0]
            if f0 - r0 < 3:
                continue
            td.append(b[r0])
            td_leg.append(b[r0])
            tdc.append(bc[r0])
            lo.append(b[f0])
            sweep.append(b[f0] - b[r0])
        if td_leg:
            td_by_leg[leg] = np.asarray(td_leg)
        # fraction of contact time spent on the forward (rising) stroke
        db = np.diff(b)
        cm = d[:-1]
        if cm.sum() > 50:
            fore.append(float((db[cm] > 0).mean()))
    if len(td) < 20:
        raise Unfit(f"only {len(td)} stance episodes")
    sd_w, sd_leg, sd_b = _td_spread(td_by_leg)
    return {"beta_TD": float(np.median(td)), "beta_LO": float(np.median(lo)),
            "beta_TD_cmd": float(np.median(tdc)),
            "track_err": float(np.median(tdc)) - float(np.median(td)),
            "sweep": float(np.median(sweep)),
            "on_fore": float(np.mean(fore)) if fore else np.nan,
            "n": len(td),
            # --- added 2026-08-22 (S149 Phase B). Additive only: every key
            # above is byte-identical to before, verified side by side on
            # label_shift run1/run2/run3. The per-stride beta_TD list was
            # already being built here and thrown away at the median; this
            # keeps its spread, which IS the return-map state variable.
            "beta_TD_sd": sd_w,             # within-leg, df-pooled
            "beta_TD_sd_by_leg": sd_leg,    # {leg: sd}
            "beta_TD_sd_between": sd_b,     # sd of the per-leg means
            "n_TD_by_leg": {l: int(len(v)) for l, v in td_by_leg.items()},
            "n_legs_sd": len(sd_leg),
            # --- added 2026-08-22 (S152). PROPULSION, and it is a VALIDITY
            # screen, not a nice-to-have. `sweep` is signed and positive rolls
            # the robot FORWARD (see this file's own header). The banked
            # k_tangential 1200 arm has median sweep -0.02/-0.04 with only
            # 50%/49% of episodes rolling forward -- it does not propel, and it
            # was ALSO the arm with the tightest sd(beta_TD), because a gait
            # that stops locomoting has tighter touchdown angles trivially.
            # Never score sd(beta_TD) without reading this beside it.
            "sweep_frac_fwd": float(np.mean(np.asarray(sweep) > 0.0))}


def _plant(path, n=16000, dt=0.001, period=200, dur=40, amp=0.20,
           offs=(0.10, 0.25, 0.40, 0.55), jit=12, seed=20260822):
    """Write a synthetic capture whose per-leg beta_TD sd is known exactly.

    beta is a sawtooth -amp -> +amp of `period` samples, so the beta held at
    sample i is EXACTLY -amp + 2*amp*((i mod period)/period). Leg l touches
    down at k*period + offs[l]*period + jitter, jitter an integer. Therefore

        beta_TD(k) = -amp + 2*amp*(base_l + jitter_k)/period

    and the per-leg sd is 2*amp/period times the sd of the jitter -- closed
    form, no estimation. The four `offs` are deliberately far apart: that
    offset is the FOOLING CASE, because it is what a flat-list sd over all
    four legs would report instead of repeatability.

    pos_error is planted as eL = eR = DIR_BETA[l]*0.13, so the mirror-corrected
    command is beta + 0.13 on EVERY leg (DIR_BETA**2 = 1). Drop the DIR_BETA
    factor in load() and this becomes +0.13 on {A,D} and -0.13 on {B,C}, and
    the pooled track_err collapses toward 0 -- exactly the failure load()'s
    docstring warns about.

    -> {leg: (starts, jitters)} so the caller can compute truth.
    """
    rng = np.random.RandomState(seed)
    idx = np.arange(n)
    beta = -amp + 2.0 * amp * ((idx % period) / float(period))
    truth = {}
    contact = {}
    for l, o in zip(LEGS, offs):
        base = int(round(o * period))
        ks = np.arange(1, n // period - 1)
        j = np.clip(rng.randint(-jit, jit + 1, size=len(ks)), -jit, jit)
        starts = ks * period + base + j
        c = np.zeros(n, dtype=bool)
        for st in starts:
            c[st:st + dur] = True
        contact[l], truth[l] = c, (starts, j, base)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "leg", "motor", "in_contact", "beta", "pos_error"])
        for i in range(n):
            t = i * dt
            for l in LEGS:
                e = DIR_BETA[l] * 0.13
                for m in ("L_Motor", "R_Motor"):
                    w.writerow(["%.6f" % t, l, m, int(contact[l][i]),
                                "%.9f" % beta[i], "%.9f" % e])
    return truth, beta, period, amp


def selftest():
    """Gate the spread statistic on a signal whose answer is known."""
    import tempfile
    fails = 0
    d = tempfile.mkdtemp(prefix="tdp_selftest_")
    path = os.path.join(d, "run1.csv")
    truth, beta, period, amp = _plant(path)
    r = stats(path)

    # 1. every leg must actually have contributed the strides we planted.
    #    A starved leg makes the sd check fail for a reason that has nothing
    #    to do with the statistic -- name that failure instead of hitting it.
    counts = r["n_TD_by_leg"]
    print("  stride counts %s" % dict(sorted(counts.items())))
    if len(counts) != 4 or min(counts.values()) < 60:
        print("  FAIL: fixture starved a leg"); fails += 1

    # 2/3. within and between sd, against closed form on the SAME jitters.
    scale = 2.0 * amp / float(period)
    per_leg, mus = {}, []
    for l, (_st, j, base) in truth.items():
        jj = j[:counts.get(l, 0)] if l in counts else j
        per_leg[l] = scale * float(np.std(jj, ddof=1))
        mus.append(-amp + 2.0 * amp * (base + float(np.mean(jj))) / period)
    den = sum(counts[l] - 1 for l in counts)
    num = sum((counts[l] - 1) * per_leg[l] ** 2 for l in counts)
    want_w = float(np.sqrt(num / den))
    want_b = float(np.std(mus, ddof=1))
    print("  within  sd %.6f rad (planted %.6f)" % (r["beta_TD_sd"], want_w))
    print("  between sd %.6f rad (planted %.6f)" % (r["beta_TD_sd_between"], want_b))
    if abs(r["beta_TD_sd"] - want_w) > 2e-3:
        print("  FAIL: within-leg sd does not match the planted value"); fails += 1
    if abs(r["beta_TD_sd_between"] - want_b) > 5e-3:
        print("  FAIL: between-leg sd does not match the planted value"); fails += 1

    # 4. THE FOOLING CASE: a flat-list sd measures asymmetry, not repeatability.
    flat = float(np.hypot(r["beta_TD_sd"], r["beta_TD_sd_between"]))
    ratio = flat / r["beta_TD_sd"] if r["beta_TD_sd"] else float("inf")
    print("  a flat pooled sd would be %.4f = %.1fx the within-leg sd,"
          " and is NOT what is reported" % (flat, ratio))
    if ratio < 3.0:
        print("  FAIL: fixture does not separate the two -- weaken nothing,"
              " widen `offs`"); fails += 1

    # 5. the DIR_BETA mirror must survive into track_err.
    print("  track_err %+.4f (mirror honoured; +0.1300 expected)" % r["track_err"])
    if abs(r["track_err"] - 0.13) > 1e-3:
        print("  FAIL: DIR_BETA mirror is not being applied"); fails += 1

    # 5b. propulsion: the planted sawtooth rises through every stance, so
    #     every episode rolls forward and the fraction must be exactly 1.0.
    print("  sweep_frac_fwd %.3f (planted sawtooth rises in stance -> 1.000)"
          % r["sweep_frac_fwd"])
    if abs(r["sweep_frac_fwd"] - 1.0) > 1e-9:
        print("  FAIL: propulsion fraction is not 1.0 on a purely forward plant")
        fails += 1

    # 6. every pre-existing key must still be there.
    old = ("beta_TD", "beta_LO", "beta_TD_cmd", "track_err", "sweep",
           "on_fore", "n")
    missing = [k for k in old if k not in r]
    print("  all %d pre-existing keys present: %s"
          % (len(old), "yes" if not missing else missing))
    if missing:
        print("  FAIL"); fails += 1

    # 7. a too-short run must still be REFUSED, not silently scored.
    short = os.path.join(d, "run2.csv")
    _plant(short, n=1200, period=200)
    try:
        stats(short)
        print("  FAIL: a 4-stride run was accepted"); fails += 1
    except Unfit as e:
        print("  short run -> Unfit (%s)" % e)

    print("")
    print("SELFTEST PASS" if not fails else "SELFTEST FAIL (%d)" % fails)
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", action="append")
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        raise SystemExit(selftest())
    if not args.dir:
        ap.error("need --dir or --selftest")
    print(f"template intends: beta_TD {TPL_TD:+.4f} -> beta_LO {TPL_LO:+.4f}"
          f"  (sweep {TPL_LO-TPL_TD:+.4f}, forward)\n")
    print(f"{'cell':30} {'TD_meas':>9} {'TD_cmd':>9} {'track':>8} "
          f"{'beta_LO':>9} {'sweep':>9} {'n':>5} {'sdTD':>8} {'sdBTW':>8} "
          f"{'%fwd':>6}")
    for i, d in enumerate(args.dir):
        lab = args.label[i] if i < len(args.label) else os.path.basename(
            d.rstrip("/"))
        vals = []
        for p in sorted(glob.glob(os.path.join(os.path.expanduser(d),
                                               "run[0-9].csv"))):
            try:
                vals.append(stats(p))
            except Unfit as e:
                print(f"  {os.path.basename(p)}: REFUSED -- {e}")
        if not vals:
            continue
        f = lambda k: float(np.median([v[k] for v in vals]))
        print(f"{lab:30} {f('beta_TD'):+9.4f} {f('beta_TD_cmd'):+9.4f} "
              f"{f('track_err'):+8.4f} {f('beta_LO'):+9.4f} "
              f"{f('sweep'):+9.4f} {sum(v['n'] for v in vals):5d} "
              f"{f('beta_TD_sd'):8.4f} {f('beta_TD_sd_between'):8.4f} "
              f"{100*f('sweep_frac_fwd'):5.1f}%"
              f"{'   <-- DOES NOT PROPEL' if f('sweep') <= 0 or f('sweep_frac_fwd') < 0.55 else ''}")


if __name__ == "__main__":
    main()
