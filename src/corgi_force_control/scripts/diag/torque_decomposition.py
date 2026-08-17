"""What is the 5-8x torque erosion actually MADE OF?

The single number holding the trot shut is the ratio between what the robot
demands and what the G-SLIP model predicts: 80-121 N.m against 15.43. Every
template-side and hardware-side lever found so far -- softest viable spring,
relaxed duty guard, and even a 6:1 -> 9:1 gearbox -- together tolerates only
about 1.87x, short of even the 2.33x lower bound. So nothing moves until this
factor is attributed.

check_torque_phase.py already answered WHEN: mid-stance, at peak compression,
not at touchdown (flight 0-1% saturated, first stance bin 3-14%, 0.20-0.60
stance progress 45-57%). This answers WHAT, by splitting the applied torque
into the three terms the driver actually sums:

    tau_demand = t_stiff + t_damp + t_ff

    t_stiff = kp * (phi_des - phi_fb)   tracking error against the impedance
    t_damp  = kd * (-phi_dot)           damping
    t_ff    = J^T (F_des + M(-acc)) + off-diagonal coupling, from force_control

WHY THE DRIVER AND NOT force_control.cpp

force_control publishes GAINS and a feedforward torque; it never forms the sum.
The kp*error term -- the one mid-stance saturation points at -- is applied
downstream in corgi_driver._apply_torque_control, which is the only place the
full torque exists. It is also the only place the UNCLIPPED demand is visible:
motor/state carries the post-clamp value, and a clipped signal cannot be told
from a satisfied one.

Consequence worth stating: because tau_demand is captured before the clamp,
this does NOT need the ceiling raised. Run it at the real 29.5 N.m and the
demand is still readable. That is a different experiment from the ceiling
sweep, which had to raise the limit to see past the clip and in doing so
changed the trajectory it was measuring.

WHAT EACH OUTCOME WOULD MEAN

  t_stiff dominates    the impedance loop is fighting its own tracking error.
                       Consistent with "less grip is faster" and with theta
                       overshooting its command. Controller-side fix, and the
                       trot's erosion is not a law of nature.
  t_ff dominates       the commanded force itself is large -- the model's
                       quasi-static prediction is simply wrong about the force,
                       and the template is the thing to change.
  t_damp dominates     damping is doing work it should not be; b_radial was
                       deliberately zeroed in the energy audit, so a large
                       damping share would point at b_tangential / b_lateral.

Run:
    export CORGI_TORQUE_DEBUG=1
    # ... run the gait ...
    python3 torque_decomposition.py [/tmp/corgi_torque_terms.csv]
"""
import csv
import os
import sys
from collections import defaultdict

import numpy as np

LEG_MOTORS = ("L_Motor", "R_Motor")
NSTANCE_BINS = 5

# Below this peak demand the capture is not the running gait. The pronk clips
# the 29.5 N.m ceiling on essentially every stride and the unclipped demand has
# been measured at 80-121 N.m, so a file peaking at single digits is a hold, a
# settle, or a failed trigger. Set well above the ~2 N.m a pre-trigger hold
# produces and well below anything the loaded gait reaches, so the
# classification is not sensitive to where in that gap it sits.
GAIT_MIN_PEAK_NM = 10.0


def load(path):
    rows = defaultdict(list)
    if not os.path.exists(path):
        # The likeliest failure by far, and it has a specific cause worth
        # naming rather than a traceback: the driver only writes this file when
        # CORGI_TORQUE_DEBUG=1 was exported in the shell that started the
        # SIMULATOR. Exporting it in the controller's shell does nothing, and a
        # stale launch holding the robot will produce no file either.
        print(f"\n  {path} does not exist.\n")
        print("  The driver writes it only when CORGI_TORQUE_DEBUG=1 was set in")
        print("  the shell that launched corgi_sim -- not the controller shell.")
        print("  If a stale launch owned the robot, your driver never ran; check")
        print("  for two Corgi_launch.py processes with 'ps -eo pid,etime,cmd'.")
        print("\n  Override the location with CORGI_TORQUE_DEBUG_PATH, or pass")
        print("  the file as the first argument.\n")
        return {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows[(r["leg"], r["motor"])].append((
                float(r["t"]), int(r["in_contact"]),
                float(r["t_stiff"]), float(r["t_damp"]), float(r["t_ff"]),
                float(r["tau_demand"]), float(r["tau_applied"]),
                float(r["pos_error"]), float(r["kp"]), float(r["kd"]),
            ))
    return {k: np.array(v) for k, v in rows.items()}


def stance_progress(down):
    """0 at touchdown -> 1 at liftoff, NaN in flight. Same convention as
    check_torque_phase.py so the two are directly comparable."""
    prog = np.full(len(down), np.nan)
    edges = np.diff(down.astype(int))
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if down[0]:
        starts = [0] + starts
    if down[-1]:
        ends = ends + [len(down)]
    for a, b in zip(starts, ends):
        if b > a:
            prog[a:b] = np.linspace(0.0, 1.0, b - a, endpoint=False)
    return prog


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/corgi_torque_terms.csv"
    data = load(path)
    if not data:
        print(f"no rows in {path} -- was CORGI_TORQUE_DEBUG=1 set for the run?")
        return

    print()
    print("=" * 78)
    print("TORQUE DECOMPOSITION")
    print("=" * 78)
    n = sum(len(v) for v in data.values())
    print(f"  {path}: {n} rows, {len(data)} (leg, motor) series")

    # --- identity check, before any interpretation ----------------------
    # If the three terms do not sum to the demand, the instrumentation is
    # wrong and nothing below means anything.
    worst = 0.0
    for k, v in data.items():
        resid = np.abs(v[:, 2] + v[:, 3] + v[:, 4] - v[:, 5]).max()
        worst = max(worst, resid)
    # Tolerance is set by the CSV's own format, not by the arithmetic. The
    # driver writes %.6f, so each of the four numbers carries up to 5e-7 of
    # rounding and their combination up to ~2e-6 -- an identity that holds
    # exactly in memory cannot survive the round trip any tighter than that.
    # An earlier 1e-6 threshold failed real data at exactly 1.00e-06, which was
    # the checker being wrong about the file, not the file being wrong about
    # the physics. 1e-4 still catches a genuine inconsistency, which would be
    # O(1) N.m -- four orders clear of this floor.
    ok = worst < 1e-4
    print(f"  identity t_stiff+t_damp+t_ff == tau_demand: "
          f"max residual {worst:.2e}  {'PASS' if ok else 'FAIL'} "
          f"(tolerance 1e-4, set by the file's %.6f format)")
    if not ok:
        print("  -> instrumentation is inconsistent; stop here.")
        return

    # --- validity gate ---------------------------------------------------
    # A decomposition of a robot that is not running the gait is not a
    # decomposition of anything. This gate exists because the first real
    # capture was a pre-trigger hold and the analyser cheerfully returned
    # "t_damp 75%, unanimous across all 8 series" from a 2 N.m signal -- a
    # confident verdict about the wrong robot state.
    #
    # Two independent tells, either of which is disqualifying:
    #   t_ff identically zero -> force_control is in its position_control
    #     branch (kp=50, kd=1, torque=0), i.e. no impedance command is being
    #     published at all. During the real gait t_ff is never all-zero.
    #   peak demand far below the saturation regime -> nothing is loaded. The
    #     gait clips the 29.5 N.m ceiling on essentially every stride.
    # --- window to the gait ----------------------------------------------
    # The CSV starts when the DRIVER starts, so it contains the whole
    # pre-trigger hold -- often minutes of it against tens of seconds of gait.
    # Pooling the two makes every mean meaningless: the first real capture
    # showed bin means of 0.8-1.9 N.m against a peak of 170, because the hold
    # outnumbers the gait by an order of magnitude.
    #
    # The boundary is sharp and does not need a heuristic: during the hold
    # force_control sits in position_control and publishes t_ff == 0 exactly.
    # The first non-zero t_ff anywhere is the moment impedance control takes
    # over. Window by TIME from there, globally, so all series stay aligned and
    # a legitimately-zero t_ff mid-gait is not dropped.
    t_start = None
    for _, v in data.items():
        nz = np.flatnonzero(np.abs(v[:, 4]) > 1e-9)
        if nz.size:
            t0 = v[nz[0], 0]
            t_start = t0 if t_start is None else min(t_start, t0)
    if t_start is not None:
        before = sum(len(v) for v in data.values())
        data = {k: v[v[:, 0] >= t_start] for k, v in data.items()}
        data = {k: v for k, v in data.items() if len(v)}
        after = sum(len(v) for v in data.values())
        print(f"  gait window: t >= {t_start:.2f} s "
              f"({after} of {before} rows, {100*after/before:.0f}%) -- "
              f"pre-trigger hold excluded")
    else:
        print("  WARNING: t_ff is zero everywhere; no gait window found")

    leg_series = [(k, v) for k, v in data.items() if k[1] in LEG_MOTORS]
    peak_all = max(np.abs(v[:, 5]).max() for _, v in leg_series)
    ff_all_zero = all(np.abs(v[:, 4]).max() < 1e-9 for _, v in leg_series)
    if ff_all_zero or peak_all < GAIT_MIN_PEAK_NM:
        print()
        print("=" * 78)
        print("  NOT A GAIT RUN -- refusing to interpret this capture")
        print("=" * 78)
        if ff_all_zero:
            print("  t_ff is identically zero on every leg motor, so")
            print("  force_control is in its position_control fallback and no")
            print("  impedance command is reaching it. The robot is holding a")
            print("  pose, not running the template.")
        if peak_all < GAIT_MIN_PEAK_NM:
            print(f"  peak demand {peak_all:.1f} N.m is below the "
                  f"{GAIT_MIN_PEAK_NM:.0f} N.m floor this gait is known to")
            print("  exceed on every stride. Nothing is loaded.")
        print()
        print("  Most likely: the trigger was never published, so the run never")
        print("  left the pre-trigger hold. Start the controller, publish the")
        print("  trigger, let the gait run, THEN re-read the file.")
        print()
        print("  The CSV is appended from the driver's start, so a valid run")
        print("  will contain this quiet period too -- that is fine and")
        print("  expected. What is not fine is a file containing ONLY it.")
        print("=" * 78)
        print()
        return

    # --- how much demand is being refused -------------------------------
    print()
    print("  clipping (leg motors only)")
    print(f"    {'leg':>4} {'peak demand':>12} {'peak applied':>13} "
          f"{'% samples clipped':>18}")
    for leg in "ABCD":
        d = np.concatenate([data[(leg, m)][:, 5] for m in LEG_MOTORS
                            if (leg, m) in data]) if any(
                                (leg, m) in data for m in LEG_MOTORS) else None
        if d is None:
            continue
        a = np.concatenate([data[(leg, m)][:, 6] for m in LEG_MOTORS
                            if (leg, m) in data])
        clipped = 100.0 * np.mean(np.abs(np.abs(d) - np.abs(a)) > 1e-9)
        print(f"    {leg:>4} {np.abs(d).max():12.1f} {np.abs(a).max():13.1f} "
              f"{clipped:17.1f}%")

    # --- the attribution, at the peak ------------------------------------
    # Share is taken at each series' own peak-demand sample rather than as a
    # mean: the question is what makes the SPIKE, and a mean over a stride
    # whose median demand is ~4 N.m would be dominated by the quiet part.
    print()
    print("  composition AT PEAK DEMAND (leg motors; sign kept, so terms may oppose)")
    print(f"    {'leg':>4} {'motor':>8} {'tau':>8} {'t_stiff':>9} {'t_damp':>8} "
          f"{'t_ff':>8} | {'stiff%':>7} {'damp%':>7} {'ff%':>7}")
    shares = []
    for leg in "ABCD":
        for m in LEG_MOTORS:
            if (leg, m) not in data:
                continue
            v = data[(leg, m)]
            i = int(np.argmax(np.abs(v[:, 5])))
            tau, ts, td, tf = v[i, 5], v[i, 2], v[i, 3], v[i, 4]
            denom = abs(ts) + abs(td) + abs(tf)
            if denom < 1e-12:
                continue
            s = (100 * abs(ts) / denom, 100 * abs(td) / denom,
                 100 * abs(tf) / denom)
            shares.append(s)
            print(f"    {leg:>4} {m:>8} {tau:8.1f} {ts:9.1f} {td:8.1f} "
                  f"{tf:8.1f} | {s[0]:6.0f}% {s[1]:6.0f}% {s[2]:6.0f}%")

    # --- by stance progress ---------------------------------------------
    print()
    print("  mean |term| by stance progress (leg motors pooled)")
    print(f"    {'progress':>11} {'t_stiff':>9} {'t_damp':>8} {'t_ff':>8} "
          f"{'demand':>8}")
    pooled = []
    for leg in "ABCD":
        for m in LEG_MOTORS:
            if (leg, m) not in data:
                continue
            v = data[(leg, m)]
            pooled.append((v, stance_progress(v[:, 1] > 0.5)))
    for b in range(NSTANCE_BINS):
        lo, hi = b / NSTANCE_BINS, (b + 1) / NSTANCE_BINS
        acc = [[], [], [], []]
        for v, p in pooled:
            sel = (~np.isnan(p)) & (p >= lo) & (p < hi)
            if not sel.any():
                continue
            acc[0].append(np.abs(v[sel, 2]).mean())
            acc[1].append(np.abs(v[sel, 3]).mean())
            acc[2].append(np.abs(v[sel, 4]).mean())
            acc[3].append(np.abs(v[sel, 5]).mean())
        if not acc[0]:
            continue
        print(f"    {lo:5.2f}-{hi:4.2f} {np.mean(acc[0]):9.1f} "
              f"{np.mean(acc[1]):8.1f} {np.mean(acc[2]):8.1f} "
              f"{np.mean(acc[3]):8.1f}")

    # --- verdict ---------------------------------------------------------
    print()
    print("=" * 78)
    if shares:
        arr = np.array(shares)
        mean_s = arr.mean(axis=0)
        names = ("t_stiff (kp * tracking error)", "t_damp (kd * velocity)",
                 "t_ff (feedforward + coupling)")
        order = np.argsort(mean_s)[::-1]
        print("  SHARE AT PEAK, mean [min-max] across series: " + ",  ".join(
            f"{names[i].split()[0]} {mean_s[i]:.0f}% "
            f"[{arr[:, i].min():.0f}-{arr[:, i].max():.0f}]" for i in order))

        # Do NOT average across series that disagree. The mean of a
        # stiff-dominant leg and an ff-dominant leg is a number describing
        # neither, and it would name a "dominant term" on the difference
        # between two near-equal averages. The left/right split in this robot
        # is documented and structural, so this is a live risk, not a
        # hypothetical one -- it was caught by a self-test in which half the
        # legs were stiff-dominant and half ff-dominant, and the mean declared
        # a winner at 49% vs 49%.
        per_series = arr.argmax(axis=1)
        unanimous = len(set(per_series.tolist())) == 1
        top = order[0]
        margin = mean_s[order[0]] - mean_s[order[1]]

        print()
        if not unanimous:
            counts = {names[i].split()[0]: int((per_series == i).sum())
                      for i in set(per_series.tolist())}
            print("  NO SINGLE DOMINANT TERM -- the series disagree:")
            for k, v in sorted(counts.items(), key=lambda kv: -kv[1]):
                print(f"    {k:<10} dominant in {v} of {len(per_series)} series")
            print()
            print("  Report this split, do NOT collapse it to a mean. Legs that")
            print("  saturate for different reasons are a finding in themselves,")
            print("  and the left/right asymmetry is already documented.")
        elif margin < 15.0:
            print(f"  NO CLEAR WINNER: {names[order[0]].split()[0]} leads "
                  f"{names[order[1]].split()[0]} by only {margin:.0f} points.")
            print("  Treat as unresolved rather than picking the larger number.")
        elif top == 0:
            print(f"  DOMINANT TERM: {names[top]} at {mean_s[top]:.0f}%, "
                  f"unanimous across all {len(per_series)} series")
            print("  The impedance loop is fighting its own tracking error, not")
            print("  carrying the body. The erosion is CONTROLLER-SIDE, which")
            print("  means it is not a law of nature and the trot verdict is")
            print("  built on a fixable number. Next: why is the error large at")
            print("  peak compression -- is kp inflated (the k_lateral / kp_h")
            print("  failure was exactly this), or is the reference unreachable?")
        elif top == 2:
            print(f"  DOMINANT TERM: {names[top]} at {mean_s[top]:.0f}%, "
                  f"unanimous across all {len(per_series)} series")
            print("  The commanded FORCE is the problem, not the tracking. The")
            print("  model's quasi-static prediction is wrong about the force")
            print("  itself, so the template is what has to change.")
        else:
            print(f"  DOMINANT TERM: {names[top]} at {mean_s[top]:.0f}%, "
                  f"unanimous across all {len(per_series)} series")
            print("  Damping is doing work it should not. b_radial was already")
            print("  zeroed deliberately; look at b_tangential / b_lateral.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
