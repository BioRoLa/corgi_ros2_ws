#!/usr/bin/env bash
# Stage 1 contact-model sweep: the lambda x alpha grid, as runs.
#
# The gate (Thesis Timeline, Stage 1): does the contact height follow
# h = R_tread cos(lambda) - w_flat sin(lambda) + r_corner?  In wheeled mode
# the contact azimuth alpha is swept FOR FREE: rolling carries the contact
# around the whole rim once per revolution, and a 20 s roll is ~6 revolutions
# -- this is what satisfies the day-one rule "push alpha well past 40 deg"
# (the running stance never leaves the foot rim; rolling exercises every rim).
# So the commanded grid is over lambda; alpha lives in the z(t) ripple at rim
# crossings, extracted by the analysis side.
#
# The lambda = 0 run is not a formality: it re-baselines the {A,C}/{B,D}
# zero-command gamma residual AT THIS kp (the residual is elastic -- a
# constant ~2.8 N.m moment deflected by whatever stiffness the loop has, log
# section 62 -- so a kp-90 reference is wrong at kp 500 by 1.4 deg).
#
# Usage:
#   stage1_lambda_alpha_sweep.sh <kp_h> [lam_deg ...]
#   stage1_lambda_alpha_sweep.sh --dry-run <kp_h> [lam_deg ...]
#
#   Default grid: 0 5 10 15 20 25 30 35 40.  Dumps land as
#   ~/camber_dumps/camber_s1_lam<L>_kp<K>_pl.npz -- tags, never paths (path
#   args are mangled through wsl.exe; see camber_cycle.sh).
#
#   --dry-run validates every commanded trajectory offline via
#   camber_roll.py --dry-run (no ROS, no simulator) and prints the run plan.
#   Run it before burning sim time; it is also how this script was validated
#   before Stage 1 opened (the section 48 pattern).
#
# Analysis side (LegWheel, Windows):
#   uv run python examples/gslip/stage1_contact_sweep_analysis.py \
#       <dump>:<lam>:<alpha> ...
#
# No `set -u`: camber_cycle sources ROS setup files.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DRY=0
if [ "$1" = "--dry-run" ]; then
    DRY=1
    shift
fi
KP="${1:?usage: stage1_lambda_alpha_sweep.sh [--dry-run] <kp_h> [lam_deg ...]}"
shift
if [ $# -gt 0 ]; then
    LAMS="$@"
else
    LAMS="0 5 10 15 20 25 30 35 40"
fi

if [ "$DRY" = "1" ]; then
    echo "DRY RUN: kp_h $KP, grid: $LAMS (no ROS, no simulator)"
    fails=0
    for lam in $LAMS; do
        echo "--- lam $lam ---"
        python3 "$HERE/camber_roll.py" --lam-deg "$lam" --pattern lr \
            --kp-h "$KP" --dry-run || fails=$((fails + 1))
    done
    echo
    echo "run plan (tags):"
    for lam in $LAMS; do
        echo "  camber_cycle.sh $lam lr s1_lam${lam}_kp${KP}_pl"
    done
    [ "$fails" = "0" ] && echo "DRY RUN CLEAN" || echo "DRY RUN: $fails FAILURES"
    exit "$fails"
fi

# --- shared-sim guard: never start on top of someone else's run ------------
FOREIGN=$(pgrep -af 'Corgi_launc[h]|ros2 launc[h]|camber_cycl[e].sh|ramp_cycl[e].sh|sim_cycl[e].sh|webots\.ex[e]' \
          | grep -v stage1_lambda)
if [ -n "$FOREIGN" ]; then
    echo "ABORT: sim not free --"
    echo "$FOREIGN"
    exit 2
fi

# --- the sweep -------------------------------------------------------------
# Each cycle is a fresh simulator and now verifies its own teardown (exit 3 =
# dump good, sim dirty). A dirty sim stops the sweep rather than contaminating
# the next dump.
export CAMBER_EXTRA="--kp-h $KP"
done_runs=""
for lam in $LAMS; do
    tag="s1_lam${lam}_kp${KP}_pl"
    echo "=== lam $lam kp $KP start $(date +%T) ==="
    bash "$HERE/camber_cycle.sh" "$lam" lr "$tag"
    rc=$?
    if [ "$rc" != "0" ]; then
        echo "SWEEP HALTED at lam $lam (rc=$rc). Completed: $done_runs"
        exit "$rc"
    fi
    # Cheap in-line dump gate (log section 39): a contaminated clock reads as
    # plausible data; refuse to continue a sweep that is recording garbage.
    python3 - "$HOME/camber_dumps/camber_${tag}.npz" <<'PYGATE'
import sys
import numpy as np
d = np.load(sys.argv[1])
bad = False
for key in ("motor_t", "odom_t"):
    t = d[key]
    dup = 1.0 - len(np.unique(t)) / len(t)
    back = int(np.sum(np.diff(t) < 0.0))
    print(f"  {key}: dup {dup:.2%}, backward {back}")
    bad |= dup > 0.01 or back > 0
sys.exit(1 if bad else 0)
PYGATE
    if [ $? -ne 0 ]; then
        echo "SWEEP HALTED at lam $lam: DUMP FAILED THE TIMESTAMP GATE."
        echo "Completed: $done_runs"
        exit 4
    fi
    done_runs="$done_runs $lam"
    echo "=== lam $lam done $(date +%T) ==="
done

echo "SWEEP COMPLETE: kp $KP, lams:$done_runs"
ls -la "$HOME"/camber_dumps/camber_s1_*_kp"${KP}"_pl.npz
