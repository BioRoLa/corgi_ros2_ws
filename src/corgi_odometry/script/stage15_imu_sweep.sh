#!/usr/bin/env bash
# Stage 1.5 IMU re-record subset: lambda in {0, 20, 30} x N reps, kp 500.
#
# Purpose: the banked Stage 1 corpus has no IMU stream, so the full ESEKF
# cannot replay on it. This records the minimal subset for the replay
# verdicts (corrected-config drift at lambda 20/30 vs sagittal; lambda 0
# no-regression), through stage15_imu_cycle.sh (fresh sim per run,
# record_camber_imu.py).
#
# DELIBERATE: theta stays at the DEFAULT command (NO --theta-wheel 18.85).
# The r0 = 0.14482 and per-leg eccentricity constants in
# tuned_offline_camber.yaml were fitted on the banked corpus at the default
# theta (achieved ~16.2 deg, uncalibrated closure, e ~1.2 mm); replay runs
# must match the calibration's closure or the ecc term corrects the wrong
# wheel. (log section 76 / handover deviation, stated in the announcement.)
#
# Usage:  stage15_imu_sweep.sh <kp_h> <n_reps> [lam_deg ...]
#         default grid: 0 20 30
# Dumps:  ~/camber_dumps/camber_s15imu_lam<L>_kp<K>_r<N>_pl.npz
#
# No `set -u`: the cycle sources ROS setup files.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KP="${1:?usage: stage15_imu_sweep.sh <kp_h> <n_reps> [lam_deg ...]}"
NREP="${2:?usage: stage15_imu_sweep.sh <kp_h> <n_reps> [lam_deg ...]}"
shift 2
if [ $# -gt 0 ]; then LAMS="$@"; else LAMS="0 20 30"; fi

# settled pre-lean dwell; analysis windows assume it (stage1 convention)
FOLD_SETTLE=3
export FOLD_SETTLE

# --- shared-sim guard: never start on top of someone else's run ------------
FOREIGN=$(pgrep -af 'Corgi_launc[h]|ros2 launc[h]|camber_cycl[e].sh|ramp_cycl[e].sh|sim_cycl[e].sh|webots\.ex[e]' \
          | grep -v stage15_imu)
if [ -n "$FOREIGN" ]; then
    echo "ABORT: sim not free --"
    echo "$FOREIGN"
    exit 2
fi

export CAMBER_EXTRA="--kp-h $KP"
done_runs=""
for rep in $(seq 1 "$NREP"); do
  for lam in $LAMS; do
    tag="s15imu_lam${lam}_kp${KP}_r${rep}_pl"
    echo "=== lam $lam kp $KP rep $rep start $(date +%T) ==="
    bash "$HERE/stage15_imu_cycle.sh" "$lam" lr "$tag"
    rc=$?
    if [ "$rc" != "0" ]; then
        echo "SWEEP HALTED at lam $lam rep $rep (rc=$rc). Completed: $done_runs"
        exit "$rc"
    fi
    # dump gate: timestamps (log section 39) + the IMU streams this sweep
    # exists to record (presence, rate, accel ~ g, quat norm ~ 1)
    python3 - "$HOME/camber_dumps/camber_${tag}.npz" <<'PYGATE'
import sys
import numpy as np
d = np.load(sys.argv[1])
bad = False
for key in ("motor_t", "odom_t", "imu_t"):
    if key not in d:
        print(f"  MISSING {key}")
        bad = True
        continue
    t = d[key]
    dup = 1.0 - len(np.unique(t)) / len(t)
    back = int(np.sum(np.diff(t) < 0.0))
    print(f"  {key}: n={len(t)}, dup {dup:.2%}, backward {back}")
    bad |= dup > 0.01 or back > 0
if not bad:
    rate = len(d["imu_t"]) / (d["imu_t"].max() - d["imu_t"].min())
    a = float(np.linalg.norm(d["imu_accel"], axis=1).mean())
    qn = float(np.linalg.norm(d["imu_quat"], axis=1).mean())
    print(f"  imu: rate {rate:.0f} Hz, |accel| {a:.2f} m/s^2, quat norm {qn:.3f}")
    if not (8.0 < a < 12.0):
        print("  IMU GATE: |accel| not ~g -- device cross-wiring suspect")
        bad = True
    if abs(qn - 1.0) > 0.05:
        print("  IMU GATE: quat norm off -- orientation field suspect")
        bad = True
    if "odom_ang" not in d:
        print("  MISSING odom_ang")
        bad = True
sys.exit(1 if bad else 0)
PYGATE
    if [ $? -ne 0 ]; then
        echo "SWEEP HALTED at lam $lam rep $rep: DUMP FAILED THE GATE."
        echo "Completed: $done_runs"
        exit 4
    fi
    done_runs="$done_runs ${lam}r${rep}"
    echo "=== lam $lam rep $rep done $(date +%T) ==="
  done
done

echo "SWEEP COMPLETE: kp $KP, runs:$done_runs"
ls -la "$HOME"/camber_dumps/camber_s15imu_*_kp"${KP}"_*_pl.npz
