#!/usr/bin/env bash
# Re-run S15's gamma yaw-authority test under the CORRECTED ABAD ceiling.
#
# S15 (2026-08-08) concluded gamma is INERT in yaw: k_yaw 0.15 -> 0 collapses
# gamma from +-5 deg pinned to +-0.3 deg and leaves the heading unchanged. That
# was measured with a shared 35 N.m clamp on every joint. The ABAD's real
# ceiling is 44.25 N.m (HT-04 stall at 9:1), and 30.5% of ABAD samples in a
# post-fix run exceed 35 -- so for roughly a third of every S15 run the ABAD was
# being throttled by a limit it does not have.
#
# Three arms, n = 3, full ramp pass:
#   A  k_yaw 0.15, ABAD 44.25   correction ON at true authority
#   B  k_yaw 0.00, ABAD 44.25   the S15 comparison, redone
#   C  k_yaw 0.15, ABAD 35.00   isolates the ceiling change against A
#
# A vs B re-asks S15's question. A vs C answers "did the ceiling matter at all".
#
# PREDICTION, registered before running:
#   S15's mechanism is GEOMETRIC -- gamma cambers the leg but does not yaw it,
#   there is no steering freedom at the foot (see the Phase 5 note). A higher
#   ceiling buys authority to ACHIEVE commanded camber, not yaw per degree of
#   camber. So expect A ~= B in heading (S15 survives) and A ~= C in heading,
#   with the ceiling showing up only in ABAD saturation % and gamma tracking.
#
#   What would OVERTURN S15: A materially different from B in heading under the
#   new ceiling, i.e. the correction does something once it is not clipped.
#
#   Note gamma already REACHED its +-5 deg command clamp in S15 (-5.07/+5.15),
#   so the torque clip was not preventing the commanded angle. That is the main
#   reason to expect S15 to survive.
#
# -----------------------------------------------------------------------------
# RAMP_UNTIL: why 18 and not 10, and not unset.
#
# check_ramp stops when contact time exceeds RAMP_UNTIL measured from ~0, but
# the template does not START until the standup + settle finishes, which is the
# reported `anchor` -- about 5.5-6.0 s. So:
#
#     template seconds captured  =  RAMP_UNTIL - anchor
#
# Confirmed on 17 runs across two failed attempts: RAMP_UNTIL=10 with anchor
# 5.68 gave 4.32 s; leaving it UNSET makes stop_at default to the template
# duration 9.5738, which with anchor 5.96 gave 3.62 s -- i.e. unsetting it is
# WORSE, because 9.5738 < 10.
#
# The ramp needs the full 9.5738 s and anchor has ranged 5.45-6.02, so
# RAMP_UNTIL must exceed ~15.6. 18 leaves real margin for a slow standup.
# -----------------------------------------------------------------------------
set -o pipefail

WS=/home/alexc/corgi_ws/corgi_ros2_ws
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S202. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
HERE="$WS/src/corgi_force_control/scripts/diag"
TPL="$WS/src/corgi_force_control/config/gslip_speed_ramp_template.csv"
OUT=/home/alexc/corgi_ws/runs/2026-08-12_gamma_yaw_retest
mkdir -p "$OUT"

RAMP_SECONDS=9.5738

DRV="$WS/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg/corgi_driver.py"
if ! grep -q "MAX_TORQUE_ABAD" "$DRV"; then
    echo "ABORT: installed driver has no per-joint ceilings. colcon build --packages-select corgi_sim"
    exit 1
fi

# Coverage is the thing two previous attempts got silently wrong, so check it
# per run rather than discovering it 40 minutes later.
coverage () {   # $1 = npz
    python3 - "$1" "$RAMP_SECONDS" <<'PY'
import sys
import numpy as np
try:
    d = np.load(sys.argv[1], allow_pickle=True)
except Exception as e:
    print(f"    COVERAGE: no dump ({e.__class__.__name__})")
    sys.exit(0)
need = float(sys.argv[2])
a = float(d["anchor"])
cov = float(d["imu_t"].max()) - a
print(f"    COVERAGE: anchor {a:.2f}, {cov:.2f} s of {need:.2f} s "
      f"-- {'OK' if cov >= need else 'TRUNCATED'}")
PY
}

# S15 used the bare ramp with no steering channel -- keep it that way, or the
# differential-beta steering added later would confound the yaw measurement.
run () {   # $1 = tag, $2 = k_yaw, $3 = ABAD ceiling
    for N in 1 2 3; do
        TAG="$1_$N"
        echo "=== $TAG   k_yaw=$2  ABAD=$3"
        CORGI_MAX_TORQUE_ABAD="$3" \
        RAMP_UNTIL=18 RAMP_DUMP="$OUT/$TAG.npz" \
            bash "$HERE/ramp_cycle.sh" "$TPL" k_yaw:="$2" 2>&1 | tail -3
        cp /tmp/ramp_ctl.log "$OUT/$TAG.ctl.log" 2>/dev/null
        coverage "$OUT/$TAG.npz"
        echo
    done
}

run kyaw015_abad44 0.15 44.25
run kyaw000_abad44 0.0  44.25
run kyaw015_abad35 0.15 35.0

echo "GAMMA YAW RETEST DONE"
