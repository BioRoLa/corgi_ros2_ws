#!/usr/bin/env bash
# Open-loop steering-authority sweep at RUNNING speed.
#
# The authority figure in the implementation log (0.8 deg/stride per degree of
# command) was measured on the in-place HOP, where beta is identically zero.
# The forward rungs demonstrably tolerate far larger commands than the hop does
# -- the full ramp ran with steer_limit at 12 deg and kept flight near design --
# so the hop number cannot be used to size a turn. This measures it again on a
# leg that is already sweeping.
#
# Open loop on purpose: k_steer_yaw stays 0, so what is measured is the channel,
# not the loop. The parasitic yaw (~-2.2 deg/stride, measured at steer_offset 0)
# is a constant offset on every run, so it cancels in the SLOPE. That is why the
# sweep is symmetric about zero rather than one-sided.
#
# Each run is a fresh simulator. About 7 minutes apiece.
#
#   bash sweep_steer.sh [RAMP_UNTIL]
#
# Dumps land in /tmp/auth_<tag>.npz with the controller log beside them as
# /tmp/auth_<tag>.ctl.log -- check_turn.py reads the commanded turn rate out of
# that, and a sweep is only as good as the record of which value made which
# dump.
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
TPL="$WS/src/corgi_force_control/config/gslip_pronk_template.csv"
UNTIL="${1:-10}"

# steer_offset in radians, tag. steer_limit is held at 12 deg throughout so the
# clamp never silently caps a sweep point -- the +-8 deg ends would hit an 8 deg
# limit exactly and flat-top the curve at the two points that set the slope.
RUNS=(
  "-0.13963 m8"
  "-0.08727 m5"
  "0.04363  p25"
  "0.08727  p5"
  "0.13963  p8"
)

for r in "${RUNS[@]}"; do
    set -- $r
    OFF="$1"; TAG="$2"
    echo "=== steer_offset $OFF rad ($(python3 -c "print(f'{$OFF*57.2957795:+.2f}')") deg) -> /tmp/auth_$TAG.npz"
    RAMP_UNTIL="$UNTIL" RAMP_DUMP="/tmp/auth_$TAG.npz" \
        bash "$HERE/ramp_cycle.sh" "$TPL" \
            steer_offset:="$OFF" steer_limit:=0.2094 \
            k_steer_yaw:=0.0 turn_rate:=0.0 2>&1 | tail -6
    cp /tmp/ramp_ctl.log "/tmp/auth_$TAG.ctl.log" 2>/dev/null
    echo
done
echo "SWEEP DONE"
