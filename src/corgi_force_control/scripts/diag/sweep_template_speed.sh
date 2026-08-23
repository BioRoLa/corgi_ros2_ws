#!/usr/bin/env bash
# Does commanding a template the robot can actually hold help?
#
# The shipped template is the v~1.20 fixed point: 1.516 m/s mean forward,
# alpha* 17.79 deg, |beta| +-18 deg. The robot reaches 0.70-0.85 m/s, which by
# MEAN FORWARD SPEED is the v~0.70 fixed point -- alpha* 40.74 deg, |beta|
# +-9.5 deg, a substantially different gait.
#
# Two things to be careful about, both of which have already caused a wrong
# answer today:
#
#   v~ indexes TOUCHDOWN SPEED MAGNITUDE, not forward speed. v~0.45 sounds like
#   a slow template but travels at 0.418 m/s -- SLOWER than the robot already
#   manages. v~0.70 is the one that matches the operating point. Both are run,
#   because v~0.45 was asked for and it brackets the comparison usefully.
#
#   The exporter's default beta search window is 70-80 deg, which does NOT
#   contain the fixed point at low speed (beta* rises to 80.75 at v~0.70 and
#   83.75 at v~0.45). These templates were exported with a widened window.
#
# Prediction, registered before running: if the template/speed mismatch is
# load-bearing, v~0.70 should raise flight toward its design 59.5% and reduce
# torque saturation from the current 100%-of-strides. If the robot is slow for
# other reasons -- desync, slip, tracking -- all three will look alike.
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=/home/alexc/corgi_ws/corgi_ros2_ws
CFG="$WS/src/corgi_force_control/config"
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
OUT=/home/alexc/corgi_ws/runs/2026-08-11_template_speed
mkdir -p "$OUT"

GAINS="steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094 turn_rate:=0.0"

run () {   # $1 = tag, $2 = template file
    for N in 1 2 3; do
        TAG="$1_$N"
        echo "=== $TAG   ($2)"
        RAMP_UNTIL=10 RAMP_DUMP="$OUT/$TAG.npz" \
            bash "$HERE/ramp_cycle.sh" "$CFG/$2" $GAINS 2>&1 | tail -3
        cp /tmp/ramp_ctl.log "$OUT/$TAG.ctl.log" 2>/dev/null
        echo
    done
}

run v120 gslip_pronk_template.csv
run v070 gslip_pronk_template_v070.csv
run v045 gslip_pronk_template_v045.csv

echo "TEMPLATE SPEED SWEEP DONE"
