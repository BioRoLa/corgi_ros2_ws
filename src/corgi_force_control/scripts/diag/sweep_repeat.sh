#!/usr/bin/env bash
# Three runs at one operating point. Single runs decide nothing.
#
# The point is turn_rate = +0.288 rad/s, the only command in the campaign that
# was actually delivered (92%) while the body still went where it pointed
# (R_fit/R_rate = 1.03, cross-track 25 mm). +0.360 was delivered at 28% and
# -0.360 held its rate but destroyed forward speed, so neither is worth
# repeating until this one is established.
#
# RAMP_UNTIL=10, not 20: the usable turn is the 4-8 s window. After about 8 s
# every run so far degenerates into a pirouette -- yaw rate maintained, forward
# speed gone -- and recording it three more times establishes nothing that the
# 20 s runs have not already shown.
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

for TAG in a b c; do
    echo "=== repeat $TAG -> /tmp/rep_$TAG.npz"
    RAMP_UNTIL=10 RAMP_DUMP="/tmp/rep_$TAG.npz" \
        bash "$HERE/ramp_cycle.sh" "$TPL" \
            steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094 \
            turn_rate:=0.288 2>&1 | tail -4
    cp /tmp/ramp_ctl.log "/tmp/rep_$TAG.ctl.log" 2>/dev/null
    echo
done
echo "REPEAT DONE"
