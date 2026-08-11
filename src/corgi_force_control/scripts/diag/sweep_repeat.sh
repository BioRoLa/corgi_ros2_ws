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
