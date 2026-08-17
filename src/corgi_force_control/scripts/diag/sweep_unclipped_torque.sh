#!/usr/bin/env bash
# What does the leg ACTUALLY demand? -- the clamp as an instrument.
#
# Every stride peak recorded so far is clipped at 35 N.m, so the erosion factor
# behind the trot verdict (>=2.33x) is a LOWER BOUND, not a measurement. This
# raises the clamp to 100 N.m to make the peak observable.
#
# This does NOT model a stronger motor and is NOT a proposed fix. 35 N.m stays
# the real limit. A leg that no longer clips also tracks the template better, so
# these are different runs, not the same runs seen more clearly -- which is the
# point: it gives the demand of a leg that actually follows the commanded
# spring, which is the number the trot feasibility calc needs.
#
# Both templates are run because they answer different questions:
#   v120 - the shipped template, the one 2.33x was computed against
#   v070 - the template matched to the robot's operating point, which fixed
#          flight (41->57%) and saturation DUTY (4.2->1.8%) but left 95% of
#          strides still touching the clamp
#
# Prediction, registered before running: if the peak is a brief impact spike,
# the un-clipped peak lands well above 35 and the trot verdict hardens further.
# If the clamp itself was causing the over-compression, demand falls below 35
# and erosion collapses -- the only outcome that puts a trot back on the table
# (it needs erosion < 1.38x at v~0.50).
set -o pipefail

WS=/home/alexc/corgi_ws/corgi_ros2_ws
HERE="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
OUT=/home/alexc/corgi_ws/runs/2026-08-12_unclipped_torque
mkdir -p "$OUT"

# Same gains as the template-speed sweep, so the two campaigns are comparable.
GAINS="steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094 turn_rate:=0.0"

export CORGI_MAX_TORQUE=100.0

run () {   # $1 = tag, $2 = template file
    for N in 1 2 3; do
        TAG="$1_$N"
        echo "=== $TAG   ($2)   clamp=$CORGI_MAX_TORQUE"
        RAMP_UNTIL=10 RAMP_DUMP="$OUT/$TAG.npz" \
            bash "$HERE/ramp_cycle.sh" "$CFG/$2" $GAINS 2>&1 | tail -3
        cp /tmp/ramp_ctl.log "$OUT/$TAG.ctl.log" 2>/dev/null
        echo
    done
}

run v120 gslip_pronk_template.csv
run v070 gslip_pronk_template_v070.csv

echo "UNCLIPPED TORQUE SWEEP DONE"
