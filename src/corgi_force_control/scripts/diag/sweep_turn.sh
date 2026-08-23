#!/usr/bin/env bash
# Commanded turns, CLOSED loop, on the constant-speed template.
#
# Deliberately not gated on an open-loop authority number. That sweep was run
# first, as the handover asks, and it does not yield one: at running speed the
# parasitic yaw (-2.2 deg/stride) varies run to run by as much as the command is
# worth, the measured left/right differential only reaches 0.77-0.87 of command
# for negative offsets and 0.0-0.2 for positive ones, and every steered run lost
# 30-70% of forward speed. n=1 per point cannot separate signal from that.
#
# The closed loop does not need the number. turn_rate is fixed by the radius
# wanted, R = v/psi_dot, and whether the channel can deliver it is exactly what
# check_turn.py's "delivered % of the commanded yaw rate" measures. So this asks
# the question directly.
#
# Gains are the ones section 19 established, unchanged:
#   steer_offset 0.04363 (2.5 deg trim)  k_steer_yaw -0.8  steer_limit 0.2094
#   d_steer_yaw 0 -- no damping was needed and none is added without an
#   oscillation to justify it.
#
# RAMP_UNTIL=20 leaves ~16.5 s of running after the ~3.5 s the gait needs to get
# airborne. At 0.36 rad/s that is a full circle, which the Kasa fit needs: a fit
# to much less than a quarter turn is weakly determined however clean it looks.
#
#   bash sweep_turn.sh [RAMP_UNTIL]
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
UNTIL="${1:-20}"

# turn_rate rad/s, tag. v ~ 0.72 m/s measured on the baseline, so
# R = 2.0 m -> 0.360, R = 2.5 m -> 0.288.
#
# hold  is the control: same gains, no turn. If the heading does not hold on
#       this template the radii mean nothing, and section 19's hold was measured
#       on the speed ramp, not here.
# r2n   is the mirror of r2. The open-loop sweep found the channel behaves very
#       differently for positive and negative commands, so a left turn is not
#       evidence about a right one.
RUNS=(
  "0.0    hold"
  "0.360  r2"
  "0.288  r25"
  "-0.360 r2n"
)

for r in "${RUNS[@]}"; do
    set -- $r
    RATE="$1"; TAG="$2"
    echo "=== turn_rate $RATE rad/s -> /tmp/turn_$TAG.npz"
    RAMP_UNTIL="$UNTIL" RAMP_DUMP="/tmp/turn_$TAG.npz" \
        bash "$HERE/ramp_cycle.sh" "$TPL" \
            steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094 \
            turn_rate:="$RATE" 2>&1 | tail -5
    cp /tmp/ramp_ctl.log "/tmp/turn_$TAG.ctl.log" 2>/dev/null
    echo
done
echo "TURN SWEEP DONE"
