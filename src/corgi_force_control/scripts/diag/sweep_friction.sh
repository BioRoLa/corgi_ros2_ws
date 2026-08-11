#!/usr/bin/env bash
# Is the slip friction-limited, or is it forced by the gait's geometry?
#
# Measured: 8% slip running straight, 32% while turning, and the run-to-run slip
# tracks the turn-radius scatter that blocks the Stage 4 comparison
# (corr(no-slip, R_rate) = -0.76, n = 4).
#
# Two candidate causes, with opposite consequences:
#
#   FRICTION-LIMITED   the foot slides because the surface cannot hold it.
#                      More grip fixes it.
#   GEOMETRICALLY FORCED  the foot has no steering freedom, so on a curved path
#                      ~8 deg of its velocity is perpendicular to the only
#                      direction it can roll (examples/gslip/turn_scrub_geometry.py
#                      in LegWheel). More grip cannot fix that -- the foot binds
#                      instead of sliding.
#
# A WIDE bracket, not a plausible value. The real tread is cut from a bicycle
# tyre, so mu is probably near 0.7-1.2 and the Webots default of 1.0 is already
# about right; asking "does 1.5 help" would confound a small effect with noise.
# Asking whether anything moves between 0.6 and 1.6 is decisive either way.
#
# PREDICTION, recorded before the runs so it can be wrong:
#   straight-line no-slip % improves with mu;
#   turning no-slip % improves much less;
#   radius scatter narrows.
# If turning improves as much as straight, the forced-scrub model is wrong.
#
#   bash sweep_friction.sh
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=/home/alexc/corgi_ws/corgi_ros2_ws
TPL="$WS/src/corgi_force_control/config/gslip_pronk_template.csv"
OUT=/home/alexc/corgi_ws/runs/2026-08-09_friction
mkdir -p "$OUT"

GAINS="steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094"

for MU in 0.6 1.6; do
    python3 "$HERE/set_friction.py" "$MU"
    ( cd "$WS" && source /opt/ros/humble/setup.bash \
        && colcon build --packages-select corgi_sim 2>&1 | tail -1 )

    for MODE in straight turn; do
        RATE=0.0
        [ "$MODE" = turn ] && RATE=0.288
        for N in 1 2 3; do
            TAG="mu${MU}_${MODE}_$N"
            echo "=== $TAG (mu=$MU, turn_rate=$RATE)"
            RAMP_UNTIL=10 RAMP_DUMP="$OUT/$TAG.npz" \
                bash "$HERE/ramp_cycle.sh" "$TPL" \
                    $GAINS turn_rate:="$RATE" 2>&1 | tail -3
            cp /tmp/ramp_ctl.log "$OUT/$TAG.ctl.log" 2>/dev/null
            echo
        done
    done
done

# Leave the world at the Webots default, so an unrelated run later is not
# silently sitting on the last sweep point.
python3 "$HERE/set_friction.py" 1.0
( cd "$WS" && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select corgi_sim 2>&1 | tail -1 )
echo "FRICTION SWEEP DONE -- world restored to mu = 1.0"
