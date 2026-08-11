#!/usr/bin/env bash
# Is the ~40% speed shortfall torque-limited?
#
# Section 27 separated two problems: the template mismatch explains the
# gait-quality failures and NONE of the speed shortfall. Every template reaches
# only 34-61% of its own design speed. This tests the most obvious remaining
# suspect -- the 35 N.m clamp, which the leg motors touch on 95-100% of strides
# even on the well-behaved v~0.70 template.
#
# Raising CORGI_MAX_TORQUE does NOT model a stronger motor, and the result must
# not be read as "the robot would be faster with better hardware". It makes the
# CLIPPED demand observable, which is the actual question: is the controller
# asking for more than 35 N.m and being refused, or is it asking for little and
# the speed is lost elsewhere?
#
# Prediction, registered before running:
#   speed recovers with the ceiling  -> torque-limited; the clamp is the cause,
#                                       and the real robot needs a gentler
#                                       template or stronger motors
#   speed unchanged                  -> the brief clipping costs nothing, the
#                                       loss is elsewhere (damping, slip,
#                                       desync) and this rules out a whole
#                                       branch
#
# v~0.70 throughout, because that is the template the robot actually executes
# and the one Stage 2a/3 will build on.
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=/home/alexc/corgi_ws/corgi_ros2_ws
TPL="$WS/src/corgi_force_control/config/gslip_pronk_template_v070.csv"
OUT=/home/alexc/corgi_ws/runs/2026-08-11_torque_ceiling
mkdir -p "$OUT"

GAINS="steer_offset:=0.04363 k_steer_yaw:=-0.8 steer_limit:=0.2094 turn_rate:=0.0"

# The INSTALLED driver is what runs, not src. The first attempt at this sweep
# was silently invalid because corgi_sim had never been rebuilt after the env
# hook was added: install/ still had `self.Max_Torque = 35.0` hardcoded, so all
# nine runs used 35 N.m and the honest-looking conclusion would have been
# "raising the ceiling changes nothing, therefore not torque-limited". Check
# before spending 35 minutes.
DRV=$WS/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg/corgi_driver.py
if ! grep -q "CORGI_MAX_TORQUE" "$DRV"; then
    echo "ABORT: the installed driver has no CORGI_MAX_TORQUE hook."
    echo "  $DRV"
    echo "  Run: colcon build --packages-select corgi_sim"
    exit 1
fi

for TQ in 35 70 200; do
    export CORGI_MAX_TORQUE="$TQ"
    for N in 1 2 3; do
        TAG="tq${TQ}_$N"
        echo "=== $TAG   CORGI_MAX_TORQUE=$CORGI_MAX_TORQUE"
        RAMP_UNTIL=10 RAMP_DUMP="$OUT/$TAG.npz" \
            bash "$HERE/ramp_cycle.sh" "$TPL" $GAINS 2>&1 | tail -3
        cp /tmp/ramp_ctl.log "$OUT/$TAG.ctl.log" 2>/dev/null
        echo
    done
done
unset CORGI_MAX_TORQUE
echo "TORQUE CEILING SWEEP DONE -- driver default (35) restored by unset"
