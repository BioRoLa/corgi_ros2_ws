#!/usr/bin/env bash
# Does using the spring's REST length as the radial reference fix the
# over-compression?
#
# gslip_pronk.cpp argues the radial reference should be the constant rest
# length l0, not the template's compressed l(t): in SLIP-RF the compression is
# an OUTPUT of the dynamics, so feeding l(t) to an impedance controller as its
# rest position double-counts it. Measured symptom, recorded in that comment:
# "commanded theta bottomed at 84.4 deg while the leg reached 71-75 deg".
#
# It was tried once on the PRONK, flipped the robot, and was reverted -- but the
# failure was attributed to the robot not being at its fixed point, not to the
# reasoning being wrong.
#
# So test it on the HOP instead, where the robot IS near its fixed point and the
# gait already works at ~58% flight. That isolates the reference question from
# the standing-start problem that confounded the first attempt.
#
#   hop survives + stance torque drops -> reasoning validated, carry to pronk
#   hop degrades                       -> idea is dead, stop pursuing it
#
# Repeats each setting: the flight noise band is wide (11-42% on the pronk), so
# a single run per setting decides nothing.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TPL=~/corgi_ws/corgi_ros2_ws/install/corgi_force_control/share/corgi_force_control/config/gslip_hop_template.csv
REPS="${REPS:-2}"

for mode in false true; do
    for rep in $(seq 1 "$REPS"); do
        echo "=== spring_rest_reference=$mode  rep $rep ==="
        bash "$HERE/sim_cycle.sh" "$TPL" spring_rest_reference:="$mode" \
            2>/dev/null | grep -E "airborne|^   [ABCD] |roll |pitch|FAILED"
        echo
    done
done
echo "hop spring-rest test done"
