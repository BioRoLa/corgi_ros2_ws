#!/bin/bash
# Does the stance kp actually scale with the commanded k_radial?
#
# WHY. The torque decomposition measured kp_stance/kp_flight at 1.22-1.39 on all
# eight leg-motor series. That ratio is a weighted average of {k_radial,
# k_lateral, k_tangential} over k_flight, so it CANNOT exceed 8941/12000 =
# 0.745 for any Jacobian, pose or gamma. The measured stance gain is therefore
# not producible from the commanded stiffnesses, and t_stiff -- which is
# kp * tracking error -- is the dominant term in the torque erosion.
#
# THE TEST. Hold the stance spring statically (hold_stance:=true, no trigger)
# and vary only k_radial. If kp is affine in k_radial with a sensible slope the
# leg_frame branch is fine and the fault is elsewhere. If kp barely moves, the
# branch is not applying the commanded radial stiffness.
#
# WHY NO TRIGGER. Without it SUPPORT_BOX stays, so the robot is statically
# supported between arms and cannot collapse while no controller is publishing
# -- force_control drives theta -> 0 when nothing commands it. It also makes the
# pose repeatable across arms, which is the whole point: kp = J^T K J depends on
# pose, so the pose must be held constant while K varies.
#
# ONE SIM, MANY CONTROLLERS. The driver owns the CSV and truncates it once on
# first write, so a single simulator instance gives one continuous file. Arms
# are separated by SIM time, bracketed from the file itself rather than from
# wall clock.
set -e

WS=~/corgi_ws/corgi_ros2_ws
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
CSV=/tmp/corgi_torque_terms.csv
OUT=${OUT:-/tmp/k_radial_sweep_brackets.txt}
DWELL=${DWELL:-14}        # seconds per arm, wall clock
ARMS=${ARMS:-"2235.0 4470.0 8941.0 17882.0"}

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

last_t() { tail -1 "$CSV" 2>/dev/null | cut -d, -f1; }

: > "$OUT"
echo "k_radial sweep, hold_stance:=true, dwell ${DWELL}s per arm"
echo "arms: $ARMS"
echo

for K in $ARMS; do
  T0=$(last_t)
  echo "--- k_radial=$K   (csv t before = ${T0:-none}) ---"
  ros2 launch corgi_force_control gslip_pronk.launch.py \
       hold_stance:=true k_radial:="$K" > "/tmp/hold_${K}.log" 2>&1 &
  LP=$!
  sleep "$DWELL"
  T1=$(last_t)
  # Kill the launch and its children. Both nodes die together, which matters:
  # force_control alone with no impedance command folds the legs past 17 deg.
  kill -INT $LP 2>/dev/null || true
  sleep 3
  pkill -f '[g]slip_pronk_node' 2>/dev/null || true
  pkill -f '[f]orce_control_node' 2>/dev/null || true
  sleep 2
  echo "$K $T0 $T1" >> "$OUT"
  echo "    sim time bracket: $T0 -> $T1"
done

echo
echo "brackets written to $OUT"
cat "$OUT"
