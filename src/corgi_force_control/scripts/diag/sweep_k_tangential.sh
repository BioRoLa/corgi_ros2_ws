#!/bin/bash
# Does lowering k_tangential reduce the front/rear leg desynchronisation?
#
# WHY THIS LEVER. The pitch moment that splits the legs front/rear is generated
# by the TANGENTIAL leg force: perpendicular to the leg, so it acts on the full
# 0.293 m hip-to-contact arm rather than the few-centimetre hip-to-COM offset
# the axial force sees. That makes it ~30x the axial term at a realistic
# tracking error, and the moment is LINEAR in k_tangential.
#
# k_tangential has never been tuned against this. It was a launch default of
# 1200, later dropped to 600 on this reasoning (desync 40% -> 33%). This sweeps
# it properly.
#
# EXPECT A MINIMUM, NOT A MONOTONIC WIN. k_tangential is also the fore-aft
# compliance that lets the leg track the template's sweep, and beta tracking
# error is itself an input to the moment. Too soft and the leg stops following
# the sweep, which feeds the very error being minimised.
#
# n = 2 PER ARM IS A SCREEN, NOT A DECISION. Identical-settings runs already
# span 33.7-39.4% on front/rear desync and 0.65-0.75 on the regime ratio, so
# only a large effect is detectable at n = 2. Anything promising gets repeated
# at n = 3 before it is believed.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
ARMS=${ARMS:-"150.0 300.0 600.0 1200.0"}
NPER=${NPER:-2}
BASE=${BASE:-/home/alexc/corgi_runs/kt_sweep}

mkdir -p "$BASE"
echo "k_tangential sweep: arms [$ARMS], n=$NPER each"
echo "current shipped default is 600.0"
echo

for KT in $ARMS; do
  echo "=================== k_tangential = $KT ==================="
  OUT="$BASE/kt$KT"
  mkdir -p "$OUT"
  N=$NPER OUTDIR="$OUT" CTL_ARGS="k_tangential:=$KT" \
      bash "$WS/src/corgi_force_control/scripts/diag/repeat_gain_regime.sh"
done

echo
echo "=================== ANALYSIS ==================="
for KT in $ARMS; do
  echo
  echo "--- k_tangential = $KT ---"
  python3 "$WS/src/corgi_force_control/scripts/diag/desync_vs_gain_regime.py" \
      "$BASE/kt$KT"/run*.csv 2>/dev/null | sed -n '/run  *n/,/^$/p'
done
