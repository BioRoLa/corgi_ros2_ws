#!/bin/bash
# P-L-1..P-L-4: raise the LAGGARD leg. k_tangential x k_pitch. Log S161.
#
# REGISTERED BEFORE THIS RAN -- S161. A failed gate binds (S126).
#
# WHY. S155/S160: the body travels at the pace its SLOWEST leg permits
# (rho = +0.886 cross-campaign, +0.800 out-of-sample). A,B are FRONT and lead;
# C,D are REAR and lag (report_kt_sweep.py:26). That pair split is S17's pitch
# moment, and k_pitch is differential front/rear theta -- the only parameter
# that addresses it directly.
#
# Two levers are known to raise the laggard and have NEVER been run together:
#   k_tangential 1200   laggard 0.188 -> 0.304   (S160, config of record)
#   k_pitch -0.30       laggard 0.202 -> 0.256   (SS119-122 + S155)
# k_pitch was rejected for failing to damp PITCH; v_fwd was never scored alone.
#
# ALL FOUR CONFIGURATION ELEMENTS ARE NAMED (S159). Omitting any one of them
# silently selects a different, older design point:
#   template_path  -> default is the v~1.20 template, NOT v070
#   k_flight/b_flight -> default 12000/150, not the config of record 7150/115.8
#   CORGI_DIRBETA_TRANSFORM / CORGI_THETA_STOP -> set by repeat_gain_regime.sh
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-3}
BASE=${BASE:-/home/alexc/corgi_runs/laggard}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

# name : k_tangential : k_pitch
CELLS=${CELLS:-"base:600.0:0.0 kt:1200.0:0.0 kp:600.0:-0.30 both:1200.0:-0.30"}

echo "==========================================================="
echo " LAGGARD campaign -- P-L-1..P-L-4 (log S161)"
echo "==========================================================="
echo " cells    : $CELLS   (name:k_tangential:k_pitch)"
echo " n / cell : $NPER   (SCREEN)"
echo " template : $TPL_ARG"
echo " flight   : $FLIGHT_ARGS"
echo " base     : $BASE"
echo


BIN="$WS/install/corgi_force_control/lib/corgi_force_control/gslip_pronk_node"
[ "$(strings "$BIN" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" != 0 ] || {
  echo "!! controller lacks the gain announcement -- rebuild"; exit 1; }
echo "controller carries the gain announcement."

grep -q "sweep_frac_fwd" "$DIAG/touchdown_phase.py" || {
  echo "!! analyser lacks the propulsion screen"; exit 1; }
echo "analyser carries the propulsion screen."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
echo "v070 present, named explicitly on every cell."

GUARD=/home/alexc/corgi_runs/shift_duty_sweep/v070_db_stop_odo
gv() { python3 "$DIAG/check_menger.py" --odom-csv "$GUARD/odom_run$1.csv" \
         --torque-csv "$GUARD/run$1.csv" --start 12 2>/dev/null \
       | grep -o 'Menger R / R_fit  *[0-9.]*' | grep -o '[0-9.]*$'; }
G1=$(gv 1); G3=$(gv 3)
echo "estimator guard: $G1 / $G3 (expect 0.943 / 0.952)"
[ "$G1" = "0.943" ] && [ "$G3" = "0.952" ] || { echo "!! guard failed"; exit 1; }
echo "estimator guard OK."
echo

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo "PREFLIGHT_ONLY -- gates passed."; exit 0; }

mkdir -p "$BASE"

for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"
  KT="${REST%%:*}"; KP="${REST#*:}"
  echo
  echo "################################################################"
  echo "###  CELL $NAME : k_tangential $KT, k_pitch $KP"
  echo "################################################################"
  OUT="$BASE/$NAME"
  mkdir -p "$OUT"
  N=$NPER OUTDIR="$OUT" RECORD_ODOM=1 \
    CTL_ARGS="k_tangential:=$KT k_pitch:=$KP $FLIGHT_ARGS $TPL_ARG" \
    KT_REF_DIR="$BASE/base" KT_REF="600.0" \
    bash "$DIAG/repeat_gain_regime.sh"
done

echo
echo "==========================================================="
echo " ANALYSIS"
echo "==========================================================="
echo
echo "--- validity + swept beta ---"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS

echo
echo "--- P-L-3 speed + P-L-4 STRAIGHTNESS (conjunctive, binds adoption) ---"
echo "    bars: v_fwd > +0.324 ; straightness >= 0.80 ; |yaw| <= 6.0 deg/s"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/speed_from_odom.py" "$BASE/$NAME"/odom_run*.csv 2>&1 | tail -4
done

echo
echo "--- torque (reported, not gated) ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/tau_demand_window.py" "$BASE/$NAME"/run*.csv 2>&1 | tail -4
done
echo
echo "==========================================================="
echo " P-L-1 / P-L-2 (min vLeg, and rho against v_fwd) are scored"
echo " separately -- the laggard metric is not in any shipped analyser."
echo "==========================================================="
