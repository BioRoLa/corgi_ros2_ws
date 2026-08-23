#!/bin/bash
# P-G-1..P-G-4: beta PHASE x tracking GAIN, 2x2 factorial. Log S154.
#
# REGISTERED BEFORE THIS RAN -- S154. A failed gate binds (S126).
#
# WHY. S153: the foot lands 70% through the commanded stance sweep, so the beta
# command carries ~0 propulsion (commanded sweep -0.025 where the template
# intends +0.322). S113 rotated beta and it FAILED -- but the informative part
# is HOW: the command moved (-0.065 of an intended -0.080) and the LEG DID NOT
# (measured -0.013, and swept beta unchanged at +0.123 vs +0.132).
#
# So each lever alone is useless:
#   S113  correct-ish phase + SOFT tracking (k_t 600)  -> leg does not follow
#   S152  stiff tracking    + WRONG phase (and k12000) -> follows a bad command
# Nobody has run correct phase AND stiff tracking together. That is the cell
# this tests, and it is an INTERACTION hypothesis, not "do it harder".
#
# Also: S113 used a 27-row rotation, which reaches only 47% of the achievable
# commanded sweep. k=75 puts the commanded trough exactly at touchdown
# (commanded sweep +0.290 vs the shipped -0.025).
#
# OPERATING POINT: the CONFIG OF RECORD, passed explicitly (Open Issue #19).
# n = 2 is a SCREEN, not a decision.
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
NPER=${NPER:-2}
BASE=${BASE:-/home/alexc/corgi_runs/phase_gain}
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

# cell | template | k_tangential
CELLS=${CELLS:-"A:v070:600.0 B:b75:600.0 C:v070:1200.0 D:b75:1200.0"}

echo "==========================================================="
echo " beta PHASE x tracking GAIN factorial -- P-G-1..P-G-4 (S154)"
echo "==========================================================="
echo " cells    : $CELLS"
echo " n / cell : $NPER      (SCREEN, not a decision)"
echo " flight   : $FLIGHT_ARGS   <- config of record, explicit"
echo " base     : $BASE"
echo

STALE=$(pgrep -f 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | wc -l)
if [ "$STALE" != 0 ]; then
  echo "!! stale sim processes running -- REFUSING to start:"
  pgrep -fa 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' | head
  exit 1
fi
echo "stale-launch check clean."

BIN="$WS/install/corgi_force_control/lib/corgi_force_control/gslip_pronk_node"
if [ "$(strings "$BIN" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" = 0 ]; then
  echo "!! controller lacks the LEG-FRAME GAINS announcement -- rebuild first."
  exit 1
fi
echo "controller carries the gain announcement."

if ! grep -q "sweep_frac_fwd" "$DIAG/touchdown_phase.py"; then
  echo "!! touchdown_phase.py lacks sweep_frac_fwd -- P-G-4 unscoreable."
  exit 1
fi
echo "analyser carries the P-G-4 propulsion screen."

[ -f "$CFG/gslip_pronk_template_v070_b75.csv" ] || {
  echo "!! b75 template missing"; exit 1; }
echo "b75 template present."

# Refuse if the v070 base template is missing -- every cell now names it.
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || {
  echo "!! v070 base template missing"; exit 1; }
echo "v070 base template present (passed explicitly to EVERY cell)."

GUARD=/home/alexc/corgi_runs/shift_duty_sweep/v070_db_stop_odo
gv() { python3 "$DIAG/check_menger.py" --odom-csv "$GUARD/odom_run$1.csv" \
         --torque-csv "$GUARD/run$1.csv" --start 12 2>/dev/null \
       | grep -o 'Menger R / R_fit  *[0-9.]*' | grep -o '[0-9.]*$'; }
G1=$(gv 1); G3=$(gv 3)
echo "estimator guard: $G1 / $G3 (expect 0.943 / 0.952)"
[ "$G1" = "0.943" ] && [ "$G3" = "0.952" ] || {
  echo "!! guard failed -- REFUSING to spend sim time"; exit 1; }
echo "estimator guard OK."
echo

[ -n "${PREFLIGHT_ONLY:-}" ] && {
  echo "PREFLIGHT_ONLY -- all gates passed, stopping."; exit 0; }

mkdir -p "$BASE"

for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"
  TPL="${REST%%:*}"; KT="${REST#*:}"
  # ALWAYS pass template_path EXPLICITLY, including for v070.
  #
  # S159: omitting it does NOT give v070. The controller's fallback is
  # `<share>/config/gslip_pronk_template.csv` (gslip_pronk.cpp:927), which is
  # the v~1.20 template -- 225 rows, period 0.2249 s, beta sweep +0.6357, i.e.
  # a gait designed for 1.92 m/s on a plant that does ~0.28. The first two
  # runs of this factorial compared that against a v070-derived rotation and
  # looked like a spectacular win for the rotation. It was a template swap.
  if [ "$TPL" = "v070" ]; then
    TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
  else
    TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070_$TPL.csv"
  fi
  echo
  echo "################################################################"
  echo "###  CELL $NAME : template $TPL, k_tangential $KT"
  echo "################################################################"
  OUT="$BASE/${NAME}_${TPL}_kt${KT}"
  mkdir -p "$OUT"
  N=$NPER OUTDIR="$OUT" RECORD_ODOM=1 \
    CTL_ARGS="k_tangential:=$KT $FLIGHT_ARGS $TPL_ARG" \
    KT_REF_DIR="$BASE/A_v070_kt600.0" KT_REF="600.0" \
    bash "$DIAG/repeat_gain_regime.sh"
done

echo
echo "==========================================================="
echo " ANALYSIS"
echo "==========================================================="
echo
echo "--- P-G-4 VALIDITY + swept beta (the primary statistic) ---"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"; TPL="${REST%%:*}"; KT="${REST#*:}"
  ARGS="$ARGS --dir $BASE/${NAME}_${TPL}_kt${KT} --label ${NAME}_${TPL}_kt${KT}"
done
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS
echo
echo "  baseline swept beta = +0.1316 (S113 n=5).  P-G-2 bar: > +0.16"
echo "  'sweep' column IS the measured swept beta while down."

echo
echo "--- net forward speed (P-G-2 second bar: > +0.284 m/s) ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"; TPL="${REST%%:*}"; KT="${REST#*:}"
  echo "  cell $NAME ($TPL, k_t $KT):"
  python3 "$DIAG/speed_from_odom.py" "$BASE/${NAME}_${TPL}_kt${KT}"/odom_run*.csv 2>&1 | tail -4
done

echo
echo "--- torque (reported, NOT gated; a RISE is evidence the leg engaged) ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"; TPL="${REST%%:*}"; KT="${REST#*:}"
  echo "  cell $NAME:"
  python3 "$DIAG/tau_demand_window.py" "$BASE/${NAME}_${TPL}_kt${KT}"/run*.csv 2>&1 | tail -5
done
echo
echo "==========================================================="
