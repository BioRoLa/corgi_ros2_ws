#!/bin/bash
# P-DS-1..P-DS-3: the STRAIGHT-orbit screen for the CORRECTED dc row.
# REGISTERED BEFORE THIS RAN -- log S275. A failed gate binds (S126).
# SCREEN at n=3: full separation or UNCERTIFIABLE; adopts nothing;
# the ICRA posture (deadbeat OFF) is untouched either way.
#
# Condition: straight config of record, NO acker, NO turn -- the regime
# where the model's fixed point is valid and apexes fire every stride.
# The `on` cell runs S274's corrected K row, rho/rho_dot ONLY (k_vy 0,
# S139 surge-noise caveat). Wander metric scored post-hoc per S275's
# recipe (0.41 s rolling-median roll, steady band, per-run sd).
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"

. "$DIAG/preflight_plant.sh";  preflight_plant || exit 1
. "$DIAG/preflight_sim.sh";    preflight_sim  || exit 1

BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for BANNER in 'APEX DIFF CAMBER' 'APEX HOLD DECAY'; do
  [ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c "$BANNER")" != 0 ] || {
    echo "!! gslip_pronk_node lacks '$BANNER' -- rebuild"; exit 1; }
done
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }

NPER=${NPER:-3}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/dc_straight_screen}
TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
# S274 corrected row, rho/rho_dot only (registered S275).
DC_ON="apex_dc_gain:=1.0 apex_dc_k_vy:=0.0 apex_dc_k_rho:=-1.4376 apex_dc_k_drho:=-0.0428"

CELLS=${CELLS:-"off on"}

echo "==========================================================="
echo " APEX DC straight-orbit screen -- P-DS-1..3 (log S275)"
echo "==========================================================="
echo " cells: off | on ($DC_ON)"
echo " n/cell $NPER interleaved, GAIT_SIM ${GAIT_SIM}s, base $BASE"
echo

mkdir -p "$BASE"
{
  echo "campaign: dc_straight_screen (log S275, registered P-DS-1..3)"
  echo "date: $(date -Iseconds)"
  echo "cells: off | on: $DC_ON"
  echo "n/cell: $NPER interleaved; straight config of record; GAIT_SIM $GAIT_SIM"
  echo "wander baseline (12 banked runs): sd median 0.223 [0.155,0.434] deg"
} > "$BASE/DESIGN.txt"

for REP in $(seq 1 "$NPER"); do
  echo
  echo "%%%%%%%%%%%%%%%%  REPETITION $REP / $NPER  %%%%%%%%%%%%%%%%"
  for NAME in $CELLS; do
    OUT="$BASE/$NAME"; mkdir -p "$OUT"
    if [ "$NAME" = "on" ]; then EXTRA="$DC_ON"; else EXTRA=""; fi
    echo
    echo "###  rep $REP, CELL $NAME"
    for ATTEMPT in 1 2; do
      N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="$EXTRA $FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME rep $REP succeeded on RETRY)"
        break
      fi
      [ "$ATTEMPT" = 1 ] && echo "  !! no capture -- cold-start retry" \
        || echo "  !! NO CAPTURE on either attempt; n REDUCED for $NAME"
    done
  done
done

echo
echo "==========================================================="
echo " ANALYSIS (in-harness half; wander sd scored post-hoc, S275)"
echo "==========================================================="
python3 "$DIAG/touchdown_phase.py" --dir "$BASE/off" --label off --dir "$BASE/on" --label on
echo
for NAME in $CELLS; do
  for L in "$BASE/$NAME"/ctl_run[0-9].log; do
    [ -f "$L" ] || continue
    F=$(grep -c 'APEX DIFF CAMBER FIRED' "$L")
    D=$(grep -c 'APEX HOLD DECAY: no apex' "$L")
    echo "  $NAME/$(basename "$L"): FIRED-prints $F  decay-episodes $D  [P-DS-3: on needs F>=3, D=0]"
    grep 'APEX DIFF CAMBER FIRED' "$L" | tail -2 | sed 's/^.*]: /    /'
  done
done
echo
for NAME in $CELLS; do
  echo "  cell $NAME:"
  python3 "$DIAG/speed_from_odom.py" "$BASE/$NAME"/odom_run[0-9].csv 2>&1 | tail -2
  python3 "$DIAG/roll_from_odom.py" "$BASE/$NAME"/odom_run[0-9].csv 2>&1 | tail -1
done
echo
echo " P-DS-1..3 are REGISTERED (log S275). Full separation or"
echo " UNCERTIFIABLE; a failed bar binds. Wander sd: run the S275"
echo " analyzer on the banked odom before scoring P-DS-1."
echo "==========================================================="
