#!/bin/bash
# P-DC-1..P-DC-3: the small-gain SIGN sweep for the apex diff-camber
# channel. REGISTERED BEFORE THIS RAN -- log S270. A failed gate binds
# (S126). SCREEN at n=3: selects an arm for the post-ICRA adoption
# campaign; adoption cannot follow from this campaign.
#
# WHY. S268 (GT-verified, Open Issue #32): the plant's lambda->rho response
# is SIGN-INVERTED vs the model, so the shipped K row (model sign) is
# positive feedback -- at gain 1.0 it pegged the clamp and killed the gait.
# Hypothesis: the FLIPPED sign (negative gain scalar, k's unchanged) at
# small magnitude regulates the plant's roll. The lean_l anchor condition
# gives a sustained ~4-5 deg GT roll to regulate against.
#
# BARS (S270, registered):
#   P-DC-1  median GT |rollMED|: flip_p1 < off by > 0.5 deg AND
#           model_p1 >= off - 0.5 deg (no comparable improvement)
#   P-DC-2  every flip_* run: median swept beta >= 0.5x off-cell median
#   P-DC-3  |rollMED| flip_p2 <= flip_p1 (weak, dose)
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
    echo "!! gslip_pronk_node lacks the '$BANNER' banner -- rebuild"; exit 1; }
done
echo "controller carries both channel banners (424860b / 71ab26d)."
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }

python3 "$DIAG/roll_from_odom.py" --selftest || {
  echo "!! roll_from_odom selftest FAILED -- P-DC-1 cannot be scored"; exit 1; }

NPER=${NPER:-3}
GAIT_SIM=${GAIT_SIM:-20}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/dc_gain_sweep}
TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
LEAN_ARGS="gamma_acker_dir:=-1.0 gamma_acker_in:=0.1745 gamma_acker_out:=0.1745"

# name : apex_dc_gain    (negative gain = FLIPPED sign; k's stay S267's row)
CELLS=${CELLS:-"off:0.0 model_p1:0.1 flip_p1:-0.1 flip_p2:-0.2"}

echo "==========================================================="
echo " APEX DC small-gain SIGN sweep -- P-DC-1..3 (log S270)"
echo "==========================================================="
echo " cells    : $CELLS   (name:apex_dc_gain)"
echo " n / cell : $NPER, interleaved (S166)"
echo " condition: lean_l anchor ($LEAN_ARGS)"
echo " window   : ${GAIT_SIM}s SIM per run, ${GAIT_WALL}s wall timeout"
echo " base     : $BASE"
echo

mkdir -p "$BASE"
{
  echo "campaign: dc_gain_sweep (log S270, registered P-DC-1..3)"
  echo "date: $(date -Iseconds)"
  echo "cells: $CELLS  n/cell: $NPER interleaved"
  echo "condition: $LEAN_ARGS"
  echo "config: v070 + k7150/115.8 + env vars (repeat_gain_regime defaults)"
  echo "channel: apex_dc k's = S267 K lam_l row; decay S269 defaults 0.8/0.5"
  echo "GAIT_SIM: $GAIT_SIM"
} > "$BASE/DESIGN.txt"

if ! grep -q 'RUN_START' "$DIAG/repeat_gain_regime.sh"; then
  echo "!! repeat_gain_regime.sh has no RUN_START -- REFUSING (see sweep_yaw.sh)"
  exit 1
fi

for REP in $(seq 1 "$NPER"); do
  echo
  echo "%%%%%%%%%%%%%%%%  REPETITION $REP / $NPER  %%%%%%%%%%%%%%%%"
  for CELL in $CELLS; do
    NAME="${CELL%%:*}"; GAIN="${CELL#*:}"
    OUT="$BASE/$NAME"; mkdir -p "$OUT"
    echo
    echo "################################################################"
    echo "###  rep $REP, CELL $NAME : apex_dc_gain $GAIN"
    echo "################################################################"
    # Cold-start retry, once (S171 S6).
    for ATTEMPT in 1 2; do
      N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="apex_dc_gain:=$GAIN $LEAN_ARGS $FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME rep $REP succeeded on RETRY -- cold start)"
        break
      fi
      if [ "$ATTEMPT" = 1 ]; then
        echo "  !! cell $NAME rep $REP: NO CAPTURE -- cold-start retry"
      else
        echo "  !! cell $NAME rep $REP: NO CAPTURE on either attempt."
        echo "  !! n is REDUCED for this cell -- say so when scoring P-DC-*."
      fi
    done
  done
done

echo
echo "==========================================================="
echo " ANALYSIS"
echo "==========================================================="
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done

echo
echo "--- P-DC-1 / P-DC-3: GT roll per run + across-run medians ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/roll_from_odom.py" "$BASE/$NAME"/odom_run[0-9].csv 2>&1 | sed 's/^/    /'
done

echo
echo "--- P-DC-2 validity: swept beta / gait screen (S152) ---"
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS

echo
echo "--- engagement + decay episodes per run (contract lines) ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  for L in "$BASE/$NAME"/ctl_run[0-9].log; do
    [ -f "$L" ] || continue
    F=$(grep -c 'APEX DIFF CAMBER FIRED' "$L")
    D=$(grep -c 'APEX HOLD DECAY: no apex' "$L")
    echo "  $NAME/$(basename "$L"): FIRED-prints $F  decay-episodes $D"
    grep 'APEX DIFF CAMBER FIRED' "$L" | tail -2 | sed 's/^.*]: /    /'
  done
done

echo
echo "--- speed / straightness / yaw, reported not gated ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/speed_from_odom.py" "$BASE/$NAME"/odom_run[0-9].csv 2>&1 | tail -4
done

echo
echo "--- torque p99.5, reported not gated ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  cell $NAME:"
  python3 "$DIAG/tau_demand_window.py" "$BASE/$NAME"/run[0-9].csv 2>&1 | tail -3
done

echo
echo "==========================================================="
echo " P-DC-1..3 are REGISTERED (log S270). A failed gate binds --"
echo " re-register rather than rescue. This is a SCREEN: it selects"
echo " the arm for the post-ICRA adoption campaign; it adopts nothing."
echo "==========================================================="
