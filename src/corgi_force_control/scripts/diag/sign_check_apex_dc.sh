#!/bin/bash
# SIGN CHECK for the apex differential-camber channel (log S267 S4 item 1).
#
# A SCREEN, not a scored campaign -- no P-numbers, nothing adopted from it.
# The expected directions were recorded in S267 S3 BEFORE this ran:
#
#   1. ENGAGEMENT: 'APEX DIFF CAMBER set' at startup and >= 1 'FIRED' line
#      per run (the S138/S151 two-line contract). A run without both is not
#      a test of anything.
#   2. ROLL ANCHOR: gamma_acker_dir=+1 is lean RIGHT (S88, settled
#      empirically). If the S267 frame derivation is right (REP-103 roll,
#      model rho > 0 = lean right), the FIRED lines' rho reads POSITIVE in
#      the lean_r cell and NEGATIVE in lean_l.
#   3. THE LAW: d_lam is dominated by -5.769*rho, so d_lam carries the
#      OPPOSITE sign to rho in both cells.
#   4. TURN ANCHOR (cell validity): dir=+1 = CW/right turn (S88), so the
#      lean_r cell's yaw rate is negative, lean_l positive. If this fails,
#      the anchor itself is suspect and nothing else in the cell scores.
#
# vy is REPORTED, not judged -- apex-sampled body velocity is known
# surge-dominated (S139) and its frame is the open question S267 names.
#
# apex_dc_gain=1.0: full K-row authority so the FIRED lines show the real
# correction; the offsets ride ON TOP of the 10 deg commanded lean and are
# clamped to 4 deg, so the gait keeps its S90-measured linear-region lean.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"

. "$DIAG/preflight_plant.sh";  preflight_plant || exit 1
. "$DIAG/preflight_sim.sh";    preflight_sim  || exit 1

BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'APEX DIFF CAMBER')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the APEX DIFF CAMBER banner -- rebuild (424860b)"
  exit 1; }
echo "controller carries the APEX DIFF CAMBER banner."
[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }

GAIT_SIM=${GAIT_SIM:-20}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/dc_signcheck}
TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
LEAN=0.1745   # 10 deg -- inside the S90-measured linear lean region
DC_ARGS="apex_dc_gain:=1.0"

# name : gamma_acker_dir
CELLS=${CELLS:-"lean_r:1.0 lean_l:-1.0"}

echo "==========================================================="
echo " APEX DIFF CAMBER sign check (log S267) -- SCREEN, n=1/cell"
echo "==========================================================="
echo " cells   : $CELLS (name:gamma_acker_dir), lean $LEAN rad in=out"
echo " channel : $DC_ARGS (K lam_l row defaults, clamp 4 deg)"
echo " window  : ${GAIT_SIM}s SIM per run, ${GAIT_WALL}s wall timeout"
echo " base    : $BASE"
echo

mkdir -p "$BASE"

for CELL in $CELLS; do
  NAME="${CELL%%:*}"; DIR="${CELL#*:}"
  OUT="$BASE/$NAME"; mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  cell $NAME : gamma_acker_dir $DIR"
  echo "################################################################"
  # Cold-start retry, once (S171 S6 -- first launch after an idle gap fails).
  for ATTEMPT in 1 2; do
    N=1 RUN_START=1 OUTDIR="$OUT" RECORD_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="gamma_acker_dir:=$DIR gamma_acker_in:=$LEAN gamma_acker_out:=$LEAN $DC_ARGS $FLIGHT_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run1.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME succeeded on RETRY -- cold start, S171 S6)"
      break
    fi
    if [ "$ATTEMPT" = 1 ]; then
      echo "  !! cell $NAME produced NO CAPTURE. Cold-start mode; retrying ONCE."
    else
      echo "  !! cell $NAME produced NO CAPTURE on either attempt."
    fi
  done
done

echo
echo "==========================================================="
echo " ANALYSIS (screen)"
echo "==========================================================="
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  L="$BASE/$NAME/ctl_run1.log"
  echo
  echo "--- cell $NAME ---"
  [ -f "$L" ] || { echo "!! no ctl log banked"; continue; }
  grep -m1 'APEX DIFF CAMBER set' "$L" \
    || echo "!! NO 'set' BANNER -- the parameter never reached the node"
  echo "FIRED lines: $(grep -c 'APEX DIFF CAMBER FIRED' "$L")"
  grep 'APEX DIFF CAMBER FIRED' "$L" | tail -8
  grep -o 'd_lam=[+-][0-9.]*' "$L" | cut -d= -f2 \
    | awk '{s+=$1; n++} END{if(n) printf "  mean d_lam %+0.3f deg over %d fired\n", s/n, n}'
  grep -o 'rho [+-][0-9.]* deg' "$L" \
    | awk '{s+=$2; n++} END{if(n) printf "  mean rho   %+0.3f deg over %d fired\n", s/n, n}'
  grep -o 'vy [+-][0-9.]*' "$L" \
    | awk '{s+=$2; n++} END{if(n) printf "  mean vy    %+0.4f m/s over %d fired (REPORTED ONLY)\n", s/n, n}'
  echo "  turn anchor (dir=+1 => yaw rate NEGATIVE, S88):"
  python3 "$DIAG/speed_from_odom.py" "$BASE/$NAME"/odom_run*.csv 2>&1 | tail -6
done

echo
echo "==========================================================="
echo " Expected if S267's derivation is right:"
echo "   lean_r: rho > 0, d_lam < 0, yaw rate < 0"
echo "   lean_l: rho < 0, d_lam > 0, yaw rate > 0"
echo " A flip in rho vs the turn anchor = the roll-frame derivation is"
echo " wrong (one-parameter fix, apex_dc_k_rho sign, log the finding)."
echo "==========================================================="
