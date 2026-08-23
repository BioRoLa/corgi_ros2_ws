#!/usr/bin/env bash
# STANCE-PEAK DIP on the open-loop camber term: thermal and structural relief
# for the ABAD with kappa held. Log S210 registers the bars; this produces the
# captures and self-certifies every run.
#
# WHY. S205: the front ABADs sit AT their 44.25 N.m clamp uncambered and the
# loaded leg (A at dir +1) demands 55-59 N.m p99.5 under 15 deg of camber,
# clipped SILENTLY on 7-10% of stance -- a ~10 ms spike 30-40 ms after
# touchdown. S206/S209: the clip costs no joint angle, at the median or at
# the peak, so this is NOT authority recovery. The clamp is the real motor's
# 9:1 stall torque and usable-at-speed is ~37 N.m, so on hardware the clip is
# heat plus a cantilever at loads #15 never computed. The dip scales the
# camber command toward zero in a raised-cosine window after a debounced
# touchdown (gslip_pronk.cpp gamma_openloop / dip_tick), sized to the ~1-2 deg
# tracking error rather than the command: 0.15 and 0.30 of 15 deg is 2.3 and
# 4.5 deg at the window centre. A 0.6 dip would drive the command 9 deg past
# the measured angle and REVERSE the torque mid-stance -- not registered.
#
# CELLS. lambda 15, dir +1 (A is the loaded leg), config of record, n = 5
# each, INTERLEAVED:
#   nodip   gamma_acker_dip 0.0    -- bit-identical to camber_lambda/lam15
#   dip15   gamma_acker_dip 0.15   -- window 20-60 ms after touchdown, rearm 30
#   dip30   gamma_acker_dip 0.30   -- the scored treatment
# GAIT_SIM 24 to match camber_lambda/lam15 directly (a third, banked control).
#
# COST. 15 runs x ~14 min ~= 3.5 h exclusive simulator. Announce first.
#
# No `set -u` -- ROS setup scripts reference unbound variables.

WS=~/corgi_ws/corgi_ros2_ws
DIAG="$WS/src/corgi_force_control/scripts/diag"
CFG="$WS/src/corgi_force_control/config"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / #26. Prints the plant
# identity into this campaign's own log and refuses a dirty or unbuilt plant.
. "$WS/src/corgi_force_control/scripts/diag/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-5}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
# FRESH BASE: the retry loop tests bare file existence, so a stale run$REP.csv
# from an earlier campaign would read as "succeeded on attempt 1" and be scored.
BASE=${BASE:-/home/alexc/corgi_runs/acker_dip}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
ATT_ARGS="k_yaw:=0.0 d_yaw:=0.0"

CAM_LAM_DEG=${CAM_LAM_DEG:-15}
CAM_LAM_RAD=$(awk -v d="$CAM_LAM_DEG" 'BEGIN{printf "%.5f", d*3.14159265358979/180.0}')
CAM_LAM_FMT=$(printf '%.2f' "$CAM_LAM_DEG")
CAM_DIR=${CAM_DIR:-1.0}
CAM_ARGS="gamma_acker_in:=$CAM_LAM_RAD gamma_acker_out:=$CAM_LAM_RAD gamma_acker_dir:=$CAM_DIR"
DIP_T0=${DIP_T0:-20}; DIP_T1=${DIP_T1:-60}; DIP_REARM=${DIP_REARM:-30}
DIP_WIN="gamma_acker_dip_t0_ms:=$DIP_T0 gamma_acker_dip_t1_ms:=$DIP_T1 gamma_acker_dip_rearm_ms:=$DIP_REARM"
DIP15_ARGS="$CAM_ARGS gamma_acker_dip:=0.15 $DIP_WIN"
DIP30_ARGS="$CAM_ARGS gamma_acker_dip:=0.30 $DIP_WIN"

CELLS=${CELLS:-"nodip dip15 dip30"}

echo "==========================================================="
echo " STANCE-PEAK DIP on the camber term -- P-D-1..P-D-6 (log S210)"
echo "==========================================================="
echo " nodip    : $CAM_ARGS"
echo " dip15    : $DIP15_ARGS"
echo " dip30    : $DIP30_ARGS"
echo " both     : $ATT_ARGS $FLIGHT_ARGS"
echo " template : $TPL_ARG"
echo " n        : $NPER per cell, interleaved by repetition"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " BARS -- registered in S210 BEFORE this ran, and binding (S126):"
echo "   P-D-1  validity: S152 screen + per-run certification (gates)"
echo "   P-D-2  PRIMARY  leg-A stance clip fraction median: dip30 <= 2%  (nodip 7-10%)"
echo "   P-D-3           leg-A tau p99.5 median: dip30 <= 44.25 N.m   (nodip 55-59)"
echo "   P-D-4  GUARD    |kappa| median within +-15% of nodip, both dip cells"
echo "   P-D-5  GUARD    v_fwd within +-10% of nodip; no more collapses than nodip"
echo "   P-D-6  hardware leg-A stance time > 37 N.m: dip30 <= half of nodip"
echo " Relief WITH kappa held is the claim. A dip that buys relief by losing"
echo " the turn fails P-D-4 and that FAIL stands."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
for NEED in 'LEG-FRAME GAINS' 'ATTITUDE GAINS' 'gamma_acker' 'ACKER DIP set'; do
  [ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c "$NEED")" != 0 ] || {
    echo "!! INSTALLED gslip_pronk_node lacks '$NEED' -- a cell could not be"
    echo "!! certified from its own log. Rebuild. REFUSING."; exit 1; }
done
echo "installed controller carries every banner this campaign certifies against."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
NZ=$(awk -F, 'NR>1 && $4+0 != 0 {n++} END {print n+0}' "$CFG/gslip_pronk_template_v070.csv")
[ "$NZ" = "0" ] || { echo "!! v070 gamma column is NOT identically zero. REFUSING."; exit 1; }
echo "v070 present, gamma column identically zero."

# The scorers must pass their own selftests before they are trusted with this.
for T in abad_torque.py score_acker_dip.py; do
  python3 "$DIAG/$T" --selftest > "/tmp/mt_$T.out" 2>&1 || {
    echo "!! $T selftest FAILED -- the bars cannot be scored:"; tail -15 "/tmp/mt_$T.out"; exit 1; }
done
echo "scorer selftests pass."

if [ -d "$BASE" ] && [ -n "$(ls -A "$BASE" 2>/dev/null)" ]; then
  echo "!! $BASE already has content. A fresh campaign needs a fresh base --"
  echo "!! set BASE=... or move the old one. REFUSING."; exit 1
fi
mkdir -p "$BASE"
{
  echo "campaign  acker_dip  $(date -Iseconds)"
  echo "nodip $CAM_ARGS"
  echo "dip15 $DIP15_ARGS"
  echo "dip30 $DIP30_ARGS"
  echo "both  $ATT_ARGS $FLIGHT_ARGS $TPL_ARG"
  echo "n $NPER  gait_sim $GAIT_SIM"
} > "$BASE/DESIGN.txt"

# ---- per-run self-certification -------------------------------------------
certify() {  # certify <cell> <ctl_log> -> 0 ok, 1 INVALID
  local CELL=$1 LOG=$2 ok=1
  grep -q 'k_flight=7150.0 b_flight=115.8' "$LOG" || { echo "  !! flight gains not 7150/115.8"; ok=0; }
  grep -q 'ATTITUDE GAINS: k_yaw=0.0000' "$LOG"    || { echo "  !! k_yaw not certified 0"; ok=0; }
  grep -q 'Loaded 265 template rows' "$LOG"        || { echo "  !! template not 265 rows"; ok=0; }
  grep -q "ACKER CAMBER set: in=${CAM_LAM_FMT} deg" "$LOG" \
    && echo "  ACKER CONFIRMED: $(grep -o 'ACKER CAMBER set: [^"]*' "$LOG" | head -1)" \
    || { echo "  !! ACKER CAMBER not announced at $CAM_LAM_FMT deg"; ok=0; }
  grep -q 'turn_rate=0.0000' "$LOG" || { echo "  !! a turn_rate is set"; ok=0; }
  case "$CELL" in
    nodip)
      grep -q 'ACKER DIP set' "$LOG" && { echo "  !! nodip cell announces a DIP"; ok=0; } ;;
    dip15|dip30)
      WANT=$([ "$CELL" = dip15 ] && echo 0.15 || echo 0.30)
      grep -q "ACKER DIP set: $WANT of the camber term over ${DIP_T0}-${DIP_T1} ms" "$LOG" \
        && echo "  DIP CONFIRMED: $(grep -o 'ACKER DIP set: [^"]*' "$LOG" | head -1)" \
        || { echo "  !! ACKER DIP $WANT not announced"; ok=0; } ;;
  esac
  [ "$ok" = 1 ]
}

run_cell() {  # run_cell <cell> <rep>
  local NAME=$1 REP=$2 OUT="$BASE/$1" ARGS
  case "$NAME" in nodip) ARGS="$CAM_ARGS";; dip15) ARGS="$DIP15_ARGS";; dip30) ARGS="$DIP30_ARGS";; esac
  mkdir -p "$OUT"
  echo
  echo "################################################################"
  echo "###  rep $REP/$NPER, CELL $NAME"
  echo "###  ON THE RENDER: four legs cambered LEFT/RIGHT at $CAM_LAM_DEG deg,"
  echo "###  path curling one way (dir +1). The dip is invisible to the eye --"
  echo "###  2-4 deg for 40 ms after touchdown. Pirouette/stall = a RESULT."
  echo "################################################################"
  for ATTEMPT in 1 2; do
    N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
      GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
      CTL_ARGS="$ARGS $ATT_ARGS $FLIGHT_ARGS $TPL_ARG" \
      bash "$DIAG/repeat_gain_regime.sh"
    if [ -f "$OUT/run$REP.csv" ]; then
      [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME succeeded on RETRY -- cold start, S171 S6)"
      if certify "$NAME" "$OUT/ctl_run$REP.log"; then
        echo "  run $REP CERTIFIED ($NAME)"
      else
        echo "  !! run $REP of $NAME is INVALID -- config not certified. Quarantining."
        mv "$OUT/run$REP.csv" "$OUT/run${REP}_uncertified.csv"
        mv "$OUT/odom_run$REP.csv" "$OUT/odom_run${REP}_uncertified.csv" 2>/dev/null
      fi
      return
    fi
    if [ "$ATTEMPT" = 1 ]; then
      echo "  !! cell $NAME rep $REP produced NO CAPTURE. Cold-start failure mode,"
      echo "  !! not a result. Retrying ONCE."
    else
      echo "  !! cell $NAME rep $REP produced NO CAPTURE on either attempt."
      echo "  !! That rep is LOST for pairing -- note it."
    fi
  done
}

# ---- ATTEMPTS, interleaved by repetition ------------------------------------
for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do run_cell "$CELL" "$REP"; done
done

# ---- ANALYSIS --------------------------------------------------------------
echo
echo "==========================================================="
echo " SCORE -- score_acker_dip.py, as registered in S210"
echo "==========================================================="
python3 "$DIAG/score_acker_dip.py" --base "$BASE"
echo
echo "-- per-leg ABAD torque detail (abad_torque.py) ---------------------------"
python3 "$DIAG/abad_torque.py" --dir "$BASE/nodip" --label nodip --dir "$BASE/dip15" --label dip15 --dir "$BASE/dip30" --label dip30
echo
echo "Done. Captures in $BASE. Record the verdicts in the log as they stand."
