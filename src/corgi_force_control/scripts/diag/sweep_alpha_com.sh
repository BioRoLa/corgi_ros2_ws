#!/bin/bash
# ALPHA ON THE CENTRE OF MASS -- P-W-1..P-W-3. Log S180.
#
# WHY. alpha, the third G-SLIP state (Lu & Lin's touchdown velocity angle), was
# measured here for the first time in S167 at 43.8 deg against the model's
# 5-25 and Eita's 10-18. That number is BODY-ORIGIN velocity differentiated
# from position. The model's alpha belongs to a POINT MASS, and S115 measured
# 17.9-33.8 deg of peak-to-peak pitch that a point mass does not have.
#
#   * the MEDIAN is robust to pitch (it is ~zero-mean over a stride)
#   * the SPREAD is not -- and the spread is the half that matters, because
#     p16-p84 = 50.8 deg is most of a quarter-circle, and it is what decides
#     whether a basin of attraction computed around a POINT means anything
#     against a distribution that wide.
#
# The advisor brief ships with that caveat and cannot lose it until alpha is
# redone on the CoM. This campaign redoes it.
#
# WHAT IS NEW ON THE SIM SIDE. corgi_driver now publishes sim/com_odom --
# getCenterOfMass(), which is the CoM of the robot AND its descendants, so it
# moves with the legs. OFF by default (CORGI_PUBLISH_COM), so every existing
# campaign stays bit-identical. Both env vars are required and they are
# different things: CORGI_PUBLISH_COM makes the driver publish, RECORD_COM
# makes the harness record. Setting only the second yields an EMPTY capture,
# which repeat_gain_regime now shouts about.
#
# TWO CELLS, because the cambered case is free here and nobody knows the
# answer: does lean move alpha? Stage 3 task 3 has to command a lean, and if
# alpha shifts with lambda then the model comparison shifts with it.
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
NPER=${NPER:-3}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/alpha_com}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
CELLS=${CELLS:-"lam0:0.0:0.0:0 lam10:0.17453:1.0:10"}

echo "==========================================================="
echo " ALPHA ON THE CoM -- P-W-1..P-W-3 (log S180)"
echo "==========================================================="
echo " cells    : $CELLS"
echo " n        : $NPER, interleaved"
echo " flight   : $FLIGHT_ARGS   <- config of record"
echo " CoM      : CORGI_PUBLISH_COM=1 RECORD_COM=1 (BOTH required)"
echo " base     : $BASE"
echo
echo " BARS, registered in S180 before this ran and binding (S126):"
echo "   P-W-1  VALIDITY, scored FIRST, one consequence. Every run: full"
echo "          ${GAIT_SIM}s sim span, playback within +-5%, AND a com capture of"
echo "          >= 1000 rows whose row count is within 5% of its own odom"
echo "          capture. A run without a matched CoM stream is EXCLUDED and"
echo "          COUNTED -- it cannot answer the question at all."
echo "   P-W-2  THE POINT OF THE CAMPAIGN. At lam0 the CoM alpha SPREAD"
echo "          (p84-p16) is at least 5 deg NARROWER than the body-origin"
echo "          spread measured on the same runs. S167's body spread is"
echo "          50.8 deg. Falsified if the CoM spread is equal or wider --"
echo "          which would mean the 51 deg is the GAIT, not the pitch, and"
echo "          the brief's caveat can be dropped rather than resolved."
echo "   P-W-3  The MEDIAN moves by less than 5 deg between body and CoM."
echo "          This is the claim S167 already made ('the median is robust');"
echo "          it has never been tested and it is cheap to test here."
echo
echo " NOT claimed: that alpha can be matched to the model. S167 settled that"
echo " it cannot except by fixing speed -- v_horiz is 3-6x low and vz is fine."
echo

# ---- PREFLIGHT -------------------------------------------------------------

STALE=$(pgrep -f 'Corgi_launch.py|gslip_pronk_node|webots_ros2_driver' 2>/dev/null | wc -l)
[ "$STALE" = 0 ] || { echo "!! stale sim processes -- REFUSING"; pgrep -fa 'Corgi_launch.py|gslip_pronk_node' | head; exit 1; }
echo "stale-launch check clean."

FOREIGN=$(pgrep -f 'usr/local/webots' 2>/dev/null | wc -l)
[ "$FOREIGN" = 0 ] || { echo "!! a Linux-side Webots is running -- REFUSING"; exit 1; }
LOAD=$(cut -d' ' -f1 /proc/loadavg)
awk -v l="$LOAD" 'BEGIN{exit !(l > 4.0)}' && { echo "!! load average $LOAD -- REFUSING"; exit 1; }
echo "no foreign Webots; load average $LOAD."

if command -v powershell.exe > /dev/null 2>&1; then
  WINWB=$(powershell.exe -NoProfile -Command \
      "@(Get-Process webots* -ErrorAction SilentlyContinue).Count" 2>/dev/null | tr -d '\r\n ')
  case "$WINWB" in
    ''|*[!0-9]*) echo "windows-side Webots check: inconclusive ('$WINWB') -- continuing." ;;
    0) echo "windows-side Webots check: none running." ;;
    *) echo "!! $WINWB WINDOWS-side webots.exe holds port 1234. REFUSING."; exit 1 ;;
  esac
fi

# THE DRIVER MUST ACTUALLY HAVE THE CoM PUBLISHER, in the INSTALLED copy.
# src is not install: S28 ran nine runs against a stale driver and the result
# looked exactly like the right answer.
INST=$(find "$WS/install/corgi_sim" -name corgi_driver.py 2>/dev/null | head -1)
[ -n "$INST" ] || { echo "!! cannot find the INSTALLED corgi_driver.py -- REFUSING"; exit 1; }
grep -q 'sim/com_odom' "$INST" || {
  echo "!! the INSTALLED driver ($INST) has no sim/com_odom publisher."
  echo "!! Rebuild:  colcon build --packages-select corgi_sim"
  echo "!! Without it CORGI_PUBLISH_COM does nothing, the com capture is empty,"
  echo "!! and alpha would silently come back as the BODY-ORIGIN answer --"
  echo "!! which is the number this campaign exists to replace. REFUSING."
  exit 1; }
echo "installed driver carries the sim/com_odom publisher."

grep -q 'RECORD_COM' "$DIAG/repeat_gain_regime.sh" || {
  echo "!! repeat_gain_regime.sh cannot record sim/com_odom. REFUSING"; exit 1; }
echo "harness can record the CoM stream."

grep -q '\-\-com' "$DIAG/touchdown_velocity_angle.py" || {
  echo "!! touchdown_velocity_angle.py has no --com mode. REFUSING"; exit 1; }
python3 "$DIAG/touchdown_velocity_angle.py" --selftest > /tmp/ac_tva.out 2>&1 || {
  echo "!! touchdown_velocity_angle selftest FAILED:"; tail -12 /tmp/ac_tva.out; exit 1; }
echo "touchdown_velocity_angle selftest PASS (it owns P-W-2 and P-W-3)."

for CELL in $CELLS; do
  R="${CELL#*:}"; L="${R%%:*}"; R="${R#*:}"; D="${R%%:*}"
  for PV in "$L" "$D"; do
    case "$PV" in *.*) ;; *) echo "!! '$PV' has no decimal point -- REFUSING"; exit 1 ;; esac
  done
done
echo "double-typed launch args carry decimal points (all cells checked)."

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo; echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }
mkdir -p "$BASE"

# ---- RUN -------------------------------------------------------------------

for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do
    NAME="${CELL%%:*}"; REST="${CELL#*:}"
    LAM="${REST%%:*}"; REST="${REST#*:}"
    DIR="${REST%%:*}"; LDEG="${REST#*:}"
    OUT="$BASE/$NAME"; mkdir -p "$OUT"
    echo
    echo "################################################################"
    echo "###  rep $REP/$NPER, CELL $NAME (lambda $LDEG deg), CoM publishing ON"
    echo "################################################################"
    for ATTEMPT in 1 2; do
      CORGI_PUBLISH_COM=1 \
        SIM_ASSERT="COM PUBLISH: ON" \
        N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 RECORD_COM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="gamma_acker_in:=$LAM gamma_acker_out:=$LAM gamma_acker_dir:=$DIR $FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME rep $REP succeeded on RETRY -- S171 S6)"
        break
      fi
      [ "$ATTEMPT" = 1 ] && echo "  !! NO CAPTURE -- cold start, retrying once." \
                         || echo "  !! NO CAPTURE on either attempt; n reduced for $NAME."
    done
  done
done

# ---- ANALYSIS --------------------------------------------------------------

echo
echo "==========================================================="
echo " ANALYSIS -- P-W-1 FIRST"
echo "==========================================================="
echo
echo "-- P-W-1: did every run produce a MATCHED CoM stream? ----------------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  for n in $(seq 1 "$NPER"); do
    O="$BASE/$NAME/odom_run$n.csv"; C="$BASE/$NAME/com_run$n.csv"
    if [ ! -f "$C" ]; then echo "  $NAME run$n: NO com capture -- EXCLUDED"; continue; fi
    OR=$(wc -l < "$O" 2>/dev/null || echo 0); CR=$(wc -l < "$C")
    RATIO=$(awk -v a="$CR" -v b="$OR" 'BEGIN{ if (b>0) printf "%.3f", a/b; else print "inf" }')
    VERD=$(awk -v r="$RATIO" 'BEGIN{ print (r>0.95 && r<1.05) ? "ok" : "MISMATCH" }')
    echo "  $NAME run$n: odom $OR rows, com $CR rows, ratio $RATIO -> $VERD"
  done
done
echo
echo "-- proof of intent: the driver's own COM PUBLISH line ----------------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  grep -h -o "COM PUBLISH:.*" "$BASE/$NAME"/sim_run*.log 2>/dev/null | sort -u | sed "s/^/  [$NAME] /"
done
echo
echo "-- P-W-2 / P-W-3: alpha, BODY vs CoM, same runs ----------------------"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && ARGS="$ARGS --dir $BASE/$NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_velocity_angle.py" $ARGS --com
echo
echo "-- context: pitch, which is the thing the CoM is supposed to remove --"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/body_attitude.py" $ARGS
echo
echo "Done. Score P-W-1 FIRST. A run without a matched CoM stream is excluded,"
echo "not interpreted -- S180 owns the bars."
