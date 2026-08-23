#!/bin/bash
# P-T-1..P-T-4: the inner/outer stride mismatch, handled EXPLICITLY. Log S194.
# Thesis Timeline Stage 3 task 2.
#
# THE MISMATCH. The contact track is 0.4234 m (not the 0.240 m hip spacing), so
# in a turn of radius R the outer side travels (R+0.2117)/(R-0.2117) further
# than the inner. The timeline calls handling that explicitly "a real G-SLIP
# extension, worth a subsection".
#
# THE LEVER ALREADY EXISTS AND HAS NEVER BEEN RUN. `k_steer` is a declared
# launch parameter (gslip_pronk.cpp:1072), documented as "per-side amplitude
# scale, beta * (1 +- k_steer)", default 0. It is NOT k_steer_yaw, which is the
# heading loop's gain and is what every previous campaign used.
#
# DERIVED, NOT TUNED -- and bracketed, because R is uncertain:
#
#   R = 2.0 m  -> ratio 1.2368 -> k_steer 0.1059   (the timeline's 23.7% case)
#   R = 6.1 m  -> ratio 1.0719 -> k_steer 0.0347   (S127's revised authority,
#                                                   28.7% of geometric at k7150)
#
# S88 put lam=10's curvature at kappa -0.287 (R 3.5 m); S127 pulled authority
# down ~25% at n=5, giving R ~ 6 m. Open Issue #1 says the radius does not
# repeat at all. So the honest design brackets the range rather than asserting
# one k.
#
# RESOLVABILITY, CHECKED BEFORE BUILDING (the S193 lesson):
#   commanded differential 2*k*sweep = 3.91 deg at k=0.106 on the template's
#   0.3223 rad sweep; achieved ~0.42x that (Open Issue #20: the foot lands 70%
#   through the commanded sweep) = 1.65 deg, against a between-leg sd of
#   0.83 deg (S192 screen). 2.0 sd per run, 4.5 sd on the mean at n=5.
#   RESOLVABLE -- unlike S193's cambered template, which was 0.10 sd and was
#   cancelled before it ran.
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
# IS THE SIMULATOR FREE, AND QUIET? S202. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$WS/src/corgi_force_control/scripts/diag/preflight_sim.sh"
preflight_sim || exit 1
NPER=${NPER:-5}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
BASE=${BASE:-/home/alexc/corgi_runs/inner_outer}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"
LAM_RAD=0.17453      # 10.00 deg, the cell S172/S178 measured
ACK_DIR=1.0

# name : k_steer
CELLS=${CELLS:-"k0:0.0 k035:0.0347 k106:0.1059"}

echo "==========================================================="
echo " INNER/OUTER via k_steer -- P-T-1..P-T-4 (log S194)"
echo "==========================================================="
echo " cells    : $CELLS   (name:k_steer)"
echo " camber   : gamma_acker in=out=$LAM_RAD rad (10.00 deg), dir $ACK_DIR"
echo " n        : $NPER, interleaved"
echo " flight   : $FLIGHT_ARGS   <- config of record"
echo " base     : $BASE"
echo
echo " BARS, registered in S194 before this ran and binding (S126):"
echo "   P-T-1  VALIDITY, scored FIRST, one consequence. Every run: full"
echo "          ${GAIT_SIM}s span, playback within +-5%, and the S152 screen."
echo "          A failing run is EXCLUDED AND COUNTED."
echo "   P-T-2  PROOF OF ACTION, before any proof of effect. The per-side"
echo "          BETA SWEEP differential (mean over {A,D} minus mean over"
echo "          {B,C}) scales with k_steer and reaches >= 1.0 deg at k106."
echo "          Falsified if the differential does not appear -- which would"
echo "          mean the channel never engaged, a completely different"
echo "          finding from 'it engaged and did not help'."
echo "   P-T-3  THE EFFECT. If inner/outer mismatch is being paid in scrub,"
echo "          matching stride to path buys it back: at the k that matches"
echo "          the achieved radius, v_fwd rises OR tau p99.5 falls, by more"
echo "          than the run-to-run spread. Registered as a DIRECTION, not a"
echo "          magnitude -- there is no prior worth trusting (S178's lesson)."
echo "   P-T-4  IT MUST NOT COST THE TURN. Curvature |kappa| at k106 is not"
echo "          below k0's by more than its own spread: matching the stride"
echo "          should reduce scrub, not cancel the steering."
echo
echo " NOT claimed: which k is right. R is uncertain (S88 3.5 m, S127 ~6 m,"
echo " Open Issue #1 says it does not repeat), so the two k values BRACKET it."
echo

# ---- PREFLIGHT -------------------------------------------------------------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
# k_steer must exist in the INSTALLED binary. It has never been exercised, so
# "the parameter was silently ignored" is a live failure mode here in a way it
# is not for channels with a run history. src is not install (S28).
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'k_steer')" != 0 ] || {
  echo "!! the INSTALLED gslip_pronk_node has no k_steer parameter. Rebuild."
  exit 1; }
echo "installed controller carries k_steer."
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'gamma_acker')" != 0 ] || {
  echo "!! installed controller has no gamma_acker channel. REFUSING"; exit 1; }
echo "installed controller carries the gamma_acker channel."

for CELL in $CELLS; do
  KS="${CELL#*:}"
  case "$KS" in *.*) ;; *) echo "!! k_steer '$KS' has no decimal point -- ROS"
      echo "!! types a bare integer as int against a double. REFUSING"; exit 1 ;;
  esac
done
echo "double-typed launch args carry decimal points (all cells checked)."

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo; echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }
mkdir -p "$BASE"

# ---- RUN, interleaved ------------------------------------------------------

for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do
    NAME="${CELL%%:*}"; KS="${CELL#*:}"
    OUT="$BASE/$NAME"; mkdir -p "$OUT"
    echo
    echo "################################################################"
    echo "###  rep $REP/$NPER, CELL $NAME : k_steer $KS, camber 10 deg"
    if [ "$KS" = "0.0" ]; then
      echo "###  CONTROL. Camber only -- the inner/outer mismatch is NOT"
      echo "###  handled, which is how every campaign has run to date."
    else
      echo "###  The left pair sweeps (1+k) and the right (1-k), so the"
      echo "###  outer side takes a longer stride. ON THE RENDER: the two"
      echo "###  sides' legs should visibly swing through different arcs."
    fi
    echo "################################################################"
    for ATTEMPT in 1 2; do
      N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="k_steer:=$KS gamma_acker_in:=$LAM_RAD gamma_acker_out:=$LAM_RAD gamma_acker_dir:=$ACK_DIR $FLIGHT_ARGS $TPL_ARG" \
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
echo " ANALYSIS -- P-T-1, then P-T-2 (action) BEFORE P-T-3 (effect)"
echo "==========================================================="
echo
echo "-- P-T-1 + P-T-2: per-leg beta sweep. The DIFFERENTIAL is the proof --"
echo "-- of action: {A,D} minus {B,C}. Without it P-T-3 is unreadable.     --"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/touchdown_phase.py" $ARGS
echo
echo "-- P-T-3: speed and torque ------------------------------------------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  [$NAME]"
  python3 "$DIAG/speed_from_odom.py" --dir "$BASE/$NAME" 2>&1 | grep CELL
  python3 "$DIAG/tau_demand_window.py" --dir "$BASE/$NAME" 2>&1 | grep CELL
done
echo
echo "-- P-T-4: curvature, and whether matching the stride cancels the turn -"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  [$NAME]"
  for n in $(seq 1 "$NPER"); do
    [ -f "$BASE/$NAME/run$n.csv" ] || continue
    python3 "$DIAG/check_menger.py" --torque-csv "$BASE/$NAME/run$n.csv" \
      --odom-csv "$BASE/$NAME/odom_run$n.csv" --start 12 2>&1 \
      | grep -E "Menger|GATE" | head -2
  done
done
echo
echo "-- camber delivery, so a null cannot be blamed on the lean -----------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  [$NAME]"
  for n in $(seq 1 "$NPER"); do
    [ -f "$BASE/$NAME/run$n.csv" ] || continue
    python3 "$DIAG/audit_gamma_decomp.py" --torque-csv "$BASE/$NAME/run$n.csv" \
      --odom-csv "$BASE/$NAME/odom_run$n.csv" \
      --gamma-in 10 --gamma-out 10 --gamma-dir 1 2>&1 \
      | grep -E "L/R|gate FAIL" | head -2
  done
done
echo
echo "Done. Score P-T-1 FIRST, then P-T-2 BEFORE P-T-3 -- a sweep with no"
echo "trend has two explanations and they must be separated (S194)."
