#!/bin/bash
# P-V-1..P-V-6: is there a flight cost of lean AT ALL on the config of record?
# Log S176. Successor to sweep_camber_pattern.sh (S171 S5 / S172).
#
# WHY THE QUESTION CHANGED. The handover asked "does the flight cost scale with
# lambda?" and specified lambda in {0,5,10,15} x n>=3 at both operating points.
# S175 answered most of that offline, from 35 banked runs, at zero sim cost:
#
#   * P-U-1 FALSIFIED. At k12000, flight medians across lambda 0/5/10/15 are
#     35.6 / 36.3 / 37.4 / 34.2 -- a 3.2 point span against within-cell spreads
#     of up to 21.9. There is no camber flight cost there at any lean tested.
#     menger_acker_final, the tightest banked campaign (spreads 0.9-3.4), reads
#     +0.9 points at lambda = 10, in BOTH turn directions.
#   * So the k12000 probe this campaign was going to spend runs on is already
#     banked, and lambda = 5 is flat. Both are dropped.
#   * What S175 could NOT settle is k7150, where the only evidence is S172's
#     n = 1 pair. That is what this campaign buys.
#
# AND THE METRIC IS UNDER SUSPICION. S175 S3, from two contact-independent
# signals in the odom z channel:
#
#   * APEX EXCURSION DOES NOT FALL with camber at k7150 -- 49.2 -> 53.8 mm,
#     it RISES, while measured flight goes 24.1 -> 16.6%. A genuinely shorter
#     aerial phase lowers the apex.
#   * The fraction of time LABELLED stance that is nonetheless within 25% of
#     free fall rises 24.6 -> 32.5%. The body keeps doing the ballistic arc and
#     the contact flag is on for more of it -- S173's d_wheel edge contact
#     showing up in the gait.
#   * The detected flight windows ARE ballistic (zdd -10.04..-10.32 vs -9.81 in
#     every cell), so the detector is not calling stance "flight". Its error is
#     entirely in the other direction.
#
# Those bars were NOT registered before they were looked at, so S175 records
# them as observations. They are registered HERE, before this runs.
#
# THIS IS ALSO STAGE 3 TASK 4. Timeline: "touchdown detection under camber --
# contact timing shifts with lean because the rim reaches ground at a different
# alpha." That is the same measurement as the artifact control, so it is scored
# as one thing rather than run twice.
#
# DESIGN. lambda in {0, 10, 15}, uniform LEFT/RIGHT (in == out), n = 5,
# INTERLEAVED by repetition. n = 5 and not 3 because S175 S2 decomposed the
# flight variance and found camber moves it BETWEEN runs: sd_between 4.07 at
# lambda 10 and 3.78 at lambda 15 against sd_within ~2.4, i.e. 70-74% between.
# That REVERSES S167 S4's advice for min vLeg (where longer runs won) -- for
# flight under camber, n is the lever, not T. A variance answer for one metric
# is not a variance answer for the project.
#
# ALL SIX CONFIGURATION ELEMENTS ARE NAMED. template_path, k_flight/b_flight
# and the two env vars are S159's four (the env vars default correctly inside
# repeat_gain_regime.sh -- read, not assumed); GAIT_SIM is S166's; k_yaw is 0.0
# by default since S168 and is certified from each run's own ATTITUDE GAINS.
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
NPER=${NPER:-5}
GAIT_SIM=${GAIT_SIM:-24}
GAIT_WALL=${GAIT_WALL:-420}
# FRESH BASE. sweep_camber_pattern's retry loop tests bare file existence, so a
# stale run$REP.csv from an earlier campaign would be "succeeded on attempt 1"
# and then scored. A new campaign gets a new directory.
BASE=${BASE:-/home/alexc/corgi_runs/camber_lambda}

TPL_ARG="template_path:=$CFG/gslip_pronk_template_v070.csv"
FLIGHT_ARGS="k_flight:=7150.0 b_flight:=115.8"

ACK_DIR=${ACK_DIR:-1.0}

# CELLS are name:lambda_rad:dir:lambda_deg. The fourth field exists because
# lambda now varies per cell and audit_gamma_decomp wants DEGREES -- deriving
# it from the radians in shell arithmetic would be a rounding bug waiting to
# happen. 0.17453 rad = 10.00 deg, 0.26180 rad = 15.00 deg.
#   lam0    camber channel fully OFF (dir 0.0 makes gamma_openloop return
#           literal 0.0), so cmd->gamma is gamma_correction's roll term alone.
#           This is the PAIRED CONTROL and every bar below is a difference
#           against it -- S171's lesson, and S175 S1 is why it matters more
#           than it looked: at lambda 0 flight is stable run to run, under
#           camber it is not.
#   lam10   the cell S172 measured at n = 1. Replication is the point.
#   lam15   does anything appear further out, on the plant of record.
CELLS=${CELLS:-"lam0:0.0:0.0:0 lam10:0.17453:$ACK_DIR:10 lam15:0.26180:$ACK_DIR:15"}

echo "==========================================================="
echo " IS THERE A FLIGHT COST OF LEAN AT ALL? -- P-V-1..P-V-6 (log S176)"
echo "==========================================================="
echo " pattern  : gamma = lambda * {+1,-1,-1,+1}  (lr_sign, A=FL B=FR C=RR D=RL)"
echo " cells    : $CELLS"
echo " n        : $NPER, interleaved by repetition"
echo " template : $TPL_ARG"
echo " flight   : $FLIGHT_ARGS   <- config of record, k7150"
echo " window   : ${GAIT_SIM}s of SIM time per run (NOT wall), ${GAIT_WALL}s timeout"
echo " base     : $BASE"
echo
echo " CONTEXT: S175 falsified P-U-1 offline. At k12000 flight does NOT fall"
echo " with camber (35.6/36.3/37.4/34.2 across lambda 0/5/10/15, n=4..6), and"
echo " menger_acker_final reads +0.9 points at lambda 10. The ONLY evidence for"
echo " a flight cost is S172's n=1 pair at k7150. This campaign replicates it."
echo
echo " BARS -- registered in S176 BEFORE this ran, and binding (S126):"
echo "   P-V-1  VALIDITY, conjunctive, scored FIRST, ONE consequence."
echo "          Every run: full ${GAIT_SIM}s sim span, playback within +-5% on"
echo "          theta, >= 8 touchdowns/leg, >= 1000 in-band samples."
echo "          A run failing this is EXCLUDED AND COUNTED. If a cell keeps"
echo "          fewer than 3 runs, that cell is unscored and says so."
echo "          (S172 drafted a gate with TWO consequences and had to say so"
echo "           afterwards. This one has one.)"
echo "   P-V-2  THE PRIMARY BAR, and it is CONTACT-INDEPENDENT. Median apex"
echo "          excursion at lam10 is >= 4 mm BELOW its paired lam0 control."
echo "          Falsified if apex is unchanged or higher -- which is what the"
echo "          n=1 banked pair shows (49.2 -> 53.8 mm). Apex is primary"
echo "          because it is the signal NOT under suspicion."
echo "   P-V-3  Median flight at lam10 is >= 4 points below its paired lam0"
echo "          control. This is S172's claim, at n = 5 instead of n = 1."
echo "   P-V-4  THE DISCRIMINATOR. If P-V-3 holds and P-V-2 fails, the flight"
echo "          drop is a contact-LABELLING effect and S172's interpretation"
echo "          is withdrawn (the measurement stands). If both hold, the"
echo "          aerial phase really does shrink and the lean budget is real."
echo "          If neither holds, S172's pair was one draw from a widened"
echo "          distribution. Registered so no combination is unfalsifiable."
echo "   P-V-5  Ballistic-while-labelled-DOWN rises by >= 4 points from lam0"
echo "          to lam10. This is Stage 3 task 4's prediction stated as a"
echo "          number: contact timing shifts with lean."
echo "   P-V-6  VARIANCE, and it is a prediction not a nuisance. Within-cell"
echo "          flight spread at lam10 EXCEEDS lam0's by >= 5 points."
echo "          S175 S2 measured exactly this at k12000 (2.6 -> 21.9) and it"
echo "          has never been tested at k7150."
echo
echo " NOT claimed: steering authority, curvature, speed. S127 owns authority"
echo " (29-42% of geometric) and this plant has had no heading hold since S168."
echo

# ---- PREFLIGHT. Everything here runs before any sim time is spent. ---------


BIN="$WS/install/corgi_force_control/lib/corgi_force_control"
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'LEG-FRAME GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the gain announcement -- rebuild"; exit 1; }
echo "controller carries the gain announcement."

# PROOF OF INTENT. k_yaw must be certifiable as 0.0 from the run's own log:
# the whole reason this run is clean is that the yaw term is off, and a k_yaw
# that quietly came back would look exactly like "the pattern leaks into F/R"
# -- i.e. it would falsify P-R-1 for entirely the wrong reason.
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'ATTITUDE GAINS')" != 0 ] || {
  echo "!! gslip_pronk_node lacks the ATTITUDE GAINS banner -- k_yaw cannot be"
  echo "!! certified. Rebuild before running this."; exit 1; }
echo "controller carries the ATTITUDE GAINS banner."

# The open-loop camber channel must exist in the INSTALLED binary. src is not
# install -- S28's ceiling sweep ran nine runs against a stale driver and the
# result looked exactly like the right answer.
[ "$(strings "$BIN/gslip_pronk_node" 2>/dev/null | grep -c 'gamma_acker')" != 0 ] || {
  echo "!! the INSTALLED gslip_pronk_node has no gamma_acker parameters --"
  echo "!! the camber command would silently do nothing. Rebuild. REFUSING."
  exit 1; }
echo "installed controller carries the gamma_acker channel."

[ -f "$CFG/gslip_pronk_template_v070.csv" ] || { echo "!! v070 missing"; exit 1; }
NZ=$(awk -F, 'NR>1 && $4+0 != 0 {n++} END {print n+0}' "$CFG/gslip_pronk_template_v070.csv")
[ "$NZ" = "0" ] || {
  echo "!! v070 gamma column is NOT identically zero ($NZ rows) -- the template"
  echo "!! would contribute camber and P-R-1 would not be readable. REFUSING."
  exit 1; }
echo "v070 present, named explicitly, gamma column identically zero."

# audit_gamma_decomp OWNS P-R-1 and P-R-2. Gate it before spending sim time:
# its selftest plants a known offset on ONE pure partition at a time and must
# recover that projection and put ~0 in the others.
python3 "$DIAG/audit_gamma_decomp.py" --selftest > /tmp/cp_agd.out 2>&1 || {
  echo "!! audit_gamma_decomp selftest FAILED -- P-R-1/P-R-2 cannot be scored:"
  tail -20 /tmp/cp_agd.out; exit 1; }
echo "audit_gamma_decomp selftest PASS (it owns the delivery cross-check)."

# flight_vs_camber owns P-V-1, P-V-2, P-V-3, P-V-5 and P-V-6 -- i.e. every bar
# in this campaign except the engagement banners. It is NEW, written today, so
# it gets gated harder than the tools it wraps. Its selftest plants gaits of
# known duty, plants chatter in both directions and checks the debounce
# recovers, and -- the one that matters -- reproduces S172's OWN published
# numbers (24.4 / 17.2% flight, L/R 9.676, F/R -0.033) from the banked
# camber_pattern captures. S173's rule: the check that catches a frame error
# has to be a number somebody else derived.
python3 "$DIAG/flight_vs_camber.py" --selftest > /tmp/cl_fvc.out 2>&1 || {
  echo "!! flight_vs_camber selftest FAILED -- P-V-1..P-V-6 cannot be scored:"
  tail -25 /tmp/cl_fvc.out; exit 1; }
echo "flight_vs_camber selftest PASS (it owns P-V-1,2,3,5,6 and reproduces S172)."

# body_attitude owns P-R-3, and its signed roll columns were added TODAY for
# this run. Refuse if missing -- an unsigned roll cannot answer a question
# whose entire content is a direction.
grep -q "roll_med" "$DIAG/body_attitude.py" || {
  echo "!! body_attitude.py has no signed roll_med -- P-R-3 cannot be scored."
  exit 1; }
echo "body_attitude carries signed roll (context, not a bar here)."

# DOUBLE-TYPED PARAMETERS MUST LOOK LIKE DOUBLES. A bare integer makes
# gslip_pronk_node abort at startup (InvalidParameterTypeException) BEFORE the
# hold, so the failure reads as "controller never reached hold" and says
# nothing about the type. Refuse here instead, where the message is useful.
#
# WIDENED from sweep_camber_pattern.sh, which checked only $LAM_RAD and
# $ACK_DIR -- i.e. NOT the values actually parsed out of $CELLS and handed to
# ros2 launch. Anyone overriding CELLS= walked straight past the gate written
# to catch this. Here every rad and dir field of every cell is checked.
for CELL in $CELLS; do
  C_REST="${CELL#*:}"
  C_LAM="${C_REST%%:*}"
  C_REST="${C_REST#*:}"
  C_DIR="${C_REST%%:*}"
  for PV in "$C_LAM" "$C_DIR"; do
    case "$PV" in
      *.*) ;;
      *) echo "!! cell '${CELL%%:*}' passes '$PV' to a gamma_acker_* parameter"
         echo "!! with no decimal point. Those are DOUBLE parameters and ROS"
         echo "!! types a bare integer as int, which aborts the controller at"
         echo "!! startup. Write it as ${PV}.0. REFUSING."
         exit 1 ;;
    esac
  done
done
echo "double-typed launch args carry decimal points (all cells checked)."


echo
[ -n "${PREFLIGHT_ONLY:-}" ] && { echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }
mkdir -p "$BASE"

# ---- RUN -------------------------------------------------------------------

# INTERLEAVED by repetition, blocked only within a repetition. A blocked design
# confounds cell with time, which is exactly what S161 lost and what S166's
# analysis-load confound cost again. Free here -- every run relaunches the
# stack anyway -- and it fails EARLIER: a broken cell shows up in rep 1 rather
# than after the first cell has consumed a third of the campaign.
for REP in $(seq 1 "$NPER"); do
  for CELL in $CELLS; do
    NAME="${CELL%%:*}"; REST="${CELL#*:}"
    LAM="${REST%%:*}"; REST="${REST#*:}"
    DIR="${REST%%:*}"; LDEG="${REST#*:}"
    OUT="$BASE/$NAME"
    mkdir -p "$OUT"
    echo
    echo "################################################################"
    if [ "$NAME" = "lam0" ]; then
      echo "###  rep $REP/$NPER, CELL lam0 : PAIRED CONTROL, camber channel OFF"
      echo "###  ON THE RENDER: legs vertical, no visible camber at all."
      echo "###  Every bar in this campaign is a DIFFERENCE against this cell."
    else
      echo "###  rep $REP/$NPER, CELL $NAME : gamma = $LDEG deg * {+1,-1,-1,+1}, dir $DIR"
      echo "###  ON THE RENDER: all four legs cambered LEFT/RIGHT -- the two"
      echo "###  left legs (A=FL, D=RL) one way, the two right (B=FR, C=RR)"
      echo "###  the other. Front and rear on the SAME side must MATCH."
      echo "###  If you see FRONT-vs-REAR instead, the yaw term is back and"
      echo "###  the run is void. The body leans and drifts right; there is"
      echo "###  no heading hold since S168, so slow drift is expected."
    fi
    echo "################################################################"
    # RETRY A COLD START, once. Measured 2026-08-22 (S171 S6): the FIRST
    # launch after an idle gap fails -- either webots.exe dies before the
    # driver connects, or the driver never connects at all -- and the very
    # next launch succeeds. repeat_gain_regime.sh prints "skipping" and
    # returns 0, so a campaign SILENTLY LOSES that cell.
    for ATTEMPT in 1 2; do
      N=1 RUN_START=$REP OUTDIR="$OUT" RECORD_ODOM=1 \
        GAIT_SIM=$GAIT_SIM GAIT_WALL=$GAIT_WALL \
        CTL_ARGS="gamma_acker_in:=$LAM gamma_acker_out:=$LAM gamma_acker_dir:=$DIR $FLIGHT_ARGS $TPL_ARG" \
        bash "$DIAG/repeat_gain_regime.sh"
      if [ -f "$OUT/run$REP.csv" ]; then
        [ "$ATTEMPT" = 2 ] && echo "  (cell $NAME succeeded on RETRY -- cold start, S171 S6)"
        break
      fi
      if [ "$ATTEMPT" = 1 ]; then
        echo "  !! cell $NAME rep $REP produced NO CAPTURE. Cold-start failure"
        echo "  !! mode, not a result. Retrying ONCE before giving up."
      else
        echo "  !! cell $NAME rep $REP produced NO CAPTURE on either attempt."
        echo "  !! That rep is LOST for every cell's pairing -- note it."
      fi
    done
  done
done

# ---- ANALYSIS --------------------------------------------------------------

echo
echo "==========================================================="
echo " ANALYSIS -- P-V-1 FIRST, then P-V-2 (apex) before P-V-3 (flight)"
echo "==========================================================="
echo
echo "-- P-V-1a: engagement. k_yaw must read 0.0000 on EVERY cell ----------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  echo "  [$NAME]"
  grep -h -o "ATTITUDE GAINS.*" "$BASE/$NAME"/ctl_run*.log 2>/dev/null | sort -u
  grep -h -o "LEG-FRAME GAINS.*k_flight=[0-9.]*.*" "$BASE/$NAME"/ctl_run*.log 2>/dev/null | sort -u | head -2
done
echo
echo "-- P-V-1b + P-V-3 + P-V-6: flight, per run, with validity ------------"
echo "-- flight_vs_camber owns P-V-1's exclusion accounting.               --"
python3 "$DIAG/flight_vs_camber.py" --campaign "$BASE" --label lambda \
  --kflight 7150 --csv /tmp/camber_lambda.csv
echo
echo "-- P-V-2 + P-V-5: THE PRIMARY BARS. Apex and ballistic-while-down ----"
echo "-- These do not use the contact flag except to segment. Apex is the  --"
echo "-- signal that is NOT under suspicion -- see S175 S3.                --"
BAL_ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && BAL_ARGS="$BAL_ARGS --ballistic $BASE/$NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/flight_vs_camber.py" $BAL_ARGS
echo
echo "-- P-V-6 support: variance decomposition per cell --------------------"
VAR_ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && VAR_ARGS="$VAR_ARGS --variance $BASE/$NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/flight_vs_camber.py" $VAR_ARGS
echo
echo "-- gait identity, debounced, on the shipped tool ---------------------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  python3 "$DIAG/gait_mode.py" --dir "$BASE/$NAME" 2>&1 | tail -8
done
echo
echo "-- camber delivery: is the channel still doing what S172 measured? ---"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"; REST="${CELL#*:}"
  LAM="${REST%%:*}"; REST="${REST#*:}"
  DIR="${REST%%:*}"; LDEG="${REST#*:}"
  echo "  == $NAME (cmd L/R = $LDEG deg) =="
  for n in $(seq 1 "$NPER"); do
    [ -f "$BASE/$NAME/run$n.csv" ] || continue
    python3 "$DIAG/audit_gamma_decomp.py" --torque-csv "$BASE/$NAME/run$n.csv" \
      --odom-csv "$BASE/$NAME/odom_run$n.csv" \
      --gamma-in "$LDEG" --gamma-out "$LDEG" --gamma-dir "$DIR" 2>&1 \
      | grep -E "L/R|F/R|gate FAIL" | sed 's/^/    /'
  done
done
echo
echo "-- signed roll, paired --------------------------------------------------"
ARGS=""
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  [ -d "$BASE/$NAME" ] && ARGS="$ARGS --dir $BASE/$NAME --label $NAME"
done
# shellcheck disable=SC2086
python3 "$DIAG/body_attitude.py" $ARGS
echo
echo "-- CONTEXT ONLY, NOT A BAR: speed / straightness / yaw rate ----------"
for CELL in $CELLS; do
  NAME="${CELL%%:*}"
  python3 "$DIAG/speed_from_odom.py" --dir "$BASE/$NAME" || \
    echo "  ($NAME: speed_from_odom non-zero -- usually NO odom capture.)"
done
echo
echo "Done. Score P-V-1 FIRST, then P-V-2 (apex) BEFORE P-V-3 (flight), then"
echo "read P-V-4's discriminator table. S176 owns the bars."
