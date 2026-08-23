#!/bin/bash
# P-Y-1..P-Y-5: the contact's LATERAL migration under camber. Log S188.
#
# STAGE 1's SKIPPED ITEM 2. Stage 1's validation plan listed "static contact
# lateral offset vs r sin lambda" and it was never run -- Stage 1 measured
# contact HEIGHT only (SS65-73). S183 then corrected side_geometry's crown term
# from R_CORNER (0.015) to the rolling radius (0.145), a ~20x change to the term
# the thesis claims as its contribution, and discovered the arbiter it assumed
# existed does not. This is that arbiter.
#
# THREE CANDIDATES, migration relative to lambda = 0, hip-to-contact, mm:
#
#   lambda        0+       5      10      15      20      30
#   LegWheel now   0   +12.29  +23.79  +34.41  +44.06  +60.22   d0*cos + r*sin
#   pre-S183       0    +0.96   +1.21   +0.76   -0.40   -4.78   d0*cos + r_c*sin
#   C++ map    -20.0    -7.64   +4.09  +15.09  +25.27  +42.90   (d0-wt/2)*cos + r*sin
#
# The C++ form is LegWheel's corrected form MINUS the wheel-edge offset
# (wt/2)*cos(lambda) ~ 20 mm. So S183 moved LegWheel toward the C++ and
# overshot by exactly that one term, and the hypothesis space is two BINARY
# questions: is the swing coefficient r or r_corner, and is the wheel-edge
# offset real. Spread is 20-65 mm across the grid -- far above noise.
#
# WHY WHEELED MODE. camber_roll.py:73-77 -- theta = 17 deg is "the pose where
# the leg-wheel is genuinely a wheel on its own axle, R(alpha) constant at
# 0.145 m across the whole rim... the one place the old 0.145 m rolling-radius
# constant is correct". If r*sin(lambda) is ever right, it is right here.
#
# WHY --beta-rate 0. It makes the "roll" phase a STATIC HOLD: the schedule
# still settles the lean first, then holds motionless for ROLL_TIME. Verified
# by dry-run: "beta monotonic, sweeps 0 deg = 0.00 revolutions". No script has
# ever used it.
#
# ⚠⚠ THE BODY ROLLS AND by IS IN THE BODY FRAME. The lr pattern produces ~9 deg
# of body roll at lambda 20, and 9 deg across a 0.145 m lever is ~23 mm --
# COMPARABLE TO THE WHOLE EFFECT. Raw `by` conflates contact migration with
# body roll and cannot separate the candidates. The analyser MUST de-roll using
# the odom quaternion (record_camber.py stores it). This script only captures;
# the de-rolling is the analyser's job and is the load-bearing step.
#
# No `set -u` -- camber_cycle sources ROS setup files.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$HERE/preflight_plant.sh"
preflight_plant || exit 1
# IS THE SIMULATOR FREE, AND QUIET? S203. Same one-file treatment as the plant
# guard above, for the same reason: these checks spread by copy-paste and only
# reached 7 of 25 campaigns (the WINDOWS-side webots.exe one) to 11 of 25 (the
# stale-launch one), and the variants disagreed about what to grep. This is the
# union. Self-tested: preflight_sim_selftest.sh, 9 faked-probe cases.
. "$HERE/preflight_sim.sh"
preflight_sim || exit 1

LAMS=${LAMS:-"0 5 10 15 20 30"}
KP=${KP:-90}
export ROLL_TIME=${ROLL_TIME:-20}
export FOLD_SETTLE=${FOLD_SETTLE:-3}
OUT=${OUT:-/home/alexc/corgi_runs/contact_lateral}

echo "==========================================================="
echo " CONTACT LATERAL MIGRATION -- P-Y-1..P-Y-5 (log S188)"
echo "==========================================================="
echo " grid     : lambda = $LAMS deg, pattern lr, kp_h $KP"
echo " rig      : WHEELED mode (theta 17 deg), STATIC hold (--beta-rate 0)"
echo " hold     : ${ROLL_TIME}s settled, after fold_settle ${FOLD_SETTLE}s"
echo " capture  : CORGI_CONTACT_DEBUG=1, per-lambda CSV in $OUT"
echo
echo " BARS, registered in S188 before this ran and binding (S126):"
echo "   P-Y-1  VALIDITY, scored FIRST, one consequence. Each lambda: >= 200"
echo "          static-window samples with all four modules present, achieved"
echo "          lambda within 3 deg of commanded on BOTH sides, and a lambda=0"
echo "          control from this session. A failing lambda is EXCLUDED AND"
echo "          COUNTED."
echo "   P-Y-2  THE DISCRIMINATOR. Median DE-ROLLED delta(lambda) over"
echo "          {10,15,20,30} is closer to exactly one candidate than to the"
echo "          other two by a factor >= 2 in summed absolute residual."
echo "          Falsified if none wins by that margin -- the truth would be a"
echo "          fourth form, which is a finding, not a failure."
echo "   P-Y-3  THE EDGE STEP, and the cleanest single test: |d(5)-d(0)| > 10"
echo "          mm if the wheel-edge offset is real, < 5 mm if it is not."
echo "          A SHAPE test near zero, where the two swing coefficients"
echo "          barely differ."
echo "   P-Y-4  MONOTONICITY over {10,15,20,30}. r*sin and the C++ form both"
echo "          predict it; r_corner*sin does NOT (it peaks near 10 and turns"
echo "          negative by 20). Non-monotone SUPPORTS the pre-S183 form."
echo "   P-Y-5  CONSEQUENCE, declared in advance. r_corner wins -> S183 is"
echo "          WRONG: revert, re-run SS184-186, rewrite the brief's"
echo "          contribution term. C++ form wins -> side_geometry gains the"
echo "          edge offset, S184's grids re-run. Current form wins -> nothing"
echo "          changes."
echo
echo " NOT settled by this: the value of r for the LEGGED template. This is"
echo " wheeled mode, the only regime where r = 0.145 is defined (S21). The"
echo " FORM transfers; the coefficient's legged value is S21's open question."
echo

# ---- PREFLIGHT -------------------------------------------------------------


# The capture is the whole point: refuse if the driver cannot write it.
INST=$(find /home/alexc/corgi_ws/corgi_ros2_ws/install/corgi_sim -name corgi_driver.py 2>/dev/null | head -1)
[ -n "$INST" ] || { echo "!! installed corgi_driver.py not found -- REFUSING"; exit 1; }
grep -q 'CORGI_CONTACT_DEBUG' "$INST" || {
  echo "!! the INSTALLED driver has no CORGI_CONTACT_DEBUG sink. Rebuild:"
  echo "!!   colcon build --packages-select corgi_sim"
  echo "!! Without it every capture is empty and the sweep measures nothing."
  exit 1; }
echo "installed driver carries the contact-debug sink."

# --beta-rate must exist, or the "static" hold silently rolls and every
# lambda averages the contact around the whole rim -- which is exactly the
# measurement this campaign is NOT trying to make.
grep -q '\-\-beta-rate' "$HERE/camber_roll.py" || {
  echo "!! camber_roll.py has no --beta-rate; the hold cannot be made static."
  exit 1; }
echo "camber_roll supports --beta-rate (static hold)."

echo
echo "-- dry run of every commanded trajectory (no ROS, no simulator) --"
FAILS=0
for lam in $LAMS; do
  python3 "$HERE/camber_roll.py" --lam-deg "$lam" --pattern lr --kp-h "$KP" \
      --beta-rate 0 --fold-settle "$FOLD_SETTLE" --dry-run > /tmp/cl_dry_$lam.out 2>&1 \
    || { echo "  !! lam $lam DRY RUN FAILED:"; tail -5 /tmp/cl_dry_$lam.out; FAILS=$((FAILS+1)); }
  grep -q "sweeps 0 deg = 0.00 revolutions" /tmp/cl_dry_$lam.out || {
    echo "  !! lam $lam is NOT static -- beta sweeps. REFUSING."; FAILS=$((FAILS+1)); }
done
[ "$FAILS" = 0 ] || { echo "!! $FAILS dry-run failures. REFUSING."; exit 1; }
echo "all $(echo $LAMS | wc -w) trajectories dry-run clean and STATIC."

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo; echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }
mkdir -p "$OUT"

# ---- RUN -------------------------------------------------------------------

for lam in $LAMS; do
  TAG="clat_lam${lam}_kp${KP}"
  echo
  echo "################################################################"
  echo "###  lambda $lam deg, STATIC wheeled hold, kp_h $KP"
  echo "###  ON THE RENDER: robot folds to wheels, leans, then SITS STILL."
  echo "###  If it rolls away, --beta-rate did not take and the run is void."
  echo "################################################################"
  # Per-lambda path: the sink opens 'w', so one shared path keeps only the
  # LAST run and silently discards the rest of the sweep.
  export CORGI_CONTACT_DEBUG=1
  export CORGI_CONTACT_DEBUG_PATH="$OUT/contact_lam${lam}.csv"
  export CAMBER_EXTRA="--beta-rate 0 --kp-h $KP"
  bash "$HERE/camber_cycle.sh" "$lam" lr "$TAG"
  rc=$?
  ROWS=$(wc -l < "$CORGI_CONTACT_DEBUG_PATH" 2>/dev/null || echo 0)
  echo "  lam $lam: camber_cycle rc=$rc, contact rows $ROWS"
  if [ "$ROWS" -lt 200 ]; then
    echo "  !! lam $lam captured only $ROWS contact rows (<200). P-Y-1 excludes"
    echo "  !! this lambda. Check the driver logged '[contact debug] writing'."
  fi
done

echo
echo "==========================================================="
echo " captures"
echo "==========================================================="
ls -la "$OUT"/ 2>/dev/null
echo
echo "Analyse with contact_lateral.py (de-rolls using the odom quaternion --"
echo "raw body-frame by conflates contact migration with ~9 deg of body roll,"
echo "which is ~23 mm at lambda 20 and comparable to the whole effect)."
