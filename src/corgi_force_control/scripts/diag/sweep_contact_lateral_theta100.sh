#!/usr/bin/env bash
# CONTACT POSITION AT THE RUNNING LEG ANGLE, 1 deg in lambda across the kappa
# step -- P-C-1..P-C-4 (log S223).
#
# sweep_contact_lateral.sh's rig (camber_cycle -> camber_roll --beta-rate 0,
# CORGI_CONTACT_DEBUG per-lambda CSV) with ONE change: --theta-wheel 100, so
# the "fold" is a no-op (stand -> stand) and the static hold is a STANDING LEAN
# at theta = 100 deg, the pronk's stance angle, on the FOOT ARC the pronk rolls
# on. S222 showed migration is smooth at 5 deg on the wheel-mode rim; S218's
# kappa(lambda) steps between 10 and 12 deg on the running pronk. This asks
# whether the CONTACT steps there, at 1 deg, on the right rim.
#
#   REGISTERED_SECTION=223 bash sweep_contact_lateral_theta100.sh
#   WAIT_PID=<pid>   block until that process exits before touching the sim
#                    (camber_cycle's teardown kills ANY running campaign)
#
# No `set -u` -- camber_cycle sources ROS setup files.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -n "${REGISTERED_SECTION:-}" ] || {
  echo "!! REGISTERED_SECTION is unset. This campaign runs only under a"
  echo "!! registered log section (S126). REFUSING."; exit 1; }

LAMS=${LAMS:-"0 8 9 10 11 12 13 14 15"}
KP=${KP:-90}
THETA=${THETA:-100}
export ROLL_TIME=${ROLL_TIME:-20}
export FOLD_SETTLE=${FOLD_SETTLE:-3}
OUT=${OUT:-/home/alexc/corgi_runs/contact_lateral_theta100}

echo "==========================================================="
echo " CONTACT POSITION AT THETA $THETA -- P-C-1..P-C-4 (log S$REGISTERED_SECTION)"
echo "==========================================================="
echo " grid     : lambda = $LAMS deg, pattern lr, kp_h $KP"
echo " rig      : STANDING lean at theta $THETA deg, STATIC hold (--beta-rate 0)"
echo " hold     : ${ROLL_TIME}s settled, after fold_settle ${FOLD_SETTLE}s"
echo " capture  : CORGI_CONTACT_DEBUG=1, per-lambda CSV in $OUT"
echo
echo " BARS, registered in S$REGISTERED_SECTION before this ran and binding (S126):"
echo "   P-C-1  VALIDITY first: >= 200 tail samples, all four modules; lambda=0"
echo "          control SE(median) <= 0.2 mm. Failing lambda EXCLUDED AND COUNTED."
echo "   P-C-2  THE QUESTION: any 1-deg increment of |d_by| > 3x that module's"
echo "          median increment AND > 3 mm -> contact-position step PRESENT;"
echo "          else ABSENT at 1 deg on the foot arc. Either is a result."
echo "   P-C-3  |d_by| non-decreasing over 8..15 on all four modules."
echo "   P-C-4  PRESENT -> mesh facet promoted to leading candidate for S218."
echo "          ABSENT  -> static-contact candidate excluded; dynamic threshold"
echo "          becomes leading. Hardware sweep stays decisive either way."
echo

# ---- PREFLIGHT (as sweep_contact_lateral.sh) ---------------------------------
INST=$(find /home/alexc/corgi_ws/corgi_ros2_ws/install/corgi_sim -name corgi_driver.py 2>/dev/null | head -1)
[ -n "$INST" ] || { echo "!! installed corgi_driver.py not found -- REFUSING"; exit 1; }
grep -q 'CORGI_CONTACT_DEBUG' "$INST" || {
  echo "!! the INSTALLED driver has no CORGI_CONTACT_DEBUG sink. REFUSING."; exit 1; }
echo "installed driver carries the contact-debug sink."
grep -q '\-\-beta-rate' "$HERE/camber_roll.py" || { echo "!! no --beta-rate. REFUSING."; exit 1; }
grep -q '\-\-theta-wheel' "$HERE/camber_roll.py" || { echo "!! no --theta-wheel. REFUSING."; exit 1; }
echo "camber_roll supports --beta-rate and --theta-wheel."

echo
echo "-- dry run of every commanded trajectory (no ROS, no simulator) --"
FAILS=0
for lam in $LAMS; do
  python3 "$HERE/camber_roll.py" --lam-deg "$lam" --pattern lr --kp-h "$KP" \
      --beta-rate 0 --fold-settle "$FOLD_SETTLE" --theta-wheel "$THETA" \
      --dry-run > /tmp/clt_dry_$lam.out 2>&1 \
    || { echo "  !! lam $lam DRY RUN FAILED:"; tail -5 /tmp/clt_dry_$lam.out; FAILS=$((FAILS+1)); }
  grep -q "sweeps 0 deg = 0.00 revolutions" /tmp/clt_dry_$lam.out || {
    echo "  !! lam $lam is NOT static -- beta sweeps. REFUSING."; FAILS=$((FAILS+1)); }
  grep -q "theta ends at ${THETA}.00 deg" /tmp/clt_dry_$lam.out || {
    echo "  !! lam $lam does not end at theta $THETA. REFUSING."; FAILS=$((FAILS+1)); }
done
[ "$FAILS" = 0 ] || { echo "!! $FAILS dry-run failures. REFUSING."; exit 1; }
echo "all $(echo $LAMS | wc -w) trajectories dry-run clean, STATIC, at theta $THETA."

[ -n "${PREFLIGHT_ONLY:-}" ] && { echo; echo "PREFLIGHT_ONLY -- all gates passed."; exit 0; }
mkdir -p "$OUT"

# ---- WAIT for a running campaign to finish --------------------------------
if [ -n "${WAIT_PID:-}" ]; then
  echo
  echo "waiting for PID $WAIT_PID ($(ps -o cmd= -p "$WAIT_PID" 2>/dev/null | cut -c1-80)) to exit..."
  while kill -0 "$WAIT_PID" 2>/dev/null; do sleep 30; done
  echo "PID $WAIT_PID gone at $(date). Giving its teardown 60 s."
  sleep 60
fi
if pgrep -af "sweep_[a-z_]*\.sh|gslip_pron[k]_node" | grep -v "$$" | grep -v theta100 >/dev/null; then
  echo "!! another campaign is still running:"; pgrep -af "sweep_[a-z_]*\.sh|gslip_pron[k]_node" | grep -v theta100
  echo "!! REFUSING to tear it down."; exit 1
fi

# ---- RUN -------------------------------------------------------------------
echo "run started $(date)"
for lam in $LAMS; do
  TAG="clt${THETA}_lam${lam}_kp${KP}"
  echo
  echo "################################################################"
  echo "###  lambda $lam deg, STATIC standing lean at theta $THETA, kp_h $KP"
  echo "###  ON THE RENDER: robot stands at theta $THETA, leans, SITS STILL."
  echo "################################################################"
  export CORGI_CONTACT_DEBUG=1
  export CORGI_CONTACT_DEBUG_PATH="$OUT/contact_lam${lam}.csv"
  export CAMBER_EXTRA="--beta-rate 0 --kp-h $KP --theta-wheel $THETA"
  bash "$HERE/camber_cycle.sh" "$lam" lr "$TAG"
  rc=$?
  ROWS=$(wc -l < "$CORGI_CONTACT_DEBUG_PATH" 2>/dev/null || echo 0)
  echo "  lam $lam: camber_cycle rc=$rc, contact rows $ROWS  ($(date +%H:%M))"
  [ "$ROWS" -lt 200 ] && echo "  !! lam $lam captured only $ROWS rows (<200): P-C-1 excludes it."
done

echo
echo "==========================================================="
echo " captures  ($(date))"
echo "==========================================================="
ls -la "$OUT"/
echo
python3 "$HERE/contact_lateral_fine.py" --dir "$OUT"
