#!/usr/bin/env bash
# The camber-command confirming run, as a matrix.
#
# THE PREDICTION, REGISTERED BEFORE THE RUN. Two predictions have already been
# recorded and falsified on this project (friction helping straight-line
# rolling; torque limiting speed) and both were more useful wrong than a
# post-hoc story would have been right. So:
#
#   lam0   0 deg, no camber      straight. This is the baseline drift that
#                                every other run is measured against.
#   lr10  +10 deg, left/right    curves, R on the order of 0.809 m
#   lr20  +20 deg, left/right    curves, R on the order of 0.372 m
#   lr20n -20 deg, left/right    the mirror of lr20. The differential steering
#                                channel behaved very differently for positive
#                                and negative commands, so a left turn is not
#                                evidence about a right one.
#   fr20  +20 deg, front/rear    NO net curvature -- within the lam0 baseline.
#                                This is a retrodiction: it is precisely why
#                                gamma_correction() never curved the path.
#
# THE GATE IS THE SCALING, NOT THE ABSOLUTE RADIUS. R_pred is one wheel's
# geometry; four of them are bolted to a rigid body on a 0.4234 m contact track
# and turned at a matched beta_dot, so they must fight and the achieved radius
# will exceed the prediction. A constant derating cancels in a ratio:
#
#   R_fit(10 deg) / R_fit(20 deg)  ==  2.17
#
# Sign is RECORDED, not gated: lambda > 0 with {+1,-1,-1,+1} is predicted to
# turn RIGHT (gamma > 0 abducts outward on all four legs, so this pattern
# shifts every contact to +y while the hips stay, leaning each wheel right). If
# it comes out left the convention is inverted -- note it and correct it, do
# not re-derive it after the fact.
#
#   bash sweep_camber.sh [ROLL_TIME]
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# IS THE SIMULATED ROBOT THE RECORDED ROBOT? S198 / open issue #26. The Webots
# proto sat modified-and-uncommitted for six days, and every campaign in that
# window ran on a plant no checkout could rebuild -- `git diff` hid it, because
# it is a 103 MB LFS object and git diff shows only the pointer. This prints the
# plant identity into THIS campaign's own log, and refuses on a dirty or
# unbuilt plant. Self-tested: preflight_plant_selftest.sh, 7 planted cases.
. "$HERE/preflight_plant.sh"
preflight_plant || exit 1
ROLL="${1:-20}"

RUNS=(
  "0    none  lam0"
  "10   lr    lr10"
  "20   lr    lr20"
  "-20  lr    lr20n"
  "20   fr    fr20"
)

DUMPDIR="$HOME/camber_dumps"

for r in "${RUNS[@]}"; do
    set -- $r
    LAM="$1"; PAT="$2"; TAG="$3"
    echo "=== lambda $LAM deg, pattern $PAT -> $DUMPDIR/camber_$TAG.npz"
    ROLL_TIME="$ROLL" bash "$HERE/camber_cycle.sh" "$LAM" "$PAT" "$TAG" \
        2>&1 | tail -30
    echo
done
echo "CAMBER SWEEP DONE"
echo
echo "The scaling test, which is the actual gate:"
echo "  python3 $HERE/check_camber_turn.py $DUMPDIR/camber_lr10.npz --lam-deg 10"
echo "  python3 $HERE/check_camber_turn.py $DUMPDIR/camber_lr20.npz --lam-deg 20"
echo "  R_fit(10)/R_fit(20) should be about 2.17"
