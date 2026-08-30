#!/bin/bash
# Self-test for preflight_plant.sh, on planted fixtures with known answers.
# Per the project's standing rule (open issue notes / gate-analysers): a check
# that gates every campaign must be shown to pass what it should pass and
# refuse what it should refuse, BEFORE it is trusted. A guard that refuses
# wrongly is worse than no guard -- it trains people to set the override.
#
# Builds throwaway repos under /tmp. Touches nothing in corgi_ws.
#   bash preflight_plant_selftest.sh

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
. "$HERE/preflight_plant.sh" || exit 1

ROOT=/tmp/preflight_plant_selftest
PASS=0
FAIL=0

mkfixture() {
  # $1 = fixture name. Builds ws/ with a parent repo and a corgi_sim "submodule".
  local ws="$ROOT/$1/ws"
  rm -rf "$ROOT/$1"
  mkdir -p "$ws/src/corgi_sim/protos" "$ws/install/corgi_sim/share/corgi_sim/protos"
  ( cd "$ws/src/corgi_sim"
    git init -q .
    printf 'version https://git-lfs.github.com/spec/v1\noid sha256:%s\nsize 42\n' \
      d27d81f9a316eba265b06bbfd272fbc2ff0393b7848c4528ddec69fffbb6a6db \
      > protos/CorgiRobotABAD.proto
    git add -A && git commit -qm "fixture plant" ) > /dev/null 2>&1
  cp "$ws/src/corgi_sim/protos/CorgiRobotABAD.proto" \
     "$ws/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
  ( cd "$ws"
    git init -q .
    local sha; sha=$(git -C src/corgi_sim rev-parse HEAD)
    printf 'placeholder\n' > README
    git add README
    git update-index --add --cacheinfo 160000,"$sha",src/corgi_sim
    git commit -qm "fixture parent" ) > /dev/null 2>&1
  echo "$ws"
}

check() {
  # $1 = label, $2 = expected rc, $3 = ws, $4.. = extra env assignments
  local label="$1" want="$2" ws="$3"; shift 3
  local out rc
  out=$(env "$@" bash -c '. '"$HERE"'/preflight_plant.sh; preflight_plant "$1"' _ "$ws" 2>&1)
  rc=$?
  if [ "$rc" = "$want" ]; then
    echo "  PASS  $label (rc=$rc)"; PASS=$((PASS+1))
  else
    echo "  FAIL  $label -- expected rc=$want, got rc=$rc"; echo "$out" | sed 's/^/        /'
    FAIL=$((FAIL+1))
  fi
  LAST_OUT="$out"
}

expect_says() {
  case "$LAST_OUT" in
    *"$1"*) echo "  PASS  ...and it says '$1'"; PASS=$((PASS+1)) ;;
    *) echo "  FAIL  ...expected output to mention '$1'"; echo "$LAST_OUT" | sed 's/^/        /'
       FAIL=$((FAIL+1)) ;;
  esac
}

echo "== 1. clean plant, install == src -> ACCEPT =="
WS1=$(mkfixture clean)
check "clean plant accepted" 0 "$WS1"
expect_says "PLANT: corgi_sim"
expect_says "install matches src"

echo "== 2. proto modified and uncommitted -> REFUSE (the S198 case) =="
WS2=$(mkfixture dirty)
echo "tampered" >> "$WS2/src/corgi_sim/protos/CorgiRobotABAD.proto"
cp "$WS2/src/corgi_sim/protos/CorgiRobotABAD.proto" \
   "$WS2/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
check "dirty proto refused" 1 "$WS2"
expect_says "THE WEBOTS MODEL IS MODIFIED"

echo "== 3. same, with the deliberate override -> ACCEPT, loudly =="
check "dirty proto overridden" 0 "$WS2" CORGI_ALLOW_DIRTY_PLANT=1
expect_says "PROCEEDING ON AN UNRECORDED PLANT"

echo "== 4. src rebuilt but install stale -> REFUSE =="
WS4=$(mkfixture staleinstall)
echo "only in install" >> "$WS4/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
check "stale install refused" 1 "$WS4"
expect_says "install/ and src/ disagree"

echo "== 5. no installed proto -> ACCEPT, but say so =="
WS5=$(mkfixture noinstall)
rm -f "$WS5/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
check "missing install tolerated" 0 "$WS5"
expect_says "cannot verify src vs install"

echo "== 6. parent pointer stale -> ACCEPT, with a WARN =="
WS6=$(mkfixture stalepointer)
( cd "$WS6/src/corgi_sim"; echo x >> protos/CorgiRobotABAD.proto
  git add -A; git commit -qm "advance the plant" ) > /dev/null 2>&1
cp "$WS6/src/corgi_sim/protos/CorgiRobotABAD.proto" \
   "$WS6/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
check "stale parent pointer warns only" 0 "$WS6"
expect_says "parent records src/corgi_sim"

echo "== 7. not a git checkout -> REFUSE =="
WS7=$(mkfixture notgit)
rm -rf "$WS7/src/corgi_sim/.git"
check "non-repo refused" 1 "$WS7"
expect_says "not a git checkout"

# ---------------------------------------------------------------------------
# THE MID-CAMPAIGN LOCK (S204). Fixtures get all four installed artifacts, with
# the controller as a SYMLINK into build/ -- because that is what colcon's
# --symlink-install produces and it is the case a naive `stat` gets wrong.

mkinstall() {
  # $1 = ws from mkfixture. Adds the four artifacts plant_fingerprint watches.
  local ws="$1"
  mkdir -p "$ws/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg" \
           "$ws/install/corgi_force_control/lib/corgi_force_control" \
           "$ws/build/corgi_force_control/src"
  echo "driver" > "$ws/install/corgi_sim/local/lib/python3.10/dist-packages/corgi_driver_pkg/corgi_driver.py"
  echo "controller v1" > "$ws/build/corgi_force_control/src/gslip_pronk_node"
  echo "forcectl v1"   > "$ws/build/corgi_force_control/src/force_control_node"
  ln -sf "$ws/build/corgi_force_control/src/gslip_pronk_node" \
         "$ws/install/corgi_force_control/lib/corgi_force_control/gslip_pronk_node"
  ln -sf "$ws/build/corgi_force_control/src/force_control_node" \
         "$ws/install/corgi_force_control/lib/corgi_force_control/force_control_node"
}

lockrun() {
  # $1 label, $2 expected rc, $3 ws, $4 lock value ('-' = unset), $5 OUTDIR ('' = none)
  local label="$1" want="$2" ws="$3" lock="$4" outdir="$5"
  local out rc
  # export, do NOT `env $var ...` -- an unquoted assignment word-splits on
  # the fingerprint, which is how this harness "failed" a case the function
  # got right. The real path exports, so the real path was never broken.
  if [ "$lock" = "-" ]; then unset CORGI_PLANT_LOCK
  else export CORGI_PLANT_LOCK="$lock"; fi
  out=$(OUTDIR="$outdir" bash -c \
        ". $HERE/preflight_plant.sh; plant_verify \"\$1\"" _ "$ws" 2>&1)
  rc=$?
  unset CORGI_PLANT_LOCK
  LAST_OUT="$out"
  if [ "$rc" = "$want" ]; then echo "  PASS  $label (rc=$rc)"; PASS=$((PASS+1))
  else echo "  FAIL  $label -- want rc=$want got rc=$rc"; echo "$out" | sed 's/^/        /'
       FAIL=$((FAIL+1)); fi
}

echo "== 8. freeze then verify with nothing changed -> ACCEPT =="
WS8=$(mkfixture lock_clean); mkinstall "$WS8"
LOCK8=$(bash -c ". $HERE/preflight_plant.sh; plant_fingerprint \"\$1\"" _ "$WS8")
lockrun "unchanged plant" 0 "$WS8" "$LOCK8" ""

echo "== 9. the PROTO is rebuilt mid-campaign -> REFUSE, and name it =="
touch -d '2030-01-01 00:00:00' "$WS8/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
lockrun "proto moved" 1 "$WS8" "$LOCK8" ""
expect_says "proto  CHANGED"
expect_says "THE PLANT MOVED MID-CAMPAIGN"

echo "== 10. THE REGRESSION TEST: the controller SYMLINK's TARGET changes =="
echo "       (S168's case. Without stat -L this compares two constants forever.)"
WS10=$(mkfixture lock_symlink); mkinstall "$WS10"
LOCK10=$(bash -c ". $HERE/preflight_plant.sh; plant_fingerprint \"\$1\"" _ "$WS10")
# rebuild the BINARY; the symlink itself is untouched, exactly as colcon leaves it
echo "controller v2 -- rebuilt, bigger" > "$WS10/build/corgi_force_control/src/gslip_pronk_node"
touch -d '2030-01-01 00:00:00' "$WS10/build/corgi_force_control/src/gslip_pronk_node"
lockrun "controller rebuilt behind the symlink" 1 "$WS10" "$LOCK10" ""
expect_says "controller  CHANGED"

echo "== 11. no lock set (standalone run) -> ACCEPT, but announce the plant =="
WS11=$(mkfixture lock_none); mkinstall "$WS11"
lockrun "no lock set" 0 "$WS11" "-" ""
expect_says "(none set)"

echo "== 12. a missing artifact is reported, not silently skipped =="
WS12=$(mkfixture lock_missing); mkinstall "$WS12"
rm -f "$WS12/install/corgi_force_control/lib/corgi_force_control/force_control_node"
FP12=$(bash -c ". $HERE/preflight_plant.sh; plant_fingerprint \"\$1\"" _ "$WS12")
case "$FP12" in
  *"force_ctl=MISSING"*) echo "  PASS  missing artifact reported as MISSING"; PASS=$((PASS+1)) ;;
  *) echo "  FAIL  expected force_ctl=MISSING, got: $FP12"; FAIL=$((FAIL+1)) ;;
esac

echo "== 13. the evidence is written NEXT TO THE CAPTURES, not just to stdout =="
WS13=$(mkfixture lock_marker); mkinstall "$WS13"
LOCK13=$(bash -c ". $HERE/preflight_plant.sh; plant_fingerprint \"\$1\"" _ "$WS13")
OUT13="$ROOT/lock_marker/captures"; mkdir -p "$OUT13"
touch -d '2030-01-01 00:00:00' "$WS13/install/corgi_sim/share/corgi_sim/protos/CorgiRobotABAD.proto"
lockrun "marker file written" 1 "$WS13" "$LOCK13" "$OUT13"
if [ -f "$OUT13/PLANT_CHANGED_MID_CAMPAIGN.txt" ]; then
  echo "  PASS  PLANT_CHANGED_MID_CAMPAIGN.txt landed beside the captures"; PASS=$((PASS+1))
else
  echo "  FAIL  no marker file in $OUT13"; FAIL=$((FAIL+1))
fi

echo
echo "==== $PASS passed, $FAIL failed ===="
rm -rf "$ROOT"
[ "$FAIL" = 0 ]
