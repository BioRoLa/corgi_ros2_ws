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

echo
echo "==== $PASS passed, $FAIL failed ===="
rm -rf "$ROOT"
[ "$FAIL" = 0 ]
