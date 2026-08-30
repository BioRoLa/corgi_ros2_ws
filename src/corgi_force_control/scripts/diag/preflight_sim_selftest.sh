#!/bin/bash
# Self-test for preflight_sim.sh, driving every branch through faked probes.
# Same reasoning as preflight_plant_selftest.sh: a preflight that refuses
# wrongly does not get fixed, it gets routed around, and then it guards nothing.
# Touches no processes and never starts a simulator.
#   bash preflight_sim_selftest.sh

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PASS=0; FAIL=0

run() {
  # $1 label, $2 expected rc, rest: env assignments
  local label="$1" want="$2"; shift 2
  local out rc
  out=$(env PREFLIGHT_SIM_SELFTEST=1 "$@" \
        bash -c ". $HERE/preflight_sim.sh; preflight_sim" 2>&1)
  rc=$?
  LAST="$out"
  if [ "$rc" = "$want" ]; then echo "  PASS  $label (rc=$rc)"; PASS=$((PASS+1))
  else echo "  FAIL  $label -- want rc=$want got rc=$rc"; echo "$out" | sed 's/^/        /'
       FAIL=$((FAIL+1)); fi
}
says() {
  case "$LAST" in
    *"$1"*) echo "  PASS  ...says '$1'"; PASS=$((PASS+1)) ;;
    *) echo "  FAIL  ...expected '$1'"; echo "$LAST" | sed 's/^/        /'; FAIL=$((FAIL+1)) ;;
  esac
}

CLEAR="PREFLIGHT_SIM_FAKE_STALE=- PREFLIGHT_SIM_FAKE_FOREIGN=- PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_LOAD=0.42"

echo "== 1. everything clear -> ACCEPT =="
run "all clear" 0 $CLEAR
says "stale-launch check clean"
says "no Webots running"
says "load average 0.42"
says "PROBES ARE FAKED"

echo "== 2. a stale Corgi launch -> REFUSE (never kill: the sim is shared) =="
run "stale process" 1 PREFLIGHT_SIM_FAKE_STALE="4242 python3 Corgi_launch.py" \
    PREFLIGHT_SIM_FAKE_FOREIGN=- PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_LOAD=0.4
says "ASK before killing anything"

echo "== 3. a Linux-side foreign Webots -> REFUSE (S161's confound) =="
run "foreign linux webots" 1 PREFLIGHT_SIM_FAKE_STALE=- \
    PREFLIGHT_SIM_FAKE_FOREIGN="991 /usr/local/webots/webots" \
    PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_LOAD=0.4
says "NOT the Corgi sim"

echo "== 4. a WINDOWS-side webots.exe -> REFUSE (S171 run 1, the whole point) =="
run "windows webots" 1 PREFLIGHT_SIM_FAKE_STALE=- PREFLIGHT_SIM_FAKE_FOREIGN=- \
    PREFLIGHT_SIM_FAKE_WINWB=2 PREFLIGHT_SIM_FAKE_LOAD=0.4
says "NO WSL pgrep can see it"

echo "== 5. windows probe returns junk -> CONTINUE, but say so =="
run "windows probe inconclusive" 0 PREFLIGHT_SIM_FAKE_STALE=- PREFLIGHT_SIM_FAKE_FOREIGN=- \
    PREFLIGHT_SIM_FAKE_WINWB="ObjectNotFound" PREFLIGHT_SIM_FAKE_LOAD=0.4
says "INCONCLUSIVE"

echo "== 6. load above the limit -> REFUSE =="
run "load too high" 1 PREFLIGHT_SIM_FAKE_STALE=- PREFLIGHT_SIM_FAKE_FOREIGN=- \
    PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_LOAD=9.7
says "WALL-CLOCK"

echo "== 7. the same load under a raised CORGI_MAX_LOAD -> ACCEPT =="
run "load limit raised" 0 PREFLIGHT_SIM_FAKE_STALE=- PREFLIGHT_SIM_FAKE_FOREIGN=- \
    PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_LOAD=9.7 CORGI_MAX_LOAD=12.0
says "load average 9.7 (limit 12.0)"

echo "== 8. boundary: load exactly at the limit is NOT over it -> ACCEPT =="
run "load exactly at limit" 0 PREFLIGHT_SIM_FAKE_STALE=- PREFLIGHT_SIM_FAKE_FOREIGN=- \
    PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_LOAD=4.0

echo "== 9. WITHOUT the selftest flag the fakes are IGNORED (no silent bypass) =="
OUT=$(env PREFLIGHT_SIM_FAKE_WINWB=0 PREFLIGHT_SIM_FAKE_STALE=- \
      bash -c ". $HERE/preflight_sim.sh; preflight_sim_webots" 2>&1)
case "$OUT" in
  *"PROBES ARE FAKED"*) echo "  FAIL  fakes honoured without the flag"; FAIL=$((FAIL+1)) ;;
  *) echo "  PASS  fakes ignored unless PREFLIGHT_SIM_SELFTEST=1"; PASS=$((PASS+1)) ;;
esac

echo
echo "==== $PASS passed, $FAIL failed ===="
[ "$FAIL" = 0 ]
