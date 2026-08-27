#!/usr/bin/env bash
# Wait until the simulator has been CONTINUOUSLY idle for IDLE_MIN minutes
# (no campaign sweep, no Webots on either side, no controller nodes, no camber
# rig), then exec the given command. Built so a queued campaign does not steal
# the simulator between two runs of someone else's campaign.
#
#   IDLE_MIN=10 wait_idle_then.sh <command...>
#
# The waiter's OWN command line contains the queued sweep's name, and every
# $(...) forks a copy with a fresh PID, so exclusion is by NAME (wait_idle),
# not by $$ -- the first version excluded by PID and never launched.
BUSY='sweep_[a-z_0-9]*\.sh|webots|gslip_pron[k]|force_contro[l]_node|camber_rol[l]|camber_cycl[e]|record_cambe[r]|Corgi_launc[h]'
IDLE_MIN=${IDLE_MIN:-10}
need=$((IDLE_MIN * 2))   # 30 s polls
idle=0
echo "waiting for $IDLE_MIN min of continuous idle ($(date))"
while [ "$idle" -lt "$need" ]; do
  busy_lin=$(pgrep -af "$BUSY" | grep -vc "wait_idle_the[n]")
  busy_win=$(powershell.exe -NoProfile -Command "(Get-Process webots,webots-bin -ErrorAction SilentlyContinue | Measure-Object).Count" 2>/dev/null | tr -d '\r ')
  case "$busy_win" in ''|*[!0-9]*) busy_win=0 ;; esac
  if [ "$busy_lin" -eq 0 ] && [ "$busy_win" -eq 0 ]; then
    idle=$((idle + 1))
  else
    [ "$idle" -gt 0 ] && echo "busy again at $(date) (lin $busy_lin, win $busy_win); idle counter reset"
    idle=0
  fi
  sleep 30
done
echo "idle for $IDLE_MIN min; launching at $(date): $*"
exec "$@"
