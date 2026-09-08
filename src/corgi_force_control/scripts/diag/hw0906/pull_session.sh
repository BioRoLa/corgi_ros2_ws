#!/usr/bin/env bash
# Pull one hardware session off the Orin and the Vicon PC into ~/corgi_runs/hw_<DATE>/.
#
#   bash pull_session.sh                # today
#   bash pull_session.sh 2026-09-06     # a named session
#   VICON_HOURS=36 bash pull_session.sh 2026-09-06
#
# Idempotent: rsync skips what is already local, and Vicon files are only fetched
# when missing, so re-running after a dropped connection costs nothing. Safe to
# run while the Orin is still recording -- rsync copies what exists now.
#
# Generalised from backup_all.sh, the script that pulled the 2026-09-06 session
# (13 GB). See Reference/"Data Retrieval - Orin and Vicon PC" in the vault.
set -u

DATE="${1:-$(date +%F)}"
H="${ORIN_HOST:-biorola@192.168.30.244}"    # Ubuntu, has rsync
VH="${VICON_HOST:-Biorolab@192.168.30.104}" # Windows, scp + PowerShell only
HOURS="${VICON_HOURS:-24}"                  # how far back to look on the Vicon PC
D="$HOME/corgi_runs/hw_$DATE"

SSH_OPTS=(-n -o BatchMode=yes -o ConnectTimeout=15)
mkdir -p "$D"/{bags,logs,config,vicon,vicon_raw,orin_misc}
echo "session $DATE  ->  $D"

# ---------------------------------------------------------------- reachability
for pair in "Orin:$H" "Vicon:$VH"; do
  name="${pair%%:*}"; host="${pair#*:}"
  if ssh "${SSH_OPTS[@]}" "$host" "exit" 2>/dev/null; then
    echo "  $name  $host  OK"
  else
    echo "  $name  $host  UNREACHABLE -- its section will be skipped"
  fi
done

# ---------------------------------------------------------------- 1. the Orin
if ssh "${SSH_OPTS[@]}" "$H" "exit" 2>/dev/null; then
  echo
  echo "=== 1. bags ==="
  rsync -a --info=stats1 "$H:corgi_ws/corgi_ros2_ws/bag/" "$D/bags/" 2>&1 | tail -n 3
  echo "    local bags: $(ls -1 "$D/bags" 2>/dev/null | wc -l)"

  echo "=== 2. controller logs and CSV captures ==="
  rsync -a --info=stats1 "$H:corgi_ws/corgi_ros2_ws/output_data/" "$D/logs/" 2>&1 | tail -n 3
  echo "    local files: $(ls -1 "$D/logs" 2>/dev/null | wc -l)"

  echo "=== 3. motor config dumps (config panel) ==="
  rsync -a "$H:corgi_ws/corgi_ros2_ws/log_file/" "$D/config/" 2>/dev/null
  echo "    local files: $(ls -1 "$D/config" 2>/dev/null | wc -l)"

  echo "=== 4. crash instruments (#50) ==="
  # hb.sh -> orin_heartbeat.txt, rail.sh -> orin_rail.txt, watch_boot.sh -> tegra_watch_*.txt
  rsync -a "$H:orin_heartbeat.txt" "$H:orin_rail.txt" "$D/orin_misc/" 2>/dev/null
  rsync -a "$H:tegra_watch_*.txt" "$D/orin_misc/" 2>/dev/null
  rsync -a "$H:hb.sh" "$H:rail.sh" "$H:watch_boot.sh" "$D/orin_misc/" 2>/dev/null
  echo "    local files: $(ls -1 "$D/orin_misc" 2>/dev/null | wc -l)"
else
  echo; echo "=== Orin skipped (unreachable) ==="
fi

# ------------------------------------------------------------- 2. the Vicon PC
# Windows: no rsync. Ask PowerShell for FULL paths of recent captures, then scp
# each one. Full paths matter -- Nexus nests sessions as F:\R14\<date>\<date>\<date>
# and that nesting is not guaranteed to stay the same.
vicon_pull () {          # $1 = comma-separated -Include list, $2 = destination
  local inc="$1" dest="$2" n=0
  local ps="Get-ChildItem -Path F:\\R14 -Recurse -File -Include $inc -ErrorAction SilentlyContinue |
            Where-Object { \$_.LastWriteTime -gt (Get-Date).AddHours(-$HOURS) } |
            ForEach-Object { \$_.FullName }"
  while IFS= read -r p; do
    p="${p%$'\r'}"; [ -n "$p" ] || continue
    local f="${p##*\\}"
    [ -f "$dest/$f" ] && continue
    if scp -q -o BatchMode=yes "$VH:\"${p//\\//}\"" "$dest/" 2>/dev/null; then
      echo "    + $f"; n=$((n+1))
    else
      echo "    ! FAILED $f"
    fi
  done < <(ssh "${SSH_OPTS[@]}" "$VH" "powershell -NoProfile -Command \"$ps\"" 2>/dev/null)
  echo "    fetched $n, local total $(ls -1 "$dest" 2>/dev/null | wc -l)"
}

if ssh "${SSH_OPTS[@]}" "$VH" "exit" 2>/dev/null; then
  echo
  echo "=== 5. processed Vicon captures (last ${HOURS}h) ==="
  vicon_pull "*.c3d,*.xcp,*.vsk,*.mp,*.system" "$D/vicon"
  echo "=== 6. RAW Vicon captures (large; c3d rebuilds from these) ==="
  vicon_pull "*.x2d,*.x1d" "$D/vicon_raw"
else
  echo; echo "=== Vicon PC skipped (unreachable) ==="
fi

# ---------------------------------------------------------------- 3. verify
echo
echo "=== TOTAL ==="
du -sh "$D" 2>/dev/null
du -sh "$D"/* 2>/dev/null
echo
echo "=== bags that a reset left malformed (no metadata.yaml) ==="
bad=0
for b in "$D"/bags/*/; do
  [ -d "$b" ] || continue
  [ -f "$b/metadata.yaml" ] || { echo "    ! ${b##*/bags/}"; bad=$((bad+1)); }
done
[ "$bad" -eq 0 ] && echo "    none" || echo "    $bad -- rebuild with scripts/diag/carve_bag.py"
