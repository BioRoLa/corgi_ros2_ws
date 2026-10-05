#!/usr/bin/env bash
# Install corgi-orin-monitor as a systemd service that starts at every boot.
# Runs as root straight from the source tree: no colcon build, no ROS environment needed.
#
#   scripts/install_service.sh                        # default log dir /var/log/corgi_orin_monitor
#   scripts/install_service.sh --persistent-journal   # also make journald keep logs across reboots
#   LOG_DIR=/data/orin_monitor scripts/install_service.sh
#   EXTRA_ARGS="--ping 192.168.1.10" scripts/install_service.sh   # ping the sbRIO once a second too
set -euo pipefail

PKG_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MONITOR_PY="$PKG_DIR/corgi_orin_monitor/monitor.py"
LOG_DIR=${LOG_DIR:-/var/log/corgi_orin_monitor}
EXTRA_ARGS=${EXTRA_ARGS:-}
UNIT_NAME=corgi-orin-monitor.service
UNIT=/etc/systemd/system/$UNIT_NAME
PERSIST_JOURNAL=0
for a in "$@"; do
  case "$a" in
    --persistent-journal) PERSIST_JOURNAL=1 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown option: $a" >&2; exit 2 ;;
  esac
done

SUDO=
[ "$(id -u)" -ne 0 ] && SUDO=sudo
[ -f "$MONITOR_PY" ] || { echo "monitor.py not found at $MONITOR_PY" >&2; exit 1; }
command -v python3 >/dev/null || { echo "python3 missing" >&2; exit 1; }

echo "installing $UNIT_NAME"
echo "  monitor : $MONITOR_PY"
echo "  log dir : $LOG_DIR"
[ -n "$EXTRA_ARGS" ] && echo "  extra   : $EXTRA_ARGS"
sed -e "s|@MONITOR_PY@|$MONITOR_PY|g" \
    -e "s|@LOG_DIR@|$LOG_DIR|g" \
    -e "s|@PKG_DIR@|$PKG_DIR|g" \
    -e "s|@EXTRA_ARGS@|$EXTRA_ARGS|g" \
    "$PKG_DIR/systemd/corgi-orin-monitor.service.in" | $SUDO tee "$UNIT" >/dev/null
$SUDO mkdir -p "$LOG_DIR"

if [ "$PERSIST_JOURNAL" -eq 1 ]; then
  echo "enabling persistent journald storage (/var/log/journal)"
  $SUDO mkdir -p /var/log/journal
  $SUDO systemd-tmpfiles --create --prefix /var/log/journal || true
  $SUDO systemctl restart systemd-journald
fi

$SUDO systemctl daemon-reload
$SUDO systemctl enable --now "$UNIT_NAME"
sleep 2
$SUDO systemctl --no-pager --lines=8 status "$UNIT_NAME" || true

cat <<MSG

done.  Useful commands:
  sudo systemctl status corgi-orin-monitor      # is it running
  ls $LOG_DIR                                   # one boot_* directory per Linux boot
  python3 $PKG_DIR/corgi_orin_monitor/report.py --log-dir $LOG_DIR --all     # table of all boots
  python3 $PKG_DIR/corgi_orin_monitor/report.py --log-dir $LOG_DIR           # previous boot's last seconds
MSG
