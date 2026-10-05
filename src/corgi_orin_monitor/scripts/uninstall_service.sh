#!/usr/bin/env bash
# Stop and remove the corgi-orin-monitor systemd service. Log files are left in place.
set -euo pipefail
SUDO=
[ "$(id -u)" -ne 0 ] && SUDO=sudo
$SUDO systemctl disable --now corgi-orin-monitor.service 2>/dev/null || true
$SUDO rm -f /etc/systemd/system/corgi-orin-monitor.service
$SUDO systemctl daemon-reload
echo "removed corgi-orin-monitor.service (logs kept)"
