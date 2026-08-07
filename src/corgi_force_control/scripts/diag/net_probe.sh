#!/usr/bin/env bash
# Which host:port can WSL actually reach the Windows host on?
#
# webots_ros2_driver's WebotsController defaults WEBOTS_CONTROLLER_URL to
# tcp://localhost:<port>/<robot>. Under WSL2 NAT networking, WSL's localhost is
# NOT the Windows localhost, so that default can never reach a Windows-side
# Webots. Under mirrored networking it can. This tells us which regime we are in.
#
# Usage: net_probe.sh [port]   (default 1234)

PORT="${1:-1234}"
GW=$(ip route show default | awk '/default/ {print $3}')

echo "wsl eth0    : $(ip -4 addr show eth0 2>/dev/null | awk '/inet /{print $2}')"
echo "gateway     : ${GW:-<none>}"
echo "networking  : $(if [ -z "$GW" ]; then echo mirrored-or-broken; else echo NAT; fi)"
echo "probing port: $PORT"
echo

for target in 127.0.0.1 "$GW"; do
    [ -z "$target" ] && continue
    if timeout 3 bash -c "cat < /dev/null > /dev/tcp/$target/$PORT" 2>/dev/null; then
        echo "  $target:$PORT  OPEN"
    else
        echo "  $target:$PORT  closed"
    fi
done
