#!/usr/bin/env bash
# Slow power ladder in ONE run: ramp k_radial live while the hop template runs.
#
# k_radial is a live parameter of gslip_pronk_node (on_set_params prints
# "LIVE GAIN SET: k_radial = ..." on every step, so the tee'd launch log
# carries the ramp timeline next to the bag). Push-off force is stiffness x
# compression, so raising k_radial from a soft "standing spring" to the
# config-of-record 8941 N/m walks the hop from a bounce to full height.
#
#   1. launch the hop template with the soft start:
#        ... template_path:=.../gslip_hop_template.csv k_radial:=2500.0 ...
#   2. bag on, trigger on, wait for the settle and the first hops
#   3. run this script; it steps k_radial every DWELL seconds
#   4. trigger off when you have seen enough, or Ctrl-C here to freeze the ramp
#
# Usage: ramp_k_radial.sh [FROM] [TO] [STEP] [DWELL_S]
#   defaults 2500 8941 400 1.0  -> 16 steps, ~16 s, ~4 strides per step
set -o pipefail
FROM=${1:-2500}; TO=${2:-8941}; STEP=${3:-400}; DWELL=${4:-1.0}
NODE=/gslip_pronk_node
trap 'echo; echo "ramp stopped at k_radial=$k (Ctrl-C); the node keeps this value until relaunch"; exit 0' INT
k=$FROM
while :; do
  if ! ros2 param set "$NODE" k_radial "$k.0" >/dev/null 2>&1; then
    echo "!! ros2 param set failed at k_radial=$k -- is $NODE up? stopping"; exit 1
  fi
  printf '%s  k_radial -> %s\n' "$(date +%H:%M:%S.%N | cut -c1-12)" "$k"
  [ "$k" -ge "$TO" ] && break
  sleep "$DWELL"
  k=$(( k + STEP )); [ "$k" -gt "$TO" ] && k=$TO
done
echo "ramp complete: k_radial=$TO (config of record 8941). Trigger off when done."
