#!/usr/bin/env bash
# Read-only: score every arc of Session B, with the decay control across the
# lambda0 arcs in time order.
set -o pipefail
H=biorola@192.168.30.244
ssh -o BatchMode=yes -o ConnectTimeout=6 "$H" 'bash -s' <<'REMOTE'
source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=67
W=~/corgi_ws/corgi_ros2_ws
D=$W/src/corgi_force_control/scripts/diag/hw0906
NEW="s3_ol10n_a1 s3_l0_a2 s3_ol10_a2 s3_ol10n_a2 s3_l0_a3 s3_ol15n_a1"
echo "===== bags present / closed ====="
for R in $NEW; do
  if [ -d "$W/bag/$R" ]; then
    printf "  %-18s db3 %s  metadata %s  size %s\n" "$R" \
      "$([ -f $W/bag/$R/${R}_0.db3 ] && echo yes || echo NO)" \
      "$([ -f $W/bag/$R/metadata.yaml ] && echo yes || echo 'NO(open)')" \
      "$(du -h $W/bag/$R/${R}_0.db3 2>/dev/null | cut -f1)"
  else
    printf "  %-18s MISSING\n" "$R"
  fi
done
echo
echo "===== banner: pitch line + cell proof ====="
for R in $NEW; do
  printf "  %-18s " "$R"
  P=$(grep -o "k_pitch=[0-9.]* d_pitch=[0-9.]*" $W/output_data/ctl_$R.log 2>/dev/null | head -1)
  C=$(grep -o "ACKER CL set: ff=[0-9.]* deg.*range=\[[0-9., ]*\]" $W/output_data/ctl_$R.log 2>/dev/null | head -1)
  T=$(grep -o "turn_rate=[-0-9.]* rad/s" $W/output_data/ctl_$R.log 2>/dev/null | head -1)
  KR=$(grep -o "k_roll=[0-9.]*" $W/output_data/ctl_$R.log 2>/dev/null | head -1)
  echo "$P | ${C:-no ACKER line} | $T | $KR"
done
echo
echo "===== launch rule + whole-arc peaks ====="
for R in $NEW; do
  B=$W/bag/$R/${R}_0.db3
  [ -f "$B" ] || { echo "---- $R  NO BAG"; continue; }
  echo "---- $R"
  python3 $D/launch_check.py "$B" 2>&1 | grep -v Deprecation | tail -7
  python3 $D/l_trend.py "$B" 2>&1 | grep -v Deprecation | grep -E "^  (pkA|pkB|pkC|pkD|tauH_D|pitch|roll|flight_frac|vmin)"
done
echo
echo "===== decay control: every lambda0-config arc in time order ====="
ARGS=""
for R in s3_l0_a1 s3_l0_a1b s3_l0_a2 s3_l0_a3; do
  [ -f "$W/bag/$R/${R}_0.db3" ] && ARGS="$ARGS $W/bag/$R/${R}_0.db3"
done
python3 $D/leg_duty.py $ARGS 2>&1 | grep -v Deprecation | grep -v "square wave\|spike\|SELF-TEST"
REMOTE
