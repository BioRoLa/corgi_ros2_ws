#!/usr/bin/env bash
# Read-only: does the front-leg drag differ between +camber and -camber? If it
# is common to both signs the mirror contrast removes it; if it is antisymmetric
# it biases the camber term itself.
set -o pipefail
H=biorola@192.168.30.244
ssh -o BatchMode=yes -o ConnectTimeout=6 "$H" 'bash -s' <<'REMOTE'
source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=67
W=~/corgi_ws/corgi_ros2_ws
D=$W/src/corgi_force_control/scripts/diag/hw0906
ARGS=""
for R in s3_l0_a2 s3_ol10_a1v2 s3_ol10_a2 s3_ol10n_a2 s3_ol15n_a1 s3_l0_a3; do
  [ -f "$W/bag/$R/${R}_0.db3" ] && ARGS="$ARGS $W/bag/$R/${R}_0.db3"
done
python3 $D/leg_duty.py $ARGS 2>&1 | grep -v Deprecation | grep -v "square wave\|spike\|SELF-TEST"
echo
echo "===== the failed arc: what is actually in it? ====="
R=s3_ol10n_a1
python3 - <<'PY'
import sqlite3, os
p = os.path.expanduser("~/corgi_ws/corgi_ros2_ws/bag/s3_ol10n_a1/s3_ol10n_a1_0.db3")
con = sqlite3.connect("file:%s?mode=ro" % p, uri=True)
for i, n, t in con.execute("SELECT id,name,type FROM topics"):
    c = con.execute("SELECT count(*) FROM messages WHERE topic_id=?", (i,)).fetchone()[0]
    print("  %-16s %8d msgs" % (n, c))
r = con.execute("SELECT min(timestamp), max(timestamp) FROM messages").fetchone()
if r and r[0]:
    print("  span %.2f s" % ((r[1]-r[0])/1e9))
PY
REMOTE
