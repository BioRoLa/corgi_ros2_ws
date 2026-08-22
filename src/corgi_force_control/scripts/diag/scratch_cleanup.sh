#!/bin/bash
# Drop the two probes whose clock anchor was superseded by the kp readout
# (beta_cmd derivative / beta_cmd minimum both gave clock_duty 0.465 against
# the template's 0.408; the kp anchor gives 0.408 and is exact). Their valid
# content is reproduced in scratch_kpclock.py and scratch_rate_settle.py.
D=/home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_force_control/scripts/diag
rm -f "$D/scratch_clockphase.py" "$D/scratch_timing.py"
ls -1 "$D"/scratch_*
