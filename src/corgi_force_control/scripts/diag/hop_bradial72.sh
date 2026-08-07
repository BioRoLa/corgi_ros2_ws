#!/usr/bin/env bash
# Does b_radial explain the hop reading 58.2% earlier and ~69% now?
#
# b_radial went 72 -> 0 after the energy audit, which was worth 4.0% -> 31.9%
# flight on the pronk. If the 58.2% hop measurement predates that change, it
# explains itself. (The contact-sampling hypothesis is already dead: the driver
# does honour CORGI_CONTACT_INTERVAL, and interval 100 reads HIGHER, 77.0%.)
#
# Two reps: hop flight has read 69.2 / 69.4 / 77.0, so one run decides nothing.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TPL=/home/alexc/corgi_ws/corgi_ros2_ws/install/corgi_force_control/share/corgi_force_control/config/gslip_hop_template.csv

for rep in 1 2; do
    echo "=== b_radial=72.0  rep $rep ==="
    bash "$HERE/sim_cycle.sh" "$TPL" b_radial:=72.0 2>/dev/null \
        | grep -E "airborne|^   [ABCD] |mean|FAILED"
    echo
done
echo "BRADIAL_TEST_DONE"
