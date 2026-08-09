# G-SLIP diagnostics

Measurement scripts for the G-SLIP hop and pronk in Webots. These live in the
repo rather than `/tmp` because a WSL restart wipes `/tmp` and took an earlier
copy with it.

## Running

One complete measurement from a **fresh simulator** — teardown, sim, controller,
trigger, measure:

```bash
bash sim_cycle.sh <template.csv> [launch args...]
```

Every run needs its own restart. A Webots *Reset* is not enough, and *Reload*
drops the extern controller without recovering.

For the **speed ramp**, use `ramp_cycle.sh` instead — `sim_cycle.sh` cannot
measure it correctly (see "Measuring the speed ramp" below):

```bash
bash ramp_cycle.sh <gslip_speed_ramp_template.csv> [launch args...]
```

Templates are installed at
`install/corgi_force_control/share/corgi_force_control/config/` —
`gslip_hop_template.csv`, `gslip_pronk_template.csv` and
`gslip_speed_ramp_template.csv`.

## Scripts

| script | what it answers |
|---|---|
| `sim_cycle.sh` | the standard run: gait ranges, leg phase correlation, flight fraction, attitude |
| `check_pronk.py` | joint ranges, inter-leg correlation, flight vs duty factor |
| `check_rpy.py` | roll / pitch / yaw **separately**, plus per-leg contact balance |
| `check_cmd_vs_state.py` | commanded vs measured per leg, and torque saturation |
| `check_torque_phase.py` | *where in the stride* torque saturates |
| `check_env_propagation.sh` | whether `CORGI_*` env vars reach the driver process |
| `hop_spring_rest.sh` | the `spring_rest_reference` experiment (rejected) |
| `hop_bradial72.sh` | hop at `b_radial = 72` vs 0 |
| `ramp_cycle.sh` | the speed-ramp run: recorder started **before** the trigger, per-segment result |
| `check_ramp.py` | flight fraction **per ramp segment and per stride**, anchored to sim time |
| `check_yaw_phase.py` | *where in the stride* the yaw accumulates — offline, from a `check_ramp.py --dump` |
| `ramp_segments.py` | recovers the ramp's segment / stride boundaries from the CSV |
| `check_gains.sh` | reads back the gains actually in effect on the running node |
| `net_probe.sh` | whether WSL can reach the Windows-side Webots on 1234 |
| `check_port_1234.sh` | distinguishes "can't reach Webots" from "Webots isn't there" |

## Things that will waste your time otherwise

**Run measurements early.** The gait stops roughly a minute after the trigger.
Anything run late reads a static robot — the symptom is sample rates collapsing
and per-leg sweeps going to near zero.

**`spin_once` handles one callback per call.** With a timeout, three 1 kHz
subscriptions round-robin down to ~67 Hz and alias the stride completely. Spin
with `timeout_sec=0.0`.

**Commanded beta is a triangle, not a sawtooth.** It sweeps back through stance
and forward through flight, so there is no discontinuity to key stride
boundaries on. Segment by contact instead.

**Deviation from the CSV is not tracking error.** `gslip_pronk` publishes
`ImpedanceCmd`; the CSV row is the virtual spring's rest pose, not a setpoint.
The deviation *is* the spring deflection. It is only interesting when it differs
between legs, or when its magnitude is far from the design compression
(100 -> 84.4 deg, about 15.6 deg).

**Repeat everything.** Pronk flight has read 11-42% under identical settings;
the hop 69.2 / 69.4 / 77.0. Single runs decide nothing.

**Measure yaw on the HOP, and stop the run early.** `RAMP_UNTIL=<seconds of
template time>` makes `ramp_cycle.sh` end once that much of the pass is
recorded; the recorder gates the trigger, so this shortens the whole run.
`RAMP_UNTIL=2.6` covers the hop segment (0.00-2.14 s) at about a quarter the
cost of the full 9.58 s pass, and the hop shows the yaw effect in full.
`RAMP_DUMP=<path>` names the .npz so a parameter sweep does not overwrite
itself.

**A per-leg beta command breaks the template-time anchor.** `check_ramp.py`
recovers template t=0 from the first non-zero *commanded* beta, so anything that
makes beta non-zero off-template moves the anchor. The stance-gated steering
offset did exactly that during the settle and shifted every segment boundary by
~4.6 s; the run then reported 0% flight, 100% all-down and theta pinned below
command -- the settle wearing the hop's label, indistinguishable from a dead
gait. The anchor now uses the **common-mode** beta across the four legs, which
is immune to any antisymmetric per-leg term. **Check `template time covered` in
the header**: a good run shows about -2.9 s of settle before t=0, a
mis-anchored one shows almost none.

**Averages hide mechanisms.** Per-segment and even per-stride averages cannot
distinguish an impulse at touchdown from a torque through stance from a
pre-existing spin -- all three give the same per-stride average, and that is
what made the yaw look like a per-stride contact asymmetry for ten candidate
causes running. Bin the *rate* by contact state before believing any mechanism.
`check_yaw_phase.py` does this for yaw and needs no simulator time.

**`--dump` everything.** `check_ramp.py --dump` costs nothing at run time and
the .npz files are what made the yaw-phase analysis possible weeks later with
no new runs. Only dumps from `corgi_sim >= 77dcac1` carry the odom quaternion
(10-column `odom`); the earlier 6-column ones cannot answer anything about
orientation.

**Use the bracket trick in `pkill`.** A bare `pkill -f name` matches the calling
script's own command line and kills the caller.

**Kill Windows-side Webots between runs.** The WSL `pkill` does not reap it, and
the stale process holds port 1234 — the next launch then fails with *"not in the
list of robots with `<extern>` controllers"*. `sim_cycle.sh` does this already.

**Gains must be floats on the launch line.** `b_radial:=10` aborts the node
against a `double` parameter; use `10.0`. The node only logs `k_radial` at
startup, so a gain that silently failed to apply is otherwise invisible --
`check_gains.sh` reads back what is actually in effect.

**Webots runs far below real time, and the factor varies.** Measured ~14x
slower (8 strides = 2.13 s sim in 30 s wall). `sim_cycle.sh` waits a fixed
**20 s of wall clock** before sampling, so the sample lands at a different
*gait age* every run. Two identical hop invocations gave 68.2% and 9.3% flight
purely from this: one caught strides ~3-6, the other the startup transient at
strides ~1-4. Anything quantitative should key off **sim** time.

**The controller stops the instant the trigger goes false.** `gslip_pronk`
exits its run loop when `trigger_` clears, so killing the trigger publisher
truncates the gait -- that is what "Stopped after N strides" means in the logs.
A measurement must hold the trigger for as long as it wants to record.

## Measuring the speed ramp

`sim_cycle.sh` is wrong for the ramp in three separate ways, which is why
`ramp_cycle.sh` exists:

1. **One average hides the answer.** The ramp climbs six fixed points across
   9.58 s; a single flight fraction over all of it cannot show which rungs
   worked.
2. **The template wraps.** `gslip_pronk.cpp` wraps `index` at
   `template_.size()`, so after 9.575 s the ramp snaps from v~1.20 back to the
   in-place hop *while the robot is still carrying 2.035 m/s*. `sim_cycle.sh`'s
   20 s sample sits in the third pass, past two wrap discontinuities. Only the
   first pass means anything.
3. **The phase has to be recovered, not assumed.** `check_ramp.py` anchors on
   commanded beta, which is exactly zero through the hop segment and steps to
   +-7 deg when the first forward segment begins -- an unambiguous fiducial at
   a known template time.

**The ramp carries its own validity gate.** Its first segment *is* the in-place
hop, the one gait validated from standstill. If that segment does not reach
theta ~ 100 deg and ~58% flight, the run is dead and says nothing about the
ramp. A run reading ~10% on *every* rung with theta topping out at 97.7 deg is
this failure, not a ramp result -- it looks exactly like "the ramp does not
work" and is not.
