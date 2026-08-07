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

Templates are installed at
`install/corgi_force_control/share/corgi_force_control/config/` —
`gslip_hop_template.csv` and `gslip_pronk_template.csv`.

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

**Use the bracket trick in `pkill`.** A bare `pkill -f name` matches the calling
script's own command line and kills the caller.

**Kill Windows-side Webots between runs.** The WSL `pkill` does not reap it, and
the stale process holds port 1234 — the next launch then fails with *"not in the
list of robots with `<extern>` controllers"*. `sim_cycle.sh` does this already.

**Gains must be floats on the launch line.** `b_radial:=10` aborts the node
against a `double` parameter; use `10.0`.
