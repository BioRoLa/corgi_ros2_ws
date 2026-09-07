# hw0906 — analysers of Hardware Sessions 1–2 (2026-09-06, Vicon volume)

Rescued 2026-09-07 from the session scratchpad (a temp directory); `launch_check.py`
and `ff_scan.py` were written the same day for the next session. Every number in
log §§325.20–325.42 and the figures `Research/SLIP/Figures/*_2026-09-06.png` came
from these. Data: WSL `~/corgi_runs/hw_2026-09-06/` (bags, logs, vicon c3d).

Bag analysers need `source /opt/ros/humble/setup.bash` and
`source ~/gslip_merge_ws/install/setup.bash`; Vicon analysers need python3 + `c3d`.
The `run_*.sh` files are the exact invocations used (paths are the WSL ones).

| file | role |
|---|---|
| `vicon_l0.py` | **the registered estimator** — whole-arc v_fwd, yaw rate, κ, dropout, from a c3d |
| `vicon_split.py`, `vicon_pitch.py` | 2 s slices (exploratory only), Vicon tilt vs arc |
| `l_trend.py` | per-stride bag trend: camber, θ sweep, per-leg peak load, τ_h(D), pitch/roll, flight-fraction proxy (a_z < ½ g), bus |
| `l15_diag.py` | per-leg loads / attitude for one arc |
| `compliance.py`, `compliance2.py`, `hysteresis.py`, `ride_height.py`, `pitch_clamp.py` | the §325.35–38 compliance / hysteresis / ride-height / pitch-clamp passes |
| `filter_bags.py` | skips bags malformed by an Orin reset |
| `ceiling_shape.py` | the 41.57 N·m ceiling shape over 37 bags (§325.41) |
| `primary_outcomes.py` (+ `.json`, `.md`) | the four registered outcomes, IMU↔Vicon alignment, self-tested (§325.39) |
| `figs0906_*` | per-hop speed, per-stride τ_h and roll, literals for the figure generators |
| `exploratory_slope.py` (+ `.md`, `_strides.json`) | per-stride κ vs measured camber — exploratory, not a gate |
| `imu_vs_vicon.py`, `imu_vs_vicon2.py`, `imu_vs_vicon3.json`, `imu_vs_vicon.md` | IMU vs Vicon attitude, 18 runs; only `imu_vs_vicon3.json` stands (§325.40) |
| `launch_check.py` | **the registered launch-stride validity check** (log §325.43): VALID/VOID for one arc's bag, run between arcs |
| `ff_scan.py` | per-stride flight fraction across many bags — the false-positive check behind that rule |
| `run_launch_check.sh`, `run_ff_scan.sh` | their self-tests against the 2026-09-06 known answer |
| `session1_runsheet.html`, `session2_runsheet.html` | the bench run-sheets; the launch line's base |
