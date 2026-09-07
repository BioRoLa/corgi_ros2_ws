# IMU attitude estimator vs Vicon truth — all 18 runs of 2026-09-06

Wrap-agent analysis, 2026-09-06 night. Extends log §325.29 (three runs, pitch only) to every run
of the session that has both a bag and a Vicon c3d (18 runs), and to roll and yaw.
Analyser: `scratchpad/imu_vs_vicon2.py` (third pass; results `imu_vs_vicon3.json`, printout in
task output). Figure: `Research/SLIP/Figures/imu_vs_vicon_2026-09-06.png`, generator
`make_figures_2026-09-06_imu_vs_vicon.py` beside it. Every number below is copied from analyser
output. Labels: **MEASURED** = read off the data; **INFERRED** = follows from measured numbers
by an argument stated here; **SPECULATION** = candidate mechanism, not tested tonight.

## Method (so the numbers can be reproduced)

- **Vicon truth.** Kabsch fit of body markers B1–B6 against a reference frame near the hop
  start (500 Hz). Body axes at the reference: fore = PCA long axis of the marker cloud
  (elongation 3.2 in every file), sign-aligned to the early travel direction; up = lab vertical;
  left = up × fore. Attitude reported in **physical** terms: roll + = left side up, pitch + =
  nose up, yaw + = left turn (CCW from above). Yaw unwrapped.
- **IMU.** `/imu` is a custom `corgi_msgs/ImuStamped` at 1 kHz (header ≈ bag time, 3–6 %
  recorder drops). Three tracks: (i) the **quaternion** roll/pitch/yaw with the same ZYX formulas
  `l15_diag/l_trend/pitch_clamp` use (raw sign kept); (ii) a **gyro-only** attitude — the body
  rates integrated as a quaternion in the FLU frame, seeded with the IMU's own roll/pitch at the
  window start, Euler angles extracted in the physical convention (this is what the fusion would
  output with the accelerometer switched off, so quaternion − gyro-only = what the fusion adds);
  (iii) the raw per-axis integral ∫g (reported for completeness; it is not an Euler-angle change
  once the body yaws while pitched/rolled, and is misleading in the cambered runs).
- **Clock alignment.** No shared clock. Coarse lag from the non-periodic launch edge (the robot is
  completely static until 2.42–2.63 s after the trigger in every run — gy std 0.1 °/s, a_z std
  0.13 m/s² — then hops; Vicon sees the same edge), fine lag from cross-correlating the body's
  vertical velocity (Vicon centroid dz vs ∫a_z) within ±0.15 s. Result r = 0.877–0.998 with
  r one stride away 0.79–0.96. **Passes 1–2 of this analysis used a plain ±2 s correlation
  search and locked onto a stride alias 2 strides (0.53 s) early in every run** (3 strides in
  L0_ramp_3) because the true lag sat outside the search range; those numbers were discarded.
- **Window.** Common hop window = Vicon hop window (longest span of vertical-velocity RMS > 35 %
  of max, as in `vicon_pitch.py`) ∩ bag trigger window, 2.4–9.1 s long. **Drift = mean(last
  third) − mean(first third)**, the `vicon_pitch.py` definition, applied identically to all
  tracks. **Excess rate** = slope of a linear fit to the 1 s-smoothed pointwise IMU − Vicon on the
  aligned time base (deg/s), with R².
- **Sign test per axis** = correlation of the 0.5 s-high-passed IMU track with the high-passed
  Vicon track on the aligned base (the per-hop oscillation, ~5–10° p-p in pitch, decides the
  physical sense independently of the small drifts).

## Reproduction of §325.29, and a correction to how it was computed

| run | §325.29 Vicon | this pass, `vicon_pitch` quantity | §325.29 IMU | this pass, `l_trend` halves | like-for-like (thirds, same window): Vicon / IMU / ratio |
|---|---|---|---|---|---|
| L0_RA1 | −0.74° | −0.74° | −2.72° | −2.72° | −0.79 / −3.89 / **4.9×** |
| OL10_R1 | −0.68° | −0.68° | −1.50° | −1.50° | −0.91 / −2.83 / **3.1×** |
| OL10_NA1 | −2.11° | −2.11° | −2.72° | −2.72° | −2.24 / −4.03 / **1.8×** |

All six §325.29 numbers reproduce exactly (MEASURED). But the IMU column of §325.29 is
`l_trend.py`'s **first-half vs last-half** change of the per-stride pitch means over strides
2.6 s → gait − 0.3 s, while the Vicon column is **thirds** over the Vicon hop window. Halves
understate a linear drift by 25 % relative to thirds, and the stride window is shorter than the
hop window, so §325.29's ratios (3.7×, 2.2×, 1.3×) are low; like-for-like they are 4.9×, 3.1×,
1.8×. The finding of §325.29 stands and strengthens; only the ratios move. (Correction for the
log, as a dated callout under §325.29 — the log is owned by another agent tonight.)

## Main table — drift over the common hop window, 18 runs

IMU = quaternion (what `pitch_correction()` consumes). Ratio = IMU/Vicon; for pitch it is
ill-conditioned when |Vicon| < 0.5°, read the excess column instead. Yaw ratio = |IMU|/|Vicon|
(the quaternion yaw is mirrored, see below). Per-hop r = sign test (quaternion vs Vicon).

| run | λ | k_roll | window s | Vicon roll | IMU roll | Vicon pitch | IMU pitch | ratio pitch | Vicon yaw | IMU yaw (raw) | ratio yaw | pitch excess °/s quat / gyro-only | R² | per-hop r roll/pitch/yaw |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `L0_ramp_1` | 0 | 0.25 | 9.1 | −0.58 | −0.66 | −0.51 | −2.93 | 5.7× | −12.4 | +12.4 | 1.00 | −0.41 / −0.25 | 0.89 | +0.97 / +0.99 / −1.00 |
| `L0_ramp_2` | 0 | 0.25 | 8.4 | −0.51 | +0.49 | −0.54 | −2.76 | 5.1× | −7.8 | +7.7 | 0.99 | −0.40 / −0.24 | 0.98 | +0.96 / +0.99 / −1.00 |
| `L0_ramp_3` | 0 | 0.25 | 6.8 | −0.37 | −0.94 | +0.48 | −2.99 | (−6.2×) | −14.7 | +14.8 | 1.01 | −0.74 / −0.46 | 0.98 | +0.88 / +0.99 / −0.99 |
| `L0_ramp_4` | 0 | 0.25 | 7.6 | −0.50 | −0.45 | −0.55 | −4.41 | 8.0× | −13.9 | +14.0 | 1.01 | −0.76 / −0.49 | 1.00 | +0.93 / +0.99 / −1.00 |
| `L0_ramp_5` | 0 | 0.25 | 6.9 | −0.27 | −1.54 | −0.37 | −4.37 | 11.9× | −18.4 | +18.5 | 1.01 | −0.87 / −0.71 | 0.97 | +0.79 / +0.98 / −0.99 |
| `L15_1` | CL ~13.4 | 0.25 | 3.0 | −1.32 | −1.35 | −0.50 | +0.36 | (−0.7×) | +17.1 | −17.1 | 1.00 | +0.25 / +0.04 | 0.90 | +0.98 / +1.00 / −1.00 |
| `L15_2` | CL ~13.4 | 0.25 | 3.1 | −1.03 | −1.01 | −1.30 | −1.01 | 0.8× | +18.3 | −17.8 | 0.97 | −0.00 / −0.38 | 0.00 | +0.96 / +0.99 / −0.99 |
| `L15_B1` | CL ~13.4 | 0.25 | 3.1 | −1.95 | −1.94 | −1.24 | −1.00 | 0.8× | +13.7 | −13.3 | 0.97 | +0.27 / −0.04 | 0.94 | +0.99 / +1.00 / −0.99 |
| `L10_A1` | CL ~13.4 | 0.25 | 2.4 | −2.58 | −2.25 | −1.54 | −2.04 | 1.3× | +7.3 | −7.5 | 1.03 | −0.26 / −0.37 | 0.62 | +0.96 / +1.00 / −1.00 |
| `OL10_A1` | +10 | 0.25 | 6.5 | −0.94 | −2.19 | −0.83 | −0.22 | 0.3× | +21.7 | −21.8 | 1.00 | +0.11 / +0.09 | 0.26 | +0.94 / +0.99 / −1.00 |
| `OL10_R1` | +10 | 0.60 | 5.8 | −1.06 | −0.30 | −0.91 | −2.83 | 3.1× | +13.3 | −13.5 | 1.01 | −0.49 / −0.63 | 0.90 | +0.95 / +0.99 / −1.00 |
| `L0_RA1` | 0 | 0.60 | 6.7 | −0.58 | −0.06 | −0.79 | −3.89 | 4.9× | −3.7 | +3.6 | 0.97 | −0.69 / −0.98 | 0.97 | +0.94 / +0.98 / −0.99 |
| `OL10_NA1` | −10 | 0.60 | 4.9 | +0.22 | −1.15 | −2.24 | −4.03 | 1.8× | −20.6 | +21.0 | 1.02 | −0.47 / −0.51 | 0.55 | +0.95 / +0.99 / −0.99 |
| `OL10_NA2` | −10 | 0.60 | 5.1 | +0.48 | +0.07 | −0.44 | −1.17 | 2.7× | −15.4 | +15.4 | 1.01 | −0.21 / −0.19 | 0.89 | +0.98 / +1.00 / −1.00 |
| `OL15_A1` | +15 | 0.60 | 3.3 | −1.73 | −1.17 | −0.65 | −1.95 | 3.0× | +16.4 | −16.3 | 0.99 | −0.58 / −0.57 | 0.84 | +0.96 / +0.99 / −1.00 |
| `OL15_NA1` | −15 | 0.60 | 3.6 | +0.99 | +0.09 | −1.72 | −2.23 | 1.3× | −17.9 | +18.5 | 1.03 | −0.15 / +0.23 | 0.97 | +0.96 / +0.99 / −0.99 |
| `L0RA2` | 0 | 0.60 | 6.9 | −0.40 | −0.46 | −0.32 | −1.10 | 3.4× | −2.6 | +2.6 | 0.99 | −0.17 / −0.21 | 0.77 | +0.93 / +0.99 / −1.00 |
| `OL10A2` | +10 | 0.60 | 4.2 | −0.84 | −1.68 | −1.16 | −1.71 | 1.5× | +13.1 | −12.6 | 0.96 | −0.26 / −0.79 | 0.91 | +0.97 / +1.00 / −1.00 |

Aggregates (MEASURED): Vicon pitch drift −2.24…+0.48°, median −0.72°; quaternion pitch drift
−4.41…+0.36°, median −2.13°, negative in 17/18 (L15_1 is the exception at +0.36°). Vicon fit
100 % of frames in every window; IMU drop 3–6 %.

## Signs: does the IMU pitch agree with Vicon or mirror it?

**MEASURED, 18/18 runs each:**

| IMU quantity | vs Vicon (physical) | per-hop r | verdict |
|---|---|---|---|
| quaternion roll | same sense (left side up +) | +0.79…+0.99 | **agrees** |
| quaternion pitch | same sense (nose up +) | +0.98…+1.00 | **agrees** |
| quaternion yaw | opposite (raw yaw + = right turn) | −0.99…−1.00 | **mirrored**; drift-level sign opposite in 18/18, magnitude 0.96–1.03× |
| ∫gx (raw gyro) | same sense | +0.80…+0.99 | agrees |
| ∫gy (raw gyro) | opposite (gy + = nose down) | −0.99…−1.00 | mirrored |
| ∫gz (raw gyro) | same sense (gz + = left turn) | +0.98…+1.00 | agrees |

So the vault fact holds exactly — the quaternion pitch and yaw are the x-mirror of the gyro,
roll agrees — and Vicon settles which one is physical: **the gyro triad is FLU (x fwd, y left,
z up: nose-up = −gy, left turn = +gz), the quaternion is expressed in an FRD/aerospace frame
(pitch + = nose up, yaw + = right turn)** (INFERRED from the six sign tests; a 180° rotation about
x maps one to the other, which is why roll is the axis they share). The IMU pitch drift therefore
has the **same sign as Vicon's** — both nose-down — and is not mirrored; the phantom is an
over-reading of nose-down, not a sign error. The memory note's "pitch magnitude differs ~2×" is
not reproduced tonight: with the alias-free alignment the per-hop pitch amplitude agrees to
within ~10 % (raw traces match to 0.1–0.3°).

## The pitch excess: bias or fusion? Neither — it is in the gyro, and it is motion-induced

Pitch drift (deg, nose-up +) by track, plus the static gyro bias measured on the 2 s before the
trigger of the same run:

| run | Vicon | quaternion | gyro-only | raw ∫gy (nose-up) | fusion adds (quat − gyro-only) | pre-trigger gy bias °/s |
|---|---|---|---|---|---|---|
| `L0_ramp_1` | −0.51 | −2.93 | −2.00 | −2.27 | −0.93 | −0.006 |
| `L0_ramp_2` | −0.54 | −2.76 | −1.87 | −1.91 | −0.89 | −0.008 |
| `L0_ramp_3` | +0.48 | −2.99 | −1.73 | −1.93 | −1.26 | −0.004 |
| `L0_ramp_4` | −0.55 | −4.41 | −3.03 | −2.92 | −1.38 | −0.004 |
| `L0_ramp_5` | −0.37 | −4.37 | −3.68 | −3.63 | −0.69 | −0.006 |
| `L15_1` | −0.50 | +0.36 | −0.14 | +2.01 | +0.50 | +0.003 |
| `L15_2` | −1.30 | −1.01 | −1.87 | −0.24 | +0.86 | +0.016 |
| `L15_B1` | −1.24 | −1.00 | −1.66 | −0.15 | +0.66 | −0.011 |
| `L10_A1` | −1.54 | −2.04 | −2.28 | −1.47 | +0.24 | −0.001 |
| `OL10_A1` | −0.83 | −0.22 | −0.38 | +1.30 | +0.16 | −0.003 |
| `OL10_R1` | −0.91 | −2.83 | −3.40 | −2.56 | +0.57 | +0.002 |
| `L0_RA1` | −0.79 | −3.89 | −5.24 | −5.22 | +1.36 | +0.004 |
| `OL10_NA1` | −2.24 | −4.03 | −4.07 | −3.08 | +0.04 | −0.003 |
| `OL10_NA2` | −0.44 | −1.17 | −1.11 | −0.02 | −0.06 | −0.002 |
| `OL15_A1` | −0.65 | −1.95 | −1.90 | −0.31 | −0.05 | +0.007 |
| `OL15_NA1` | −1.72 | −2.23 | −1.16 | +0.69 | −1.07 | +0.000 |
| `L0RA2` | −0.32 | −1.10 | −1.28 | −1.33 | +0.17 | −0.003 |
| `OL10A2` | −1.16 | −1.71 | −3.20 | −2.37 | +1.48 | +0.004 |

**Apparent gyro bias implied by the excess (MEASURED):**

- Quaternion excess rate: **−0.87 … +0.27 °/s, median −0.33 °/s** (mean −0.33, sd 0.34).
  Gyro-only excess rate: −0.98 … +0.23 °/s, median −0.38 °/s (mean −0.36, sd 0.32).
- Fusion contribution (quaternion − gyro-only excess): −0.38 … +0.53 °/s, **median +0.02**. The
  accelerometer correction neither creates nor removes the pitch drift on the hop time scale.
- Static gyro bias, pre-trigger, y axis: **−0.011 … +0.016 °/s** in all 18 runs (x and z the
  same). That is 20× smaller than the median excess and 50× smaller than the worst run.
- Linearity: the 1 s-smoothed excess fits a line with R² > 0.7 in 14/18 (median 0.90); the four
  poor fits are runs whose excess is ≈ 0 (nothing to fit). Within a run the rate is not constant
  either — the time course (four runs printed) starts at −0.1…−0.4 °/s in the first 2 s and
  runs at −0.5…−1.2 °/s in the second half of the arc (L0_RA1: −0.32° at 2 s, −1.40° at 3 s,
  −3.79° at 6 s of the excess), i.e. the phantom accelerates as the arc proceeds.
- **Is it the same across runs?** No. The range +0.27 to −0.87 °/s is 3× the median; grouped:
  λ0 at k_roll 0.25 −0.40…−0.87 (five runs, the worst group); λ0 at 0.60 −0.69 / −0.17; +λ
  open-loop +0.11 (k_roll 0.25) / −0.49 / −0.58 / −0.26; −λ −0.47 / −0.21 / −0.15;
  closed-loop +0.25 / 0.00 / +0.27 / −0.26. The violent short cambered arcs (gy rms 46–55 °/s)
  show the *least* phantom; the calm straight arcs (gy rms 22–26 °/s) the most.

**Verdict (INFERRED):** the phantom nose-down is **not a gyro bias** (the static bias is ≤ 0.016
°/s and the run-to-run spread of the excess is as large as its median) and **not the fusion**
(the gyro-only dead reckoning carries the same drift to within ±0.5 °/s, and the fusion's median
contribution is +0.02 °/s). It is a **motion-induced error in the gyro's y-axis integral**: it
exists only while the robot hops, accrues roughly linearly at 0.2–1 °/s, varies 3× between
nominally identical runs, and does not snap back when the motion stops.

**Across the stop (MEASURED, five runs):** when the gait ends, Vicon shows a real nose-up settle
of +0.9…+2.0° and the gyro-only track reproduces it (+0.92 vs +0.90, +1.97 vs +2.01, +1.06 vs
+1.21, +1.72 vs +1.87; OL10_R1 +0.25 vs +1.59 is the exception), so the accumulated phantom is
**persistent** — it does not return when the loads come off, which rules out a compliant IMU
mount flexing under hop loads (INFERRED). The quaternion then creeps back toward level at
~0.15 °/s while static (+1.47 → +1.67° over 1.6 s after the stop in L0_ramp_3, +2.32 → +2.52 in
L0_RA1): that is the accelerometer re-levelling a drifted estimate, and it is why **a 60 s static
log (the test registered in §325.29 item 1) will read ≈ 0 and not reproduce the drift** —
tonight's 18 pre-trigger windows already are that test, 2 s each, and all read ≤ 0.016 °/s.

**What tonight's data exclude (MEASURED):** gyro range clipping — the extreme samples in every
run are single values (1–6 samples at the extreme, non-round limits; peak |gy| 60–165 °/s,
peak a_z 36–45 m/s²). Recorder drops — the quaternion is computed upstream of the recorder and
drifts the same as the gyro integral computed from the recorded stream.

**Mechanism candidates (SPECULATION, none tested):** (a) scale-factor asymmetry or nonlinearity
of the y gyro rectifying the asymmetric per-hop waveform (the impact spike is a fast nose-*up*
transient of −60…−165 °/s, the recovery a slower nose-down; under-reading the spikes by a few %
yields net nose-down, the sign observed); (b) vibration rectification of the gyro bias under the
3–4 g impacts; (c) coning/sculling error in the IMU's internal integration at 1 kHz output with
a higher internal rate. The observation that the most violent runs drift least argues against
(b) in its simplest form. Exploratory correlations (n = 18, covariates collinear — the CL runs
are short, violent and rolled): quaternion excess vs mean body pitch r = +0.82, vs gy rms
+0.75, vs window length −0.52; gyro-only excess vs mean pitch +0.52. Not a finding, a lead.

## Roll and yaw

**Yaw (MEASURED):** the quaternion yaw is the mirror of the physical yaw in 18/18 runs, with
|quaternion|/|Vicon| = **0.96–1.03, median 1.00** (n = 15 with |yaw| > 5°). Yaw excess rates
are within ±0.05 °/s in 16/18 (max 0.15 °/s, OL15_NA1 gyro-only). ∫gz matches Vicon within
1–4 % over the arc. **Yaw is estimated correctly up to the sign convention**; a heading loop
reading the quaternion yaw without flipping it would steer the wrong way — k_yaw is 0 in the
config of record, so nothing tonight consumed it.

**Roll (MEASURED):** quaternion roll agrees with Vicon in sign (per-hop r +0.79…+0.99) and in
drift for the closed-loop arcs (−1.35 vs −1.32, −1.01 vs −1.03, −1.94 vs −1.95, −2.25 vs −2.58);
in the open-loop cambered and straight arcs it scatters by up to ±1.3° with **no consistent
sign** (excess −0.40…+0.23 °/s). The gyro-only roll excess is larger (−0.77…+0.32 °/s) and follows
the turn direction in the open-loop arcs (+λ negative, −λ positive), so on roll the accelerometer
pull *does* work and removes most of the gyro's drift — unlike pitch, where it removes none
(INFERRED from the two tracks). P-HW-ρ and the k_roll channel read roll: the estimate they get is
right to ~1° over an arc.

| run | Vicon roll | quaternion | gyro-only | excess °/s quat / gyro-only | mean gz °/s |
|---|---|---|---|---|---|
| `L0_ramp_1` | −0.58 | −0.66 | −1.80 | −0.02 / −0.20 | −2.12 |
| `L0_ramp_2` | −0.51 | +0.49 | −0.51 | +0.17 / +0.00 | −1.64 |
| `L0_ramp_3` | −0.37 | −0.94 | −1.52 | −0.13 / −0.25 | −3.29 |
| `L0_ramp_4` | −0.50 | −0.45 | −0.92 | +0.02 / −0.07 | −2.60 |
| `L0_ramp_5` | −0.27 | −1.54 | −1.97 | −0.27 / −0.34 | −4.17 |
| `L15_1` | −1.32 | −1.35 | −2.55 | +0.03 / −0.53 | +5.52 |
| `L15_2` | −1.03 | −1.01 | −1.36 | −0.03 / −0.12 | +6.03 |
| `L15_B1` | −1.95 | −1.94 | −2.47 | −0.03 / −0.29 | +5.54 |
| `L10_A1` | −2.58 | −2.25 | −2.69 | +0.01 / −0.16 | +4.21 |
| `OL10_A1` | −0.94 | −2.19 | −4.32 | −0.27 / −0.77 | +4.43 |
| `OL10_R1` | −1.06 | −0.30 | −0.57 | +0.20 / +0.14 | +2.95 |
| `L0_RA1` | −0.58 | −0.06 | −0.93 | +0.12 / −0.07 | −0.92 |
| `OL10_NA1` | +0.22 | −1.15 | −1.20 | −0.40 / −0.40 | −6.77 |
| `OL10_NA2` | +0.48 | +0.07 | +1.58 | −0.13 / +0.31 | −4.35 |
| `OL15_A1` | −1.73 | −1.17 | −2.79 | +0.23 / −0.46 | +6.17 |
| `OL15_NA1` | +0.99 | +0.09 | +1.54 | −0.40 / +0.17 | −7.04 |
| `L0RA2` | −0.40 | −0.46 | −0.21 | −0.00 / +0.03 | −0.64 |
| `OL10A2` | −0.84 | −1.68 | −0.63 | −0.32 / +0.04 | +3.75 |

## What this means for the controller (INFERRED from §325.29's mechanism and tonight's numbers)

`pitch_correction()` applies `k_pitch · pitch` (0.30) as a front-positive/rear-negative θ offset
using the quaternion pitch. With a phantom of median −2.1° per arc (up to −4.4°), that is
0.6–1.3° of differential θ commanded for no physical reason by the end of a 7–9 s arc, on top
of the real −0.7° median tilt; the θ clamp (`pitch_limit` 3°) is hit ~52 % of stance already
(§325.26), so the phantom is spending the clamp budget. Because the phantom (i) is in the gyro,
(ii) is slow (≤ 1 °/s) against per-hop pitch dynamics of ±5°, and (iii) is corrected by the
accelerometer only when static, the software fixes that follow are: feed `pitch_correction()`
a high-passed or accelerometer-leveled pitch (the controller needs the per-stride attitude, not
the DC), or lower k_pitch (the §325.29 next-session item 3 remains the cleanest single test).
These are options, not decisions.

**Re-scope of §325.29's registered next-session item 1.** "Hold the robot still 60 s and log the
quaternion" will read ≤ 0.02 °/s and prove nothing (the 18 pre-trigger windows already did).
The test that reproduces the drift is motion: hop in place on the straps with Vicon (or a
second, independent IMU) and compare ∫gy against truth; or shake the IMU by hand for 30 s with a
marker cluster on it. A bench characterisation of the gyro against rate amplitude (turntable or
a known-angle rotate-and-return at 50–150 °/s) would separate candidate (a) from (b).

## Caveats

- n = 1 arc per run; windows 2.4–9.1 s (the closed-loop arcs are 2.4–3.1 s, so their thirds are
  ~1 s means and their drifts are noisy — treat the four CL rows as indicative).
- Vicon attitude is relative to a reference frame assumed level; drifts (differences) are
  unaffected, absolute angles are not reported for that reason.
- The fore axis is the PCA long axis; the robot's travel direction was 0.1–48° off it in the
  cambered/CL arcs (it crabs and turns), but only the sign of the axis is taken from travel.
- The raw ∫g columns are body-rate integrals and mix axes when the body yaws while pitched;
  use the gyro-only quaternion track for any argument.
- Mechanism paragraph is speculation; correlations are exploratory with n = 18 and collinear
  covariates.
- Passes 1–2 of this analysis (now superseded) had the 2-stride alignment alias; nothing from
  them survives in this note or in the figure. The alias would have inflated the excess rates
  (median −0.68 vs −0.33 °/s) and produced a fictitious launch transient.

## Files

- `C:\Users\alexc\Documents\Obsidian Vault\Research\SLIP\Figures\imu_vs_vicon_2026-09-06.png`
  and `make_figures_2026-09-06_imu_vs_vicon.py` (numbers hardcoded from `imu_vs_vicon3.json`).
- Scratchpad: `imu_vs_vicon2.py` (analyser, third pass), `imu_vs_vicon3.json` (results),
  `imu_extract.py` (tables), `imu_excess_time.py`, `imu_stop.py`, `imu_onset.py`, `imu_launch.py`,
  `imu_segments.py`, `imu_gyroclip.py`, `imu_probe.py`, and their `run_*.sh` runners.
- Superseded: `imu_vs_vicon.py` / `imu_vs_vicon.json` / `imu_vs_vicon2.json` (alias-affected).
