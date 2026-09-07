# §325.25 secondary — EXPLORATORY per-stride κ vs measured camber (data 2026-09-06, computed in the wrap-up session)

**Status: EXPLORATORY, secondary outcome, registered in §325.25 as such. Nothing here is a primary result, and
nothing here re-scores any registered gate.** Whole-arc numbers in the log (§325.27, §325.31, §325.34, §325.37)
stand; if a number here disagrees with the log, the log wins.

Analyser: `scratchpad/exploratory_slope.py` (run via `run_exploratory_slope.sh`; full printout
`exploratory_slope_out.txt`, per-stride JSON `exploratory_slope_strides.json`).
Figure: `Research/SLIP/Figures/exploratory_kappa_slope_2026-09-06.png`, generator
`make_figures_2026-09-06_exploratory_slope.py` beside it (numbers hardcoded from the analyser printout).

## 1. What was computed (protocol wording → implementation)

| protocol item | implementation | status |
|---|---|---|
| stride i | falling edge of module-A `kp_r` < 100 on `/motor/command`, 2.6 s after the trigger to 0.3 s before its end — the `l0_cell.py` / `l_trend.py` detector, stride 1 = `l_trend.py` i = 0. Hop period 0.265 s (= the v070 template's 0.2642 s) | same detector as every other stride count tonight |
| Δψ_i | integral of IMU gyro z over the stride window, bias = median of the pre-trigger window (−2.0 … −0.2 s) as in `l0_cell.py` (the fallback window was never needed; IMU ~1010 Hz) | MEASURED |
| Δs_i | Vicon body-centroid (B1..B6) horizontal displacement over the **same window**, after mapping the bag clock onto the Vicon clock (§2) | MEASURED |
| κ_i | Δψ_i / Δs_i | MEASURED |
| λ_eff,i | lean-sense mean over the four legs of (commanded γ + tracking error) + IMU roll relative to its pre-trigger level; with error = measured − commanded this is mean_legs(s_leg·γ_meas) + σ·(roll − roll₀). Lean-sense signs s = {A +1, B −1, C −1, D +1}, read off the commanded γ of the cambered runs (identical in all five). σ is a sign ASSUMPTION, §4 | MEASURED except σ |
| N_sat | first stride window in which any leg's peak \|τ_h\| ≥ 39 N·m ("for a complete stride" read as "peak within a full stride window") | MEASURED; identical to the concurrent `make_figures_2026-09-06_abad_torque_per_stride.py` values |
| survivors | strides with i < N_sat; all strides if the run never saturates | — |
| fit | OLS κ_i = a·λ_eff,i + b·i + c on the pooled survivors; OLS SE, run-cluster SE (7 clusters — unreliable, shown for scale), leave-one-run-out jackknife range | — |

Runs: `s2_l0r_a1`/L0_RA1, `s2_l0r_a2`/L0RA2 (λ 0), `s2_ol10_roll1`/OL10_R1, `s2_ol10_a2`/OL10A2 (+10),
`s2_ol10n_a2`/OL10_NA2 (−10), `s2_ol15_a1`/OL15_A1 (+15), `s2_ol15n_a1`/OL15_NA1 (−15). All k_roll 0.60,
open-loop camber, the §325.25 frozen config. 114 stride windows in total.

## 2. Bag → Vicon alignment: how, and how confident

**Method of record = count from the first full hop.** On the bag side the first full hop is the first IMU flight
segment (a_z below half its rest value) lasting ≥ 0.10 s; it ends 2.76–2.79 s after the trigger in every run and
coincides with the first `kp_r` stance entry, i.e. it *is* stride 1. On the Vicon side the first full hop is the
first touchdown (local minimum of the 30 ms-smoothed centroid vertical velocity inside the longest hopping span,
the `vicon_l0.py` window logic) with descent ≥ 0.5 m/s. In every run this is the **second** detected touchdown:
the first is a half-height ramp-in hop (−0.33 to −0.35 m/s) that the bag also shows as a 0.07–0.08 s flight
ending at 2.60 s. So bag stride 1 ↔ Vicon touchdown 2, in all seven runs, with the same ramp-in signature on
both sides. The offset is then `t_vicon = t_bag + off_count`.

**Independent check:** cross-correlation of the two binary flight indicators (IMU: a_z < ½ rest; Vicon: dz-max to
dz-min) on a 200 Hz grid.

| bag / Vicon | strides (windows) | off_count (s) | off_xcorr (s) | Δ (s) | touchdown residuals under off_count: median / sd (s) |
|---|---|---|---|---|---|
| s2_l0r_a1 / L0_RA1 | 23 (22) | +4.320 | +4.230 | −0.090 | −0.090 / 0.044 |
| s2_l0r_a2 / L0RA2 | 24 (23) | +2.191 | +2.105 | −0.086 | −0.090 / 0.046 |
| s2_ol10_roll1 / OL10_R1 | 20 (19) | +2.366 | +2.275 | −0.091 | −0.070 / 0.062 |
| s2_ol10_a2 / OL10A2 | 14 (13) | +2.699 | +2.620 | −0.079 | −0.081 / (one post-gait IMU segment spoils the sd) |
| s2_ol10n_a2 / OL10_NA2 | 17 (16) | +14.740 | +14.650 | −0.090 | −0.092 / 0.078 |
| s2_ol15_a1 / OL15_A1 | 11 (10) | +2.327 | +2.230 | −0.097 | −0.093 / 0.072 |
| s2_ol15n_a1 / OL15_NA1 | 12 (11) | +4.547 | +4.455 | −0.092 | −0.091 / 0.135 |

Reading: the two methods pick the **same hop pairing** in every run (a pairing off by one hop would show
Δ ≈ ±0.265 s, the hop period; the observed Δ is a constant −0.09 s, a third of a period). The −0.09 s is a fixed
phase difference between the IMU flight-end and the Vicon velocity-minimum in steady hopping, which the first hop
does not share (its residual is 0 by construction); I have not resolved which event is "touchdown" and do not need
to: every window is exactly one hop period long, so Δs over it is nearly phase-independent. Verified, not assumed:
κ_i recomputed under the xcorr offset differs from the count-offset κ_i by median +0.0003 m⁻¹ (sd 0.05, the
largest differences in OL10A2's stalled 1–3 cm hops), and the pooled slope is unchanged (0.0170 vs 0.0168).
**Confidence in the hop pairing: high** (same ramp-in structure on both sides in all seven runs, two methods agree
to a third of a period). Confidence in sub-hop phase: ~0.1 s, irrelevant to κ_i at this window length.

Per-run cross-check of the pipeline against the registered whole-arc κ (different window and estimator, so
agreement is expected only roughly):

| run | Σ Δψ / Σ Δs, IMU gyro | same with Vicon Kabsch yaw | registered whole-arc κ (log) |
|---|---|---|---|
| L0_RA1 | −0.057 | −0.055 | −0.0487 |
| L0RA2 | −0.034 | −0.037 | −0.0307 |
| OL10_R1 | +0.174 | +0.176 | +0.2116 |
| OL10A2 | +0.448 | +0.432 | +0.4532 |
| OL10_NA2 | −0.270 | −0.250 | −0.2519 |
| OL15_A1 | +0.504 | +0.519 | +0.5269 |
| OL15_NA1 | −0.422 | −0.429 | −0.4938 |

IMU-integrated and Vicon-yaw curvature agree within each run to ≤ 8 %, so the gyro integration and pre-trigger
de-biasing are sound. Against the registered numbers (whole hop window incl. ramp-in and trigger-off, linear yaw
fit over chord displacement) the per-stride sums sit within 2–18 %; the largest gap (OL10_R1) comes from its
strides 1–4 having small κ. These are not replacements for the registered numbers.

## 3. N_sat and the surviving strides (MEASURED)

| run | cell | N_sat | leg at N_sat (peak \|τ_h\|) | strides (windows) | survivors (i < N_sat) |
|---|---|---|---|---|---|
| s2_l0r_a1 | λ 0 | 5 | C 40.0 | 22 | **4** |
| s2_l0r_a2 | λ 0 | never (C max 36.0) | — | 23 | 23 |
| s2_ol10_roll1 | +10 | 5 | D 39.8 | 19 | **4** |
| s2_ol10_a2 | +10 | 3 | D 40.4 | 13 | **2** |
| s2_ol10n_a2 | −10 | 14 | C 39.5 | 16 | 13 |
| s2_ol15_a1 | +15 | 2 | D 40.6 | 10 | **1** |
| s2_ol15n_a1 | −15 | 11 | C 39.4 | 11 | 10 |

57 of 114 windows survive. The asymmetry the protocol anticipated (§325.25, "the mirror does not mirror") is in
the table: **+λ saturates by stride 2–5, always on D (rear-left); −λ by stride 11–14, on C (rear-right); λ 0
saturates at stride 5 on C in one arc and never in the other.** Consequence for this analysis: the +λ
pre-saturation windows are 1–4 strides long, and stride 1 is a launch transient in every run (κ₁ = −0.43 and
−0.41 at λ 0; −0.35, −0.18, −0.37, −0.11, −0.25 in the cambered arcs — a consistent yaw kick at the first hop,
opposite in sign to +λ's steady curvature).

Also seen in passing (not the question asked): OL10A2's Δs collapses from 7–10 cm/hop to 1–3 cm from stride 6
(it stalls; the per-stride speed marks * in the printout fall below the 0.257 floor from stride 5), and OL15_A1
is below the floor from stride 1.

## 4. Measured camber λ_eff and the sign assumption

MEASURED, in the lean sense (positive = the +λ direction):

| cell | commanded γ in the bag (mean over arc) | tracking error (meas − cmd) | IMU roll rel. pre-trigger | λ_eff, σ = roll adds (A) | λ_eff, roll subtracts (alt.) |
|---|---|---|---|---|---|
| +10 (R1 / A2) | +8.4 / +7.7 | +0.3 / +0.2 | −3.2 / −4.2 | +12.0 / +12.3 | +5.5 / +4.0 |
| −10 | −8.3 | −0.4 | +2.3 | −11.2 | −6.4 |
| +15 | +12.3 | +0.3 | −4.8 | +17.7 | +7.8 |
| −15 | −12.3 | −0.4 | +3.8 | −16.7 | −9.0 |
| λ 0 (RA1 / RA2) | +0.5 / +0.05 | ≈0 | +0.2 / −0.7 | +0.3 / +0.7 | — |

Three things in this table matter beyond this analysis:

1. **The bag commands ≈ 0.83 × the nominal camber** (8.3° for "±10", 12.3° for "±15"). INFERRED from the numbers,
   not from source: commanded lean ≈ λ_nominal + 0.60 × raw IMU roll fits every run to ≤ 0.3° (e.g. OL10_R1:
   10 + 0.6 × (−2.51) = 8.49 vs 8.39 measured; OL10_NA2: −10 + 0.6 × 2.70 = −8.38 vs −8.25), i.e. `k_roll` reads
   as a roll → ABAD-camber feedback with the sign that *reduces* the lean when the body rolls the way it does
   under camber. Whoever quotes "per degree" numbers should say per nominal, per commanded or per measured degree —
   they differ by 17–40 %. Someone should confirm this against the controller source (five minutes).
2. **The ABAD overshoots the command in the lean direction** by 0.2–0.5° under load (MEASURED) — the leg sags
   further into the lean, not back toward vertical.
3. **The roll mirrors in sign and is larger for +λ** (−3.2/−4.2 vs +2.3 at 10°; −4.8 vs +3.8 at 15°), MEASURED
   in the IMU's own roll convention.

**The sign assumption (σ).** Whether the IMU's roll, in its own sign convention, adds to or subtracts from the leg's
lean cannot be fixed from these data. Assumption A (of record here): the body rolls *into* the lean, so roll adds
— supported by (i) the gravity moment on leaned legs tips the body that way and the ABAD (the 40 N·m pin) is what
holds it, (ii) the measured tracking error is also into the lean, (iii) a feedback that reduces the lean command in
proportion to the roll is what one would build to compensate a roll that adds. None of the three is a direct sign
calibration. **Bench item for next session:** tilt the body a known way at standstill and read the IMU roll sign
against the ABAD γ sign — settles σ in five minutes and also fixes the IMU frame question of `corgi-imu-frames-and-attitude-signs`.
Both signs are reported below; the alternative changes a by ×2.1.

## 5. Per-cell medians over surviving strides (protocol: < 5 survivors → not estimable)

| cell | survivors (per run) | median κ (IQR), m⁻¹ | median λ_eff (A) | median λ_cmd | verdict |
|---|---|---|---|---|---|
| λ 0 | 27 (4 + 23) | **−0.060** (−0.111 … +0.001) | +0.66 | +0.05 | estimable (RA2 alone: −0.049, n 23; RA1 alone n 4: not estimable) |
| +10 | 6 (4 + 2) | +0.082 (−0.115 … +0.129) | +12.0 | +8.0 | **pre-saturation window too short**: the pooled cell reaches 6 only by adding two arcs, neither arc has 5, and all six are strides 1–4 (launch transient). The letter of the rule allows the median; the substance does not — it is not a camber number |
| −10 | 13 | **−0.308** (−0.412 … −0.174) | −11.2 | −8.3 | estimable |
| +15 | 1 | — | — | — | **pre-saturation window too short, slope not estimable** |
| −15 | 10 | **−0.447** (−0.513 … −0.387) | −16.7 | −12.5 | estimable |

Two-point slopes from the estimable cells against λ 0 (medians): −10 → 0.021 m⁻¹ per λ_eff degree, −15 → 0.022;
per commanded degree 0.030 and 0.031. **On the −λ side, pre-saturation, the per-stride response is linear in
λ (−15/−10 = 1.45 in κ against 1.49 in λ_eff), i.e. the super-linearity §325.34 measured whole-arc (2.20) is
not visible before saturation on the − side** — exploratory, n = 1 arc per cell, medians. The −λ pre-saturation
medians are ~20 % larger in magnitude than the whole-arc κ of the same arcs (−0.308 vs −0.252; −0.447 vs −0.494
is the exception) because curvature decays along the −10 arc (−0.4 … −0.6 early, −0.1 … −0.2 late).

## 6. Pooled fit κ_i = a·λ_eff,i + b·i + c (survivors, n = 57, 7 arcs)

**Result of record (assumption A, roll adds):**

> **a = +0.0168 ± 0.0022 m⁻¹ per degree of measured camber** (OLS SE; run-cluster SE 0.0012; leave-one-run-out
> range 0.0157–0.0175), **b = +0.0123 ± 0.0032 m⁻¹ per stride**, c = −0.204 m⁻¹, R² 0.58, n = 57 strides,
> 7 arcs. ≥ 5 survivors per cell: λ 0 yes, −10 yes, −15 yes, +10 only by pooling two arcs' launch transients, +15 no.

Sensitivity (all on survivors unless marked; a in m⁻¹/deg, ± OLS SE):

| variant | a | n | note |
|---|---|---|---|
| roll SUBTRACTS (alternative σ) | +0.0362 ± 0.0045 | 57 | the sign assumption is a ×2.1 lever |
| no roll term (λ_meas only) | +0.0231 ± 0.0030 | 57 | |
| commanded camber instead of λ_eff | +0.0235 ± 0.0031 | 57 | per commanded-in-bag degree |
| no stride covariate | +0.0170 ± 0.0025 | 57 | b does not move a |
| cells with ≥ 5 survivors only (drops +15) | +0.0175 ± 0.0023 | 56 | |
| runs with ≥ 5 survivors only (L0RA2, OL10_NA2, OL15_NA1) | +0.0180 ± 0.0030 | 46 | no +λ at all, 3 arcs |
| same three runs, commanded camber | +0.0253 ± 0.0041 | 46 | 0.91 × the mirror term per commanded degree (0.0279) |
| survivors minus stride 1 | +0.0209 ± 0.0022 | 50 | |
| survivors minus strides 1–2 | +0.0228 ± 0.0024 | 44 | |
| κ from Vicon Kabsch yaw instead of the gyro | +0.0188 ± 0.0031 | 57 | cross-check |
| xcorr alignment instead of count | +0.0170 ± 0.0021 | 57 | cross-check |
| ALL 114 strides incl. saturated — NOT protocol | +0.0305 ± 0.0036 | 114 | reference only; OL10A2's 1–3 cm hops give κ up to 3.6 |
| ALL strides with Δs ≥ 4 cm — NOT protocol | +0.0268 ± 0.0022 | 108 | reference only |

The stride covariate is real and not small: **b = +0.012 m⁻¹ per stride (3.8 SE)** is a common positive drift of κ
along the arc, present at λ 0 (L0RA2 runs −0.41 → +0.24 over 23 strides) and at −10 (−0.6 → −0.1); over a
21-stride arc it amounts to ~+0.26 m⁻¹, the size of the camber term itself. It does not bias a (0.0170 without it),
but it is the reason a whole-arc κ is a window-dependent number — and a reason the mirror design's common-mode
cancellation only works if both signs share the same drift. Exploratory; belongs beside the drift-stationarity
observations in §325.20's thirds-of-arc table.

## 7. Comparison with the whole-arc mirror-contrast camber term (§325.31)

The mirror term is 0.2318 m⁻¹ at nominal ±10° (OL10_R1 / OL10_NA2), i.e. **0.0232 per nominal degree**. In the
units this analysis measures, the same pair had mean |λ_cmd| 8.32° → **0.0279 per commanded degree**, and mean
|λ_eff| (A) 11.46° → **0.0202 per λ_eff degree**. (The 15° pair, 0.5104 m⁻¹: 0.0416 per commanded, 0.0303 per
λ_eff degree — the super-linear one.)

| units | per-stride pooled fit, survivors | mirror term, same units | ratio | separation |
|---|---|---|---|---|
| per λ_eff degree (A) | 0.0168 ± 0.0022 | 0.0202 | 0.83 | 1.5 OLS SE |
| per commanded degree | 0.0235 ± 0.0031 | 0.0279 | 0.84 | 1.4 OLS SE |
| per nominal degree (commanded fit × 8.32/10) | 0.0196 | 0.0232 | 0.84 | — |

**Do they agree?** In sign and magnitude, yes: the pre-saturation per-stride slope is 0.83–0.84 × the whole-arc
mirror term in every unit system, and the 16 % shortfall is inside the ±12–15 % plant repeatability (§325.37) and
1.4–1.5 OLS SE (the run-cluster SE would call it 3 SE, but 7 clusters cannot support that number). **It is not an
independent confirmation of the mirror**, for a structural reason: the survivors are λ 0 plus the −λ arcs (23 of
the 30 cambered survivors are −λ), and the only +λ contribution is seven launch-transient strides at λ_eff ≈ +11.5
with mean κ ≈ 0, which is exactly what pulls the slope down (drop stride 1 everywhere: 0.0209, 0.90 ×; drop strides
1–2: 0.0228, 0.98 ×). Stated plainly: **the −λ side, pre-saturation, reproduces the whole-arc camber gain to
~10 %; the +λ side cannot be tested this way because it saturates before it has five strides.** That is the
honest per-sign answer §325.25 asked for, and it is the same asymmetry as N_sat(+10) = 3–5 vs N_sat(−10) = 14.

## 8. Labels

- MEASURED: κ_i, Δs_i, Δψ_i, N_sat, survivors, commanded/measured γ, tracking error, IMU roll (in the IMU's own sign), all medians and fits as numbers.
- INFERRED: the count↔xcorr equivalence (verified numerically, mechanism of the 0.09 s not resolved); "k_roll feeds raw IMU roll into the camber command at 0.6" (numerical fit, source not read); the reading that +λ's shortfall is the launch transient (supported by the drop-stride-1/2 variants).
- ASSUMPTION: σ = roll adds to the lean (both signs reported; ×2.1 lever on a).
- SPECULATION: none intended. The linear-on-the-−side observation (§5) and the drift-along-arc term b (§6) are exploratory observations on n = 1 arc per cell, not findings.

Not done, deliberately: no vault note was edited by this step (the log entry and any Open-Issues row are the
parent's to splice); no whole-arc gate was re-scored; nothing was run on the robot, the Orin, the Vicon PC or Webots.
