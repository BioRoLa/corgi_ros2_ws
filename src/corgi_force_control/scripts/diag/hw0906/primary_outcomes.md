# Registered primary outcomes of the §325.25 protocol — first formal computation (2026-09-06, session wrap)

Everything below is MEASURED from the local bags and c3d files unless marked INFERRED or SPECULATION.
Analyser: scratchpad `primary_outcomes.py` (run via `run_primary_outcomes.sh`), full output in
`primary_outcomes_out.txt`, per-stride data in `primary_outcomes.json`. Figure:
`Research/SLIP/Figures/primary_outcomes_2026-09-06.png` (generator
`make_figures_2026-09-06_primary_outcomes.py` beside it). Floor **0.257 m/s** (§325.37, 0.85 × mean of the two λ0 arcs at
k_roll 0.60); saturation threshold **39 N·m** on per-stride |τ_h| (§325.25); k_roll 0.60 throughout.

## How the numbers were made (so they can be checked)

- **Stride clock (bag).** l_trend.py's detector: kp_r on `module_a` in `/motor/command` falling below 100, edges inside
  (2.6 s, trigger-off − 0.3 s); stride k = the k-th complete window [edge_k, edge_k+1). The controller's own
  "Stopped after N strides" count is exactly windows + 2 in all 8 runs (the first edge sits at 2.76–2.79 s, and the last
  edge inside 0.3 s of trigger-off is dropped), so **stride k here = the controller's stride k**; the arc simply loses
  its first partial and last partial stride. The kp_r edge sits in flight ~80 ms before touchdown (IMU a_z −9.5 m/s²
  before the edge, −5 to −8.6 after; Vicon trajectory bottom at edge + 0.128 s, spread ≤ 0.010 s over every stride of
  every run). A constant phase offset does not change per-stride displacement.
- **Clock alignment (bag ↔ Vicon).** IMU a_z (band-matched with two 20 ms boxes) cross-correlated against the second
  derivative of the Vicon body-centroid height: 0.25 s RMS envelope for the coarse lag under two non-periodic priors
  (the template starts at 2.6 s in every run; hopping cannot end before trigger-off nor > 1.2 s after), raw signals
  refine within half a stride. Synthetic self-test (known 0.371 s shift) recovered 0.374 s. Audit per run:

  | run | lag (s) | r | envelope margin vs ±1 stride | hop window in bag time | end − trigger-off | edge→bottom median / max |
  |---|---|---|---|---|---|---|
  | s2_l0r_a1 | −4.234 | 0.99 | +0.074 | 2.43–9.30 | +0.22 | 0.129 / 0.137 |
  | s2_l0r_a2 | −2.103 | 0.96 | +0.061 | 2.43–9.59 | +0.21 | 0.128 / 0.133 |
  | s2_ol10_roll1 | −2.279 | 0.99 | +0.060 | 2.43–8.72 | +0.53 | 0.128 / 0.137 |
  | s2_ol10_a2 | −2.618 | 0.90 | +0.058 | 2.44–6.96 | +0.29 | 0.120 / 0.132 |
  | s2_ol10n_a1 | −3.055 | 0.99 | +0.068 | 2.43–7.49 | +0.20 | 0.125 / 0.137 |
  | s2_ol10n_a2 | −14.651 | 0.96 | +0.062 | 2.43–7.73 | +0.16 | 0.127 / 0.138 |
  | s2_ol15_a1 | −2.233 | 0.99 | +0.087 | 2.42–5.89 | +0.16 | 0.124 / 0.137 |
  | s2_ol15n_a1 | −4.457 | 0.98 | +0.066 | 2.43–6.26 | +0.25 | 0.128 / 0.137 |

  All eight windows start at 2.42–2.44 s bag time (the template start) and the edge-to-bottom phase is the same
  constant in every run — the alignment is good to ~10 ms.
- **Per-stride speed.** Vicon body-centroid net horizontal displacement over the bag's stride window ÷ window duration,
  i.e. the registered whole-arc estimator applied stride by stride in the machine's own clock. Self-test: the whole-arc
  estimator (vicon_l0.py) was re-implemented and reproduces all eight logged headline speeds to three digits.
  Cross-check: a bottom-to-bottom hop split (vicon_split.py's window) gives medians within 0.02 m/s of the
  machine-clock medians in 7/8 runs (OL10A2, the decaying arc, 0.121 vs 0.178 because its late partial hops are short).
- **Attitude.** IMU quaternion roll/pitch exactly as l_trend.py computes them, stride means, slope by least squares
  over strides 1..N_sat (registered) and over the whole arc (labelled exploratory). Pitch sign is the IMU's (§325.12:
  the x-mirror of the physical sense).

## Table 1 — N_sat, N_floor, slopes per run (k_roll 0.60, complete strides only)

| run (bag / Vicon) | λ | arc: complete strides / controller count | N_sat (leg, peak N·m) | strides ≥ 39 N·m | N_floor (first stride < 0.257) | below floor sustained from | strides < floor | median per-stride v (m/s) | whole-arc v (m/s, registered) | droll/dstride over 1..N_sat (°/stride, n) | dpitch/dstride over 1..N_sat, IMU (°/stride) | whole-arc droll / dpitch (exploratory) | \|roll\|max (°) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| s2_l0r_a1 / L0_RA1 | 0 | 22 / 24 | 5 (C, 40.0) | 1/22 | 5 | 21 | 4/22 | 0.314 | 0.285 | +0.085 (n=5) | +0.012 (n=5) | −0.008 / −0.233 | 2.12 |
| s2_l0r_a2 / L0RA2 | 0 | 23 / 25 | none (C max 36.0) | 0/23 | none | — | 0/23 | 0.331 | 0.320 | −0.027 (n=23, = whole arc) | −0.051 (n=23) | −0.027 / −0.051 | 1.20 |
| s2_ol10_roll1 / OL10_R1 | +10 | 19 / 21 | 5 (D, 39.8) | 3/19 | none | — | 0/19 | 0.288 | 0.262 | −0.060 (n=5) | −0.189 (n=5) | +0.013 / −0.166 | 4.14 |
| s2_ol10_a2 / OL10A2 | +10 | 13 / 15 | 3 (D, 40.4) | 9/13 | 5 | 5 | 9/13 | 0.178 | 0.168 | −0.316 (n=3, weak) | +0.050 (n=3, weak) | −0.101 / −0.113 | 5.33 |
| s2_ol10n_a1 / OL10_NA1 (rope, not scored) | −10 | 15 / 18 | 10 (C, 39.8) | 2/15 | 6 | 11 | 6/15 | 0.282 | 0.212 | −0.124 (n=10) | −0.385 (n=10) | −0.133 / −0.345 | 4.61 |
| s2_ol10n_a2 / OL10_NA2 | −10 | 16 / 19 | 14 (C, 39.5) | 1/16 | none | — | 0/16 | 0.334 | 0.309 | −0.035 (n=14) | −0.074 (n=14) | −0.030 / −0.059 | 3.82 |
| s2_ol15_a1 / OL15_A1 | +15 | 10 / 12 | 2 (D, 40.6) | 7/10 | 2 | 10 | 6/10 | 0.247 | 0.232 | not estimable (n=2) | not estimable (n=2) | +0.001 / −0.078 | 5.68 |
| s2_ol15n_a1 / OL15_NA1 | −15 | 11 / 13 | 11 (C, 39.4) | 1/11 | none | — | 0/11 | 0.314 | 0.273 | −0.086 (n=11, = whole arc) | −0.129 (n=11) | −0.086 / −0.129 | 5.54 |

Per-leg arc maxima of |τ_h| (N·m), A/B/C/D: L0_RA1 21.8/27.5/40.0/35.6 · L0RA2 20.6/23.0/36.0/26.9 · OL10_R1
30.7/16.3/37.3/39.8 · OL10A2 26.3/18.9/37.4/40.8 · OL10_NA1 21.1/28.4/39.8/28.1 · OL10_NA2 19.2/34.0/39.5/27.5 ·
OL15_A1 29.9/20.9/38.0/41.0 · OL15_NA1 21.5/29.2/39.4/23.5.

## Table 2 — per |λ| per sign

| \|λ\| | sign | clean arcs (n) | N_sat per arc (leg) | N_floor per arc, first / sustained | whole-arc v | clears 0.257 | median per-stride v | reading |
|---|---|---|---|---|---|---|---|---|
| 0 | — | 2 | 5 (C), none | 5 / 21, none / none | 0.285, 0.320 | 2/2 | 0.314, 0.331 | baseline; N_floor-as-registered fires in the control itself |
| 10 | + | 2 | 5 (D), 3 (D) | none / none, 5 / 5 | 0.262, 0.168 | 1/2 | 0.288, 0.178 | not reproducible: one arc holds every stride, the repeat collapses from stride 5 |
| 10 | − | 1 (+1 rope) | 14 (C) [rope: 10 (C)] | none / none [rope: 6 / 11] | 0.309 [rope 0.212] | 1/1 | 0.334 [rope 0.282] | holds every stride |
| 15 | + | 1 | 2 (D) | 2 / 10 | 0.232 | 0/1 | 0.247 | fails: 6/10 strides below, D pinned from stride 2 |
| 15 | − | 1 | 11 (C) | none / none | 0.273 | 1/1 | 0.314 | holds every stride |

**λ_sustain per sign (registered item 4, whole-arc ≥ 0.257):**
- **+λ: 0°** on the strict reading (no |λ| > 0 cell passes in every arc); +10° passed 1 of 2 (0.262, 0.168), +15° 0 of 1.
  Reported as "10° not reproducible", n = 2 at 10°, n = 1 at 15°.
- **−λ: 15°** — the largest |λ| tested; n = 1 at 15° (0.273) and n = 1 clean at 10° (0.309). The rope-contaminated −10
  arc (0.212) is excluded; had it counted, −10 would read 1 of 2.
- Consistent with the 18 % +/− speed asymmetry of §325.34; per stride the asymmetry is sharper: the −λ medians
  (0.334, 0.314) sit at the λ0 level (0.314, 0.331), the +λ medians (0.288, 0.178, 0.247) are 11–45 % lower.

## What the registered outcomes say

1. **N_sat is sign-asymmetric, and the asymmetry is in which leg pins.** Under +λ the rear-left ABAD (D) crosses 39 N·m
   by stride 2–5 in all three arcs and stays there (3/19, 9/13, 7/10 strides ≥ 39; D arc-max 39.8–41.0). Under −λ the
   mirror leg (C, rear-right) only grazes 39.4–39.5 once, late (stride 11–14, 1/11 and 1/16 strides) — which is exactly
   the λ0 behaviour (L0_RA1: C touches 40.0 once at stride 5; L0RA2 never). So N_sat(+10) = 3–5 ≠ N_sat(−10) = 14: the
   registered "if they differ, that is the result" case (§325.25). Note C is the heavy leg in every configuration
   (37–40 N·m arc-max in all 8 runs, §325.27), while D is heavy only under +λ (23–36 N·m otherwise). MEASURED.
2. **N_floor as registered (first stride below the floor) fires in the λ0 control** (L0_RA1 stride 5: 0.206, 0.209 on
   strides 5–6, then recovery to 0.33), because per-stride speed scatters about ±15 % around its median. The
   informative version is the sustained one: the three arcs that fail (OL10A2 from stride 5, OL15_A1 from stride 10
   with 6/10 below, rope arc from 11) never recover; the five that pass never have more than 4/22 strides below.
   Both variants are reported. MEASURED; the interpretation that "first-below" is a noisy statistic is INFERRED from the
   control.
3. **Slopes over 1..N_sat are mostly not estimable as registered**: N_sat ≤ 5 in every +λ arc and in L0_RA1, so the
   pre-saturation window has 2–5 points. Where it is ≥ 10 points (the −λ arcs, L0RA2) the roll slope is −0.03 to
   −0.09 °/stride, i.e. the lean shrinks slightly over the arc — no roll divergence, consistent with §325.24–26.
   |roll| never exceeded 5.7° (abort rule 8° never fired). The lean scales with |λ|: stride-mean roll ≈ 2–4° at 10°,
   3.6–4.9° at 15°, sign following the camber sign. IMU pitch slopes are −0.05 to −0.23 °/stride whole-arc in every
   clean run including λ0 (−0.35 in the rope arc);
   **§325.29 found the IMU over-reports pitch change 2–3.7× against Vicon**, so these are upper bounds and the true
   rate is likely < 0.1 °/stride. (§325.29's own IMU drift figures were computed with a different window; the ratio
   finding stands, the numbers are not directly comparable.)
4. **The registered whole-arc estimator reads below the per-stride median of the same arc in all eight runs** — by
   0.01–0.04 m/s in the seven clean arcs (e.g. OL10_R1 0.262 vs 0.288, OL15_NA1 0.273 vs 0.314) and 0.07 in the rope
   arc, whose end-collapse the median hides — because the hop window carries ~0.17 s of the launch
   lunge and 0.16–0.53 s of post-trigger settle at near-zero speed. The absolute dilution is similar for every arc, so
   it costs short arcs more (the 10–11-stride ±15 arcs) than the 22–23-stride λ0 arcs. The registered scoring stands
   (registered gates bind); this is a stated systematic on cross-cell comparisons, not a rescore. INFERRED.

## Caveats

- n = 1 per cell except λ0 and +10 (n = 2); the plant's own repeat spread is 12 % in whole-arc speed (§325.37).
- Only OL10_R1 reached the 21-stride run length; the other cambered arcs stopped at 12–19 controller strides with
  |roll| ≤ 5.7°, so the abort rule did not end them and the reason is not in the bag (operator stop or capture volume —
  check the runsheet). Nothing here assumes a reason. The λ0 arcs ran 24–25.
- The rope arc is shown for completeness only; every summary excludes it.
- Stride boundaries are the controller's kp_r edges, which precede touchdown by ~80 ms; the strides are therefore the
  controller's strides, not touchdown-to-touchdown. Displacement per full period is insensitive to this.
- SPECULATION, not tested: the D-pins-under-+λ / C-heavy-everywhere pattern would follow from a standing right-side
  load bias (the λ0 drift is −κ in both k_roll 0.60 arcs); item (7) of the bench list (lateral play A/C vs B/D) and a
  static load sweep are the tests.
