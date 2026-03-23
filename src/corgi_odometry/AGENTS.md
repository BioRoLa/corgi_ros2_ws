# corgi_odometry — Agent Context

> Auto-loaded when working inside `src/corgi_odometry/`.
> Read this before modifying any file in this package.

---

## Package Purpose

ESEKF-based leg odometry for the **Corgi** wheeled-legged robot.  
Estimates body velocity in the body frame by fusing IMU propagation with no-slip contact kinematics from 4 legs (a=LF, b=RF, c=RH, d=LH).  
Has **offline replay mode** (CSV → RMSE) and **online ROS 2 node** mode.

---

## Directory Map

```
config/           YAML parameter files
  config_tuned_v1.yaml   ← ACTIVE best params (use this for tuning)
  config_online.yaml     ← online ROS2 deployment
  config_test.yaml       ← default fallback (offline_test uses this if no --config)
  config_obs_baseline.yaml
  config_analysis_*.yaml

data/             CSV datasets (robot recordings)
  walk_2m_01.csv  ← normal walk (12000 rows)
  walk_obs.csv    ← walk with unexpected obstacle contacts (13159 rows)

include/
  common/Params.hpp       ← all runtime-tunable params (struct corgi::Params)
  common/Config.hpp       ← compile-time constants (DT=0.001, geometry, etc.)
  es_ekf/ESEKF.hpp        ← filter state types, StateIdx enums, NoiseParams, LegUpdateDiag
  estimation/EstimationPipeline.hpp
  kinematic/Leg.hpp, LinkLegModel.hpp, ContactMap.hpp
  offline/OfflineTestNode.hpp, CSVReader.hpp, RmseAccumulator.hpp

src/
  es_ekf/ESEKF.cpp        ← CORE FILTER (predict + update_leg + inject_and_reset)
  estimation/EstimationPipeline.cpp
  kinematic/Leg.cpp, LinkLegModel.cpp
  offline/offline_test.cpp    ← main() entry point
  offline/OfflineTestNode.cpp ← CSV replay loop, RMSE calc, CSV logging
  node/corgi_leg_odom.cpp     ← online ROS2 node
  sim/imu_noise_sim.cpp       ← optional IMU noise injection
```

---

## Filter Architecture

### State Dimensions

18-dim **error state** `δx`, 19-dim **nominal state** (uses quaternion for attitude):

```
StateIdx enum:
  P_IDX  = 0   position       δp  (3)
  V_IDX  = 3   velocity       δv  (3)   ← body frame
  TH_IDX = 6   attitude       δθ  (3)   ← rotation vector
  BA_IDX = 9   accel bias     δba (3)
  BW_IDX = 12  gyro bias      δbw (3)
  BV_IDX = 15  velocity bias  δbv (3)   ← extended, kept near zero
```

### Predict Step (`ESEKF::predict`)
- Uses full-rate IMU at 1000 Hz (`Config::DT = 0.001 s`)
- Velocity propagated in **body frame**: `v_{k+1} = R_Δᵀ v_k + (a_hat + g_body) dt`
- `Fx(V,TH) = R_Δᵀ · [g_body]× · dt` — attitude–velocity coupling via gravity

### Update Step (`ESEKF::update_leg`)
- Observation model: `z_leg = v_body + noise` (no-slip → encoder-derived velocity)
- Only **pitch rate** `w_y` used in PointVelocity; `w_x = w_z = 0` to avoid sagittal-plane coupling noise
- H matrix: `H[0:3, V_IDX] = I₃`, plus `H(0, BW_IDX+1) = +r_c.z()`, `H(2, BW_IDX+1) = -r_c.x()`
- Mahalanobis gate: `D² = yᵀ S⁻¹ y > threshold` → reject (default threshold = 16.27 = χ²(3,p=0.999))
- Joseph-form covariance update for numerical stability

### Sequential multi-leg update
`update_all_legs()` calls `update_leg()` for each contact leg sequentially.  
`dx_` accumulates across legs before `inject_and_reset()` at end of each timestep.

---

## Parameter Guide (`Params.hpp` / YAML keys)

| YAML key | Params field | Default | Notes |
|---|---|---|---|
| `esekf.sigma_a` | `sigma_a` | [5,3,2] | Accel process noise std [m/s²]. σ_a_y↑ → P_vy grows → higher Vy Kalman gain |
| `esekf.sigma_w` | `sigma_w` | [0.001,0.01,0.001] | Gyro process noise |
| `esekf.sigma_ba` | `sigma_ba` | [1e-5,1e-4,5e-6] | Accel bias random walk. σ_ba_y=1e-4 lets ba_y track systematic IMU bias |
| `esekf.sigma_bw` | `sigma_bw` | [1e-8,...] | Gyro bias random walk. Keep tiny — large P_bw_y caused bw_y divergence to 0.017 rad/s |
| `esekf.sigma_leg_vec` | `sigma_leg_vec` | [0.02,1.5,0.8] | Leg measurement noise std [m/s] per axis |
| `esekf.mahalanobis_threshold` | `mahalanobis_threshold` | 16.27 | Set to 1e30 to disable |
| `simulate_imu_noise` | `simulate_imu_noise` | false | Adds Gaussian noise to IMU for robustness testing |
| `use_esekf_state` | `use_esekf_state` | false | Use ESEKF output vs raw kinematics |
| `offline.csv_filename` | `csv_filename` | walk_2m_01 | Name without extension, from `data/` dir |
| `offline.max_processed` | `max_processed` | 12000 | Max rows to process |
| `offline.enable_logging` | `enable_logging` | true | Write 88-col CSV to `output_data/` |

### sigma_leg coupling with Mahalanobis — CRITICAL

`D² = yᵀ S⁻¹ y`, where `S = HPHᵀ + R`. When `S ≈ R` (P small), rejection threshold for axis i is `|innov_i| > sqrt(threshold) × sigma_leg_i`.

| sigma_leg_y | Vy rejection threshold |
|---|---|
| 0.02 | 0.16 m/s (realistic) |
| 0.10 | 0.40 m/s (realistic) |
| 1.5  | **6.05 m/s (never fires)** |

**Current sigma_leg_y=1.5 intentionally disables Vy Mahalanobis.** Reason: zeroing `w_x, w_z` in PointVelocity creates systematic Vy bias ~50–200 mm/s. Tight sigma forces filter toward biased measurement → RMSE worsens. All obstacle contact rejections (walk_obs: 1.8–4.1% per leg) are 100% Vx-driven via sigma_leg_x=0.02.

---

## Build & Run

```bash
# Build (from workspace root)
colcon build --packages-select corgi_odometry

# Offline test — uses config/config_tuned_v1.yaml by default
./build/corgi_odometry/src/offline_test

# Use a specific config (name without .yaml, relative to config/)
./build/corgi_odometry/src/offline_test --config config_tuned_v1

# Positional args (no --config): sigma_ax sleg_x sleg_z maha imu_noise quiet use_esekf
./build/corgi_odometry/src/offline_test 5.0 0.02 0.8 16.27 0 1 1
```

Output when `quiet=false`:
```
Velocity RMSE body(m/s):[vx, vy, vz]
Velocity RMSE total: X m/s
```

Logging CSV written to: `src/output_data/<csv_filename>_esekf.csv` (88 columns including `innov_{x,y,z}_{a,b,c,d}`, `d2_{a,b,c,d}`, `rejected_{a,b,c,d}`).

---

## Tuned Parameters — Current Best (`config_tuned_v1.yaml`)

```yaml
esekf:
  sigma_a: [5.0, 3.0, 2.0]
  sigma_leg_vec: [0.02, 1.5, 0.8]
  sigma_ba: [1.0e-05, 1.0e-04, 5.0e-06]
  mahalanobis_threshold: 16.27
```

| Dataset | Vx | Vy | Vz (mm/s) | Baseline Vy |
|---|---|---|---|---|
| walk_2m_01 | 14.8 | 18.4 | 10.4 | 23.5 (↓22%) |
| walk_obs | 19.7 | 23.0 | 17.2 | 25.5 (↓10%) |

---

## Known Issues / Design Decisions

1. **w_x, w_z zeroed in PointVelocity** — intentional. Full ω×r_c causes Vy/Vz drift because the large y-offset (`LEG_Y_OFFSET=0.193 m`) amplifies roll/yaw rate into false body velocity. Fix would require extending H to include bw_x, bw_z states (currently not observable).

2. **bv (velocity bias) excluded from H** — intentional. Including bv in H creates unobservable direction (bv indistinguishable from v under static contact) → rank deficiency → divergence.

3. **P_bw initialized to 1e-10 (not 1e-4)** — past bug: large P_bw_y (~1e-4) allowed early bw_y to drift to 0.017 rad/s, corrupting attitude. Never increase sigma_bw or init P_bw without checking attitude trajectory.

4. **GT velocity from central differencing + LPF** — `gt_velocity_lpf_cutoff=10 Hz` smooths position-differentiated ground truth. RMSE is against this filtered GT, not raw sim velocity.

5. **Observation model limitation** — Leg FK only observes Vy through `ω_y × r_cz` (pitch coupling). Unmodeled `(ω_x × r_c)_y` and `(ω_z × r_c)_y` create ~50-200 mm/s systematic Vy bias. Proper fix: use full angular velocity in observation model + add bw_x, bw_z to H.

---

## Analysis Notebooks

`esekf_state_analysis.ipynb` — Jupyter notebook for:
- Comparing predict-only vs full-ESEKF RMSE (shows how much measurement update helps)
- P matrix diagonal evolution (covariance growth vs collapse)
- Per-leg rejection statistics from logged CSV
- Grid sweeps over sigma_leg / sigma_ba combinations
