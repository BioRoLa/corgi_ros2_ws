# corgi_odometry

`corgi_odometry` provides contact detection and leg odometry estimation for Corgi, including:

- **Online**: real-time ROS 2 nodes (`corgi_contact_leg_est`, `corgi_leg_odom`)
- **Offline**: CSV replay testing (`offline_test`)
- **Simulation helpers**: `velocity_estimator`, `imu_noise_sim`

---

## 1) Build and environment

Run in the workspace root:

```bash
cd ~/corgi_ws/corgi_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select corgi_odometry
source install/setup.bash
```

---

## 2) Launch file usage

### A. Contact estimation only (Online)

```bash
ros2 launch corgi_odometry contact_leg_estimator.launch.py
```

Launched node:
- `corgi_contact_leg_est`

### B. Contact estimation + velocity estimation (Simulation)

```bash
ros2 launch corgi_odometry contact_leg_estimator_sim.launch.py
```

Launched nodes:
- `velocity_estimator`
- `corgi_contact_leg_est`

### C. Leg odometry simulation pipeline (Simulation)

```bash
ros2 launch corgi_odometry leg_odom_sim.launch.py
```

Launched nodes:
- `velocity_estimator`
- `imu_noise_sim`
- `corgi_leg_odom`

### D. Leg odometry (Real robot)

```bash
ros2 launch corgi_odometry leg_odom.launch.py
```

Launched nodes:
- `corgi_leg_odom`

IMU remapping in this launch:
- `/imu` -> `/imu_raw`

---

## 3) Online mode (without launch files)

### Contact estimation node

```bash
ros2 run corgi_odometry corgi_contact_leg_est
```

### Leg odometry node

```bash
ros2 run corgi_odometry corgi_leg_odom
```

Main input topics (shared by both):
- `motor/state`
- `imu`
- `sim/position`
- `sim/velocity`
- `trigger`

Main output topics:
- `corgi_contact_leg_est`: `contact_state`
- `corgi_leg_odom`: `contact_state`, `ekf`

---

## 4) Offline mode (CSV replay)

### Run with default settings

```bash
ros2 run corgi_odometry offline_test
```

### Specify a YAML config

```bash
ros2 run corgi_odometry offline_test --config tuned_0422
```

Note: `--config` expects a **basename (without `.yaml`)**, and the file must be located under `config/`.

### Other common option

```bash
# Disable dynamic dt (use fixed dt instead)
ros2 run corgi_odometry offline_test --config tuned_0422 --fixed-dt
```

### Output files

Offline outputs are written to `output_data/` under your current working directory, for example:

- `<csv_filename>_gmo.csv`
- `<csv_filename>_esekf_dynamic_dt.csv` or `<csv_filename>_esekf_fixed_dt.csv`

---

## 5) Config YAML usage

Config files are located at:

```text
src/corgi_odometry/config/
```

Common files:
- `config_online.yaml` (default for online nodes)
- `config_test.yaml`, `tuned_offline_example.yaml`, `tuned_0422.yaml` (commonly used for offline testing)

### Parameter structure

Main YAML fields:

```yaml
esekf:
  sigma_a: [5.0, 1.0, 1.0]
  sigma_w: [0.001, 0.01, 0.001]
  sigma_ba: [1e-5, 1e-5, 1e-5]
  sigma_bw: [1e-8, 1e-8, 1e-8]
  sigma_leg_vec: [0.05, 1.5, 1.2]
  mahalanobis_threshold: 16.27

observer:
  cutoff_freq: 15.0
  encoder_cutoff_freq: 30.0

contact:
  rm_threshold_high: 25.0
  rm_threshold_low: 15.0
  beta_threshold_high: 10.0
  beta_threshold_low: 1.0

simulate_imu_noise: true
use_esekf_state: true
quiet: true

offline:
  csv_filename: walk_2m_01
  start_index: 0
  max_processed: 12000
  rmse_skip: 0
  enable_logging: true
  log_details: false
  imu_noise_seed: 42
  use_dynamic_dt: true

gt_velocity_lpf_cutoff: 10.0
```

### Online config loading behavior (important)

`corgi_contact_leg_est` and `corgi_leg_odom` currently load this file path in code:

```text
<share>/corgi_odometry/config/config_online.yaml
```

So to tune online parameters, edit `config_online.yaml` (or copy your target config to this filename before running).
