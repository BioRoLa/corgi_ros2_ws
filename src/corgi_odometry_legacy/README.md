# corgi_odometry_legacy

`corgi_odometry_legacy` is the legacy state-estimation package for the CORGI leg-wheel robot. It estimates body velocity, position, and leg contact states from leg kinematics, motor states, and IMU data using a Kullback–Leibler divergence (KLD) information filter. A separate node estimates the body height from the geometry of the contacting legs.

This package is retained for existing MPC workflows, reproduction of earlier experiments, and comparisons with the newer ES-EKF implementation in `corgi_odometry`.

## Method Source

The state-estimation method implemented in this package is based on:

> *應用於輪足複合平台之狀態估測器*, master's thesis.

This cleanup only changes launch files, runtime configuration, build structure, and documentation. It does not modify the KLD filter, information filter, leg kinematics, contact detection, or height-estimation algorithms.

## Nodes

| Executable | Purpose |
|---|---|
| `corgi_odometry_legacy` | Estimates 3D velocity, integrated position, and contact states |
| `corgi_z_position_legacy` | Estimates hip height from contacting legs, leg geometry, and body attitude |

The two nodes should normally run together. `corgi_z_position_legacy` uses the contact states published by `corgi_odometry_legacy`.

## Data Flow

```text
/trigger ─────────────────────────────────────────────────────┐
/motor/state ────────────────┐                                │
                             ▼                                ▼
/imu or /imu/gravity_compensated ─► corgi_odometry_legacy     │
                                      ├─► /odometry/legacy/velocity
                                      ├─► /odometry/legacy/position
                                      └─► /odometry/legacy/contact
                                                    │
/motor/state + /imu + /trigger ─────────────────────┤
                                                    ▼
                                  corgi_z_position_legacy
                                                    │
                                                    └─► /odometry/legacy/z_position_hip
```

## Topics

### `corgi_odometry_legacy`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/trigger` | `corgi_msgs/msg/TriggerStamped` | Starts or stops estimation |
| Input | `/motor/state` | `corgi_msgs/msg/MotorStateStamped` | Motor angles and states for all legs |
| Input | `/imu` | `corgi_msgs/msg/ImuStamped` | Real-robot IMU data |
| Input | `/imu/gravity_compensated` | `corgi_msgs/msg/ImuStamped` | Simulation IMU data |
| Output | `/odometry/legacy/velocity` | `geometry_msgs/msg/Vector3` | Estimated body velocity |
| Output | `/odometry/legacy/position` | `geometry_msgs/msg/Vector3` | Integrated body position |
| Output | `/odometry/legacy/contact` | `corgi_msgs/msg/ContactStateStamped` | Contact states and KLD scores |

The node selects its IMU input according to `use_sim_time`: `/imu` on the real robot and `/imu/gravity_compensated` in simulation.

### `corgi_z_position_legacy`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Input | `/trigger` | `corgi_msgs/msg/TriggerStamped` | Starts height estimation |
| Input | `/motor/state` | `corgi_msgs/msg/MotorStateStamped` | Leg configuration |
| Input | `/imu` | `corgi_msgs/msg/ImuStamped` | Body attitude |
| Input | `/odometry/legacy/contact` | `corgi_msgs/msg/ContactStateStamped` | Contacting-leg selection |
| Output | `/odometry/legacy/z_position_hip` | `std_msgs/msg/Float64` | Estimated hip height |

## Launch Files

The package provides two launch entry points. Neither launch file publishes `/trigger` automatically.

### Real Robot

```bash
ros2 launch corgi_odometry_legacy legacy_odom_real.launch.py
```

This starts both legacy estimator nodes. The IMU driver, motor driver, and `/trigger` publisher must be started separately.

To record estimator inputs and outputs:

```bash
ros2 launch corgi_odometry_legacy legacy_odom_real.launch.py \
  record_bag:=true record_delay:=3.0
```

| Argument | Default | Description |
|---|---:|---|
| `record_bag` | `false` | Runs `legacy_odom_bag.sh` |
| `record_delay` | `3.0` | Delay before starting bag recording, in seconds |

### Simulation

```bash
ros2 launch corgi_odometry_legacy legacy_odom_sim.launch.py
```

The simulator must provide `/clock`, `/imu/gravity_compensated`, `/imu`, `/motor/state`, and `/trigger`.

For deterministic validation with a bag that preserves matching motor and IMU sequence numbers and timestamps:

```bash
ros2 launch corgi_odometry_legacy legacy_odom_sim.launch.py \
  deterministic_replay:=true
```

Deterministic replay pairs motor and IMU messages by sequence number and timestamp. It also publishes:

- `/validation/legacy/velocity_stamped`
- `/validation/legacy/position_stamped`

Do not enable this option during normal simulation or real-robot operation.

## Runtime Configuration

ROS runtime configuration is stored in:

```text
config/config_online.yaml
```

It currently contains only the existing `deterministic_replay` switch. A launch argument overrides the YAML default.

The KLD threshold, estimator rate, window size, position frame, and height-estimation method retain their original compile-time defaults in `include/corgi_odometry.hpp`. These values directly affect algorithm behavior and matrix dimensions, so this cleanup does not convert them into runtime parameters.

## Bag Recording

`legacy_odom_bag.sh` records:

- `/trigger`
- `/motor/state`
- `/imu`
- `/odometry/legacy/velocity`
- `/odometry/legacy/position`
- `/odometry/legacy/contact`
- `/odometry/legacy/z_position_hip`

By default, bags are written under the package source tree's `bag/` directory. An explicit output path can also be supplied:

```bash
bash install/corgi_odometry_legacy/share/corgi_odometry_legacy/script/legacy_odom_bag.sh \
  /path/to/output_bag
```

## Integration with corgi_mpc

The `state_source:=odom_legacy` mode in `corgi_mpc` consumes:

- `/odometry/legacy/position`
- `/odometry/legacy/velocity`
- `/odometry/legacy/z_position_hip`

Some `corgi_mpc` launch files start both legacy executables directly, so renaming the package-level launch files does not affect those workflows.

## Build

```bash
cd ~/corgi_ws/corgi_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select corgi_odometry_legacy --symlink-install
source install/setup.bash
```

Inspect the available launch arguments with:

```bash
ros2 launch corgi_odometry_legacy legacy_odom_real.launch.py --show-args
ros2 launch corgi_odometry_legacy legacy_odom_sim.launch.py --show-args
```

## Notes

- The package does not publish `/trigger` automatically.
- Both nodes use ROS time for their processing loops; simulation requires a valid `/clock` source.
- `corgi_z_position_legacy` requires `/odometry/legacy/contact` and should normally run with the main legacy odometry node.
- Prefer `corgi_odometry` for new experiments; this package is maintained for compatibility and comparison.
