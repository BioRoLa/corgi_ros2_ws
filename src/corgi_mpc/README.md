# corgi_mpc

Convex MPC walking for the ABAD Corgi. `mpc_walk` takes the leg trajectory from
`corgi_walk`'s `WalkGait`, solves a single-rigid-body MPC (OSQP) for the stance
ground reaction forces, and publishes them as feed-forward forces on
`/impedance/command`. `corgi_force_control` turns that into motor commands.

## Run

Start the simulator (or the real robot's motor driver) first, then:

```bash
# Simulation, legacy odometry
ros2 launch corgi_mpc mpc_walk.launch.py environment:=sim

# Simulation, Webots ground-truth position and velocity
ros2 launch corgi_mpc mpc_walk.launch.py environment:=sim state_source:=sim_driver

# Real robot
ros2 launch corgi_mpc mpc_walk.launch.py environment:=real
```

The robot unfolds from wheel mode over 3 s, then waits for the trigger:

```bash
ros2 topic pub -r 20 /trigger corgi_msgs/msg/TriggerStamped "{enable: true}"
```

The launch file starts legacy odometry (`corgi_odometry_legacy`,
`corgi_z_position_legacy`), `force_estimation_node`, `force_control_node` and
`mpc_walk`; on the real robot also `corgi_imu`'s `imu_node`.

| Argument | Default | Use |
|---|---|---|
| `environment` | `sim` | `sim` or `real` |
| `state_source` | `odom_legacy` | `odom_legacy`, or `sim_driver` (simulation only) |
| `use_sim_time` | `auto` | follows `environment` |
| `params_file` | empty | YAML with `mpc_walk` parameters, under `corgi_mpc: ros__parameters:` |

`state_source:=sim_driver` reads `/tf` (`odom` to `base_link`) and
`sim/body/velocity` from the corgi_sim driver. The Webots floor is not at
z = 0, so the controller locates it once, from the leg kinematics at the first
MPC step while all four feet are down.

## Open loop (walk_open)

`walk_open` sends corgi_walk's `WalkGait` straight to `motor/command` as
position commands (kp 90, kd 1.75), with no MPC and no force control. Start-up
and stop match `mpc_walk`: unfold over 3 s, wait for the trigger, 1 s ramp-up,
walk for `duration`, finish the step, then hold the pose.

```bash
# Simulation (start corgi_sim first)
ros2 launch corgi_mpc walk_open.launch.py environment:=sim gait:=h20_v10

# Real robot (also starts imu, force estimation and legacy odometry for logging)
ros2 launch corgi_mpc walk_open.launch.py environment:=real gait:=h20_v10
```

Then trigger it as above.

| Argument | Default | Use |
|---|---|---|
| `environment` | `sim` | `sim` or `real` |
| `gait` | `h20_v10` | `hXX_vYY`: stand height XX cm, speed YY cm/s |
| `step_length` | 0.2 | hip travel per gait cycle (m) |
| `step_height` | 0.04 | swing height (m); at most stand height - 0.145 |
| `duration` | 20.0 | walking time, 1 s ramp-up included (s) |
| `use_sim_time` | `auto` | follows `environment` |
| `params_file` | empty | YAML under `corgi_walk_open: ros__parameters:` (for example `kp`, `kd`) |

The dev branch's `h20_v10` used a 0.08 m step, which the current leg model
does not allow at 0.20 m stand height (the gait needs stand height >= 0.145 m
foot radius + step height). In Webots, `h20_v10` walks 1.77 m in 20 s
(0.087 m/s).

## mpc_walk parameters

| Parameter | Default | Use |
|---|---|---|
| `stand_height` | 0.25 | hip (ABAD axis) height above the ground (m) |
| `velocity` | 0.1 | cruise speed (m/s) |
| `step_length` | 0.3 | hip travel per gait cycle (m) |
| `step_height` | 0.04 | swing height (m), see the corgi_walk README |
| `duration` | 20.0 | walking time, 1 s ramp-up included (s) |
| `stance.*`, `swing.*` | k 2000, b 200 | impedance about the gait trajectory, world x / y / z |
| `state_source` | `odom_legacy` | as above |
| `position_topic`, `velocity_topic`, `height_topic`, `contact_topic` | legacy odometry topics | state inputs for `odom_legacy` |
| `config_path` | `~/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml` | MPC weights, mass, force limits |

**Stopping.** `WalkGait` advances the swing with the distance walked, so slowing
down keeps a leg in the air longer and at zero speed freezes it there. After
`duration` the robot keeps cruise speed until all four feet are down (3 s
timeout), then holds the last pose with an even share of the weight on each
leg and stance gains.

## Simulation results

Webots, `Corgi_ABAD.wbt`, default parameters (20 s at 0.1 m/s, about 1.95 m
commanded):

| `state_source` | Distance | Sideways | Roll | Pitch |
|---|---|---|---|---|
| `odom_legacy` | 1.96 m | −0.07 m | −2.9° to +4.1° | −1.8° to +2.0° |
| `sim_driver` | 1.57 m | −0.06 m | −1.5° to +2.8° | −1.8° to +1.4° |

Known behaviour:

- Standing on four feet before the walk, the MPC puts the load on one
  diagonal pair with about 30 N of sideways force. `R_weight` 1e-8 leaves the
  forces free; raising it to 1e-5 (with or without an even-share force
  reference) made the robot fall in both modes.
- The legacy height estimate (`corgi_z_position_legacy`) uses the 3D foot model.
  The old 2D rim model read about 2.5 cm low on this foot, and the MPC lifted
  the body until the robot flipped.

None of this has been run on the real robot.
