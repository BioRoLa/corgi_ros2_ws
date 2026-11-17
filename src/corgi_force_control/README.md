# Running corgi_force_control

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/corgi_ws/corgi_ros2_ws
colcon build --packages-up-to corgi_force_control corgi_sim corgi_panel
source install/setup.bash
```

Run the following in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
```

## Simulation

### Terminal 1: Start Webots

```bash
ros2 launch corgi_sim Corgi_force_plate_launch.py
```

Start the simulation in Webots before continuing.

### Terminal 2: Start force estimation and force control

```bash
ros2 launch corgi_force_control force_control.launch.py
```

This launch file sets `use_sim_time=true` for both nodes. Rebuild and restart `corgi_sim` together with the control packages so they share the active motor direction config.

### Terminal 3: Start an experiment

```bash
ros2 run corgi_force_control exp_h24_node --ros-args -p use_sim_time:=true
```

Except for the H21 nodes documented below, simulation uses the `dev` defaults
`K=(4000.0, 4000.0, 4000.0)` and `B=(200.0, 200.0, 200.0)`. Hardware uses
`K=(2000.0, 100.0, 100.0)` and `B=(80.0, 10.0, 10.0)` in
horizontal/lateral/vertical order. The ROS parameters are `k`, `k_lateral`,
`k_vertical`, `b`, `b_lateral`, and `b_vertical`.

### Override K and B

Pass gain overrides after `--ros-args` with one `-p` option per parameter.
For simulation, keep `use_sim_time:=true` in the same command:

```bash
ros2 run corgi_force_control exp_h24_node --ros-args \
  -p use_sim_time:=true \
  -p k:=4000.0 -p b:=200.0 \
  -p k_lateral:=4000.0 -p b_lateral:=200.0 \
  -p k_vertical:=4000.0 -p b_vertical:=200.0
```

For hardware, omit `use_sim_time` or set it to `false`:

```bash
ros2 run corgi_force_control exp_h24_node --ros-args \
  -p k:=2000.0 -p b:=80.0 \
  -p k_lateral:=100.0 -p b_lateral:=10.0 \
  -p k_vertical:=100.0 -p b_vertical:=10.0
```

Here `k` and `b` control horizontal X, the `_lateral` parameters control Y,
and the `_vertical` parameters control Z. The experiment reads these values
at startup, so restart the node after changing them. The same parameter names
apply to `exp_h15_node`, `exp_h18_node`, and `exp_h21_node`.

## H21 Fx and Fy experiments

Run exactly one experiment publisher at a time:

| Executable | Position trajectory | Force trajectory |
|---|---|---|
| `exp_h21_node` | Two 216 mm fore-aft sweeps | Baseline Fz; existing Fz excitation during the second sweep |
| `exp_h21_fx_node` | Same two fore-aft sweeps | Fz plus balanced Fx during the second sweep: A/B `+Fx`, C/D `-Fx` |
| `exp_h21_fy_node` | Gamma-only middle -> left -> middle -> right -> middle, repeated twice; commanded beta remains zero | Fz plus balanced Fy during the second cycle: A/D `+Fy`, B/C `-Fy` |

All three nodes default to `K=(4000,4000,4000)` and
`B=(100,100,100)`. Fx and Fy amplitude defaults to 10 N. Fy gamma amplitude
defaults to 5 degrees. `exp_h21_fx_node` and `exp_h21_fy_node` default to a
23.05 kg robot mass in simulation (the four force plates read 226 N in total)
and 29.6 kg on hardware. The original `exp_h21_node` uses 23.05 kg.

### H21 simulation commands

First start Webots and force control in separate terminals as described above.
Then run one of the following commands.

Use the built-in defaults:

```bash
# Original fore-aft and Fz experiment
ros2 run corgi_force_control exp_h21_node --ros-args \
  -p use_sim_time:=true

# Fore-aft experiment with Fx and Fz
ros2 run corgi_force_control exp_h21_fx_node --ros-args \
  -p use_sim_time:=true

# Lateral gamma experiment with Fy and Fz
ros2 run corgi_force_control exp_h21_fy_node --ros-args \
  -p use_sim_time:=true
```

Pass explicit flags to override the defaults:

```bash
# Original H21: override all Cartesian impedance gains
ros2 run corgi_force_control exp_h21_node --ros-args \
  -p use_sim_time:=true \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0

# H21 Fx: also override simulation mass and Fx amplitude
ros2 run corgi_force_control exp_h21_fx_node --ros-args \
  -p use_sim_time:=true \
  -p robot_mass:=23.05 \
  -p horizontal_force_amplitude:=10.0 \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0

# H21 Fy: also override simulation mass, Fy amplitude, and gamma amplitude
ros2 run corgi_force_control exp_h21_fy_node --ros-args \
  -p use_sim_time:=true \
  -p robot_mass:=23.05 \
  -p horizontal_force_amplitude:=10.0 \
  -p gamma_amplitude_deg:=5.0 \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0
```

### H21 real-hardware commands

Start the hardware driver, IMU, force estimation, and force control first. The
experiment nodes default to hardware mode, so `use_sim_time` can be omitted.

Use the built-in defaults:

```bash
ros2 run corgi_force_control exp_h21_node

ros2 run corgi_force_control exp_h21_fx_node

ros2 run corgi_force_control exp_h21_fy_node
```

Pass explicit flags when tuning hardware:

```bash
# Original H21
ros2 run corgi_force_control exp_h21_node --ros-args \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0

# H21 Fx
ros2 run corgi_force_control exp_h21_fx_node --ros-args \
  -p robot_mass:=29.6 \
  -p horizontal_force_amplitude:=10.0 \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0

# H21 Fy
ros2 run corgi_force_control exp_h21_fy_node --ros-args \
  -p robot_mass:=29.6 \
  -p horizontal_force_amplitude:=10.0 \
  -p gamma_amplitude_deg:=5.0 \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0
```

Unspecified parameters keep their defaults, so it is valid to override only
one value, for example `-p gamma_amplitude_deg:=3.0` or
`-p horizontal_force_amplitude:=5.0`.

### Trigger H21 experiments

Initialization begins as soon as the experiment node starts. Wait for it to
finish, start recording if needed, and then trigger the control phase:

```bash
ros2 topic pub --once /trigger corgi_msgs/msg/TriggerStamped "{enable: true}"
```

In simulation, the **Start Trigger** button in `corgi_panel` performs the same
action. These are feed-forward force experiments: `force_control_node` does
not use the force estimate. Stop immediately if hardware motion or
oscillation grows unexpectedly.

## Force control options

`force_control_node` and `force_estimation_node` share a leg model (the force
estimate itself is used by `corgi_mpc`, not by force control). Set the same
values on both nodes:

| Parameter | Simulation default | Hardware default | Meaning |
|---|---|---|---|
| `foot_link` | `F_l` | `axis` | How a foot contact loads the motors (below) |
| `leg_gravity` | `true` | `false` | Compensate the leg's own weight |
| `leg_m_cz` | -0.070 kg m | | leg mass x centre-of-mass height below the hip (beta torque) |
| `leg_m_cz_theta` | -0.105 kg m/rad | | change of that moment with theta (theta torque) |
| `abad_m_d` | -0.150 kg m | | module mass x sideways centre-of-mass offset (ABAD torque) |

**Foot link.** The `LegModel` foot arc is centred on the leg axis (O_r = G + R)
and stays aligned with it. With `foot_link:=axis` that alignment carries no
load, so a foot contact acts on the motors through the O-G axis (the original
Jacobian). corgi_sim holds the foot piece with a servoed joint on one lower
link, so moments about G go through that link instead; `foot_link:=F_l`
reproduces this. With `axis`, the simulated estimate is off by up to 15 N in Fz
whenever beta is not zero (too high on one side of the robot, too low on the
other), and the commanded forces are off the same way. Which link carries the
foot on the hardware has not been measured, so hardware keeps `axis`.

**Leg weight.** Beta turns the whole linkage, so the leg weight puts
`leg_m_cz * g * sin(beta)` on the beta axis and about 1 N m on the theta axis.
The defaults were fitted to the force plates in corgi_sim. With `leg_gravity`
on, the estimator drops its old constant 0.9 kg leg-weight offset. The hardware
model angles have not been checked against this convention, so hardware keeps
`leg_gravity:=false` until it is fitted there.

## Other real-hardware experiments

Start the hardware driver and IMU first. Use separate terminals for the following commands.

### Terminal 1: Start force estimation

```bash
ros2 run corgi_force_estimation force_estimation_node
```

Skip this command if force estimation is already running.

### Terminal 2: Start force control

```bash
ros2 run corgi_force_control force_control_node
```

### Terminal 3: Start the experiment

```bash
ros2 run corgi_force_control exp_h24_node
```

Initialization starts immediately. After initialization finishes, click **Start Trigger** in your running `corgi_panel`.

Direct execution defaults to `use_sim_time=false`, enabling hardware friction compensation. The launch file above is for simulation.

## Stop

Press `Ctrl+C` in each terminal to stop its process. Stopping a process does not issue a motor-disable command; use the hardware driver's disable mechanism to disable the motors.
