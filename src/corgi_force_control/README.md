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
22.055468 kg robot mass in simulation and 29.6 kg on hardware. The original
`exp_h21_node` retains its original 23.05 kg mass constant.

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
  -p robot_mass:=22.055468 \
  -p horizontal_force_amplitude:=10.0 \
  -p k:=4000.0 -p b:=100.0 \
  -p k_lateral:=4000.0 -p b_lateral:=100.0 \
  -p k_vertical:=4000.0 -p b_vertical:=100.0

# H21 Fy: also override simulation mass, Fy amplitude, and gamma amplitude
ros2 run corgi_force_control exp_h21_fy_node --ros-args \
  -p use_sim_time:=true \
  -p robot_mass:=22.055468 \
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
action. These are feed-forward force experiments: measured force is recorded
but is not fed back by the current controller. Stop immediately if hardware
motion or oscillation grows unexpectedly.

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
