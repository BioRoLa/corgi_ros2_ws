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

Simulation uses the `dev` defaults `K=(4000.0, 4000.0, 4000.0)` and
`B=(200.0, 200.0, 200.0)`. Hardware uses `K=(2000.0, 100.0, 100.0)` and
`B=(80.0, 10.0, 10.0)` in horizontal/lateral/vertical order. The ROS
parameters are `k`, `k_lateral`, `k_vertical`, `b`, `b_lateral`, and
`b_vertical`.

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

Run only one experiment publisher. Initialization starts immediately after the simulator's active motor directions are received; wait for it to finish before triggering the experiment.

### Trigger the experiment

Click **Start Trigger** in `corgi_panel`, which is started by the Webots launch above.

## Real Hardware

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
