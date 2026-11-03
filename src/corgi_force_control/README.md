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

This command uses the simulation defaults: stiffness `k=4000.0` and damping `b=200.0`. No `k` or `b` arguments are needed.

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
