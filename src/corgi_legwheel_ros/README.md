# corgi_legwheel_ros

ROS 2 adapter package for integrating the external LegWheel library into the Corgi ROS 2 workspace.

## What this package does

- Keeps `legwheel` as an external library (via git submodule at `src/vendor/LegWheel`)
- Provides ROS 2 node entrypoints that call into `legwheel`
- Starts with a smoke-test node to validate import and FK execution

## Prerequisites

1. Install LegWheel into the same Python environment used by ROS 2:

```bash
pip install -e /root/corgi_ws/corgi_ros2_ws/src/vendor/LegWheel
```

2. Build this package:

```bash
cd /root/corgi_ws/corgi_ros2_ws
colcon build --packages-select corgi_legwheel_ros --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run corgi_legwheel_ros legwheel_smoke
```

or

```bash
ros2 launch corgi_legwheel_ros legwheel_smoke.launch.py
```

## Next step suggestions

- Add a ROS 2 service wrapper for IK/FK conversion
- Add a topic bridge from gait commands to LegWheel planners
- Add test vectors from existing `examples/` scripts in LegWheel
