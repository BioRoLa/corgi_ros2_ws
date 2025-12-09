# corgi_algo

High-level gait orchestration package (currently minimal). This ROS 2 node constructs a `GaitSelector` with parameters and spins; it does not emit gait commands on its own. Individual gaits in corgi_gait_generate should also be able to run without this package.

## Status

- Builds on ROS 2 Humble.
- Depends on `corgi_gait_selector`, `corgi_utils`, `corgi_msgs`, Eigen, and standard ROS interfaces.
- The legacy ROS1 keyboard-driven orchestrator remains removed as it was mostly commented out; only the minimal ROS2 scaffold is active.

## What it does now

- Declares parameters: `default_gait` (`wheeled|legged|hybrid|transform`), `sim` (bool), `com_bias` (m), `pub_rate` (Hz), `body_length`, `body_width`, `body_height`.
- Instantiates `GaitSelector` with those parameters and sets `currentGait/newGait`.
- Logs configuration, then spins.

## Run

```bash
ros2 run corgi_algo algo \
  --ros-args \
  -p default_gait:=wheeled \
  -p sim:=true \
  -p pub_rate:=1000 \
  -p body_length:=0.444 -p body_width:=0.4 -p body_height:=0.2
```

## Future use, maybe:

- If you need a unified orchestrator (mode switching, keyboard/GUI, parameter tweaking), add that logic here using `rclcpp` timers/subscriptions and calls into gait generators.
- Keep mode transitions consistent with `GaitSelector` (WHEELED ↔ LEGGED ↔ HYBRID with TRANSFORM intermediary).
- Always publish motor commands in theta–beta coordinates and respect the 17° offset via `GaitSelector`/`LegModel`.
