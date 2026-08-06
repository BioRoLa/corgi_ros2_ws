# corgi_gait_generate

Collection of gait generation packages for the Corgi quadruped. Each subpackage targets a locomotion mode (legged, wheeled, hybrid, transform) or utilities for wheel–leg experiments. All code is ROS 2 (Humble) and publishes motor commands in theta–beta coordinates (`MotorCmdStamped`).

## Subpackages

- `corgi_legged`: legged gait generator.
- `corgi_wheeled`: wheeled gait generator.
- `corgi_hybrid`: hybrid (wheeled-legged) gait generator and tests.
- `corgi_transform`: transformations between modes.
- `wheeled_control`: ROS 2 nodes for joystick/control of wheeled mode (wheel/steer commands).
- `wlw_gen`: wheel–leg–walk experimental motor command publishers (standalone tests).
- `wlw_transform`: offline/compute utilities for wheel–leg trajectory/angle transforms.

## Runtime interfaces (common)

- Publishes `corgi_msgs/msg/MotorCmdStamped` to `/motor/command`.
- Subscribes to `corgi_msgs/msg/MotorStateStamped` from `/motor/state` for feedback (where applicable).
- Uses theta–beta with the 17° offset per robot convention.

## How to run (examples)

```bash
# Legged/hybrid/wheeled generators are launched by their packages’ launch files (see each subpackage).

# Wheeled control (joystick + motor command pipeline)
ros2 run wheeled_control wheel_control_node

# Wheeled control utilities
ros2 run wheeled_control wheeled_state_publisher   # simple joystick state helper
ros2 run wheeled_control wheeled_display           # console subscriber/logger
ros2 run wheeled_control wheeled_simple            # scripted steering test

# Wheel–leg walk experiments
ros2 run wlw_gen wlw
ros2 run wlw_gen wlw_test

# Wheel–leg transform utilities (no ROS topics)
ros2 run wlw_trans wlw_trans
ros2 run wlw_trans turn
```

## Notes

- These generators depend on `corgi_gait_selector` (FSM) and `corgi_utils` for kinematics and shared helpers.
- “copy” source files in some subpackages are legacy/unused and are not built.
- For simulation, ensure the rest of the stack (`corgi_sim`, motor bridge, etc.) is running and topics `/motor/command` and `/motor/state` are available. Still untested.
