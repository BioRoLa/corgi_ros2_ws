# corgi_virtual_agent - ROS2 Conversion

## Overview

Mock FPGA driver for testing `corgi_ros_bridge` without hardware. This package has been converted to ROS2 but requires gRPC dependencies.

## ROS2 Conversion Status

✅ **Converted to ROS2** - Ready to build once dependencies are available

### Changes Made:

- Updated `package.xml` to format 3 with ament_cmake
- Converted `CMakeLists.txt` to use ament_cmake
- Updated C++ code to use rclcpp instead of roscpp
- Changed ROS initialization from `ros::init()` to `rclcpp::init()`
- Changed shutdown from `ros::shutdown()` to `rclcpp::shutdown()`
- Changed ok check from `ros::ok()` to `rclcpp::ok()`

### Architecture Notes:

This package uses a **custom gRPC-based core library** (`core::NodeHandler`) rather than standard ROS publishers/subscribers. This is intentional - it communicates via gRPC protocol buffers (Motor.pb.h, Power.pb.h, Steering.pb.h) to mock the FPGA driver interface.

## Dependencies

### Required gRPC Libraries

This package requires gRPC and custom core libraries to be installed in `$HOME/corgi_ws/install`:

- Protobuf
- gRPC C++
- Custom core libraries:
  - `libregistration_grpc_proto.a`
  - `libconnection_grpc_proto.a`
  - `libserviceserving_grpc_proto.a`
  - `libstd_grpc_proto.a`
- `grpc_proto_lib`

### Installation Instructions

According to project docs, grpc_core should be installed to `$HOME/corgi_ws/install`. Once installed:

```bash
cd ~/corgi_ws/corgi_ros_ws/
colcon build --packages-select corgi_virtual_agent corgi_ros_bridge
source install/setup.bash
```

## Usage

First, start the `grpccore`.
```bash
# Terminal 1: Start grpccore
grpccore
```

### Launch Virtual Agent

```bash
# Terminal 2: Start virtual agent
ros2 run corgi_virtual_agent corgi_virtual_agent
```

This will:

1. Start the mock FPGA driver
2. Subscribe to command topics via gRPC
3. Echo motor commands to console
4. Publish mock state messages back

### Test with corgi_ros_bridge

```bash
# Terminal 3: Start ROS bridge
ros2 run corgi_ros_bridge corgi_ros_bridge
```

### Test: Publish a command
```bash
# Terminal 4: Publish motor command
ros2 topic pub /motor/command corgi_msgs/msg/MotorCmdStamped "
header:
  seq: 1
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
module_a:
  theta: 1.0
  beta: 0.5
  kp_r: 0.0
  kp_l: 0.0
  ki_r: 0.0
  ki_l: 0.0
  kd_r: 0.0
  kd_l: 0.0
  torque_r: 0.0
  torque_l: 0.0
module_b:
  theta: 1.0
  beta: 0.5
  kp_r: 0.0
  kp_l: 0.0
  ki_r: 0.0
  ki_l: 0.0
  kd_r: 0.0
  kd_l: 0.0
  torque_r: 0.0
  torque_l: 0.0
module_c:
  theta: 1.0
  beta: 0.5
  kp_r: 0.0
  kp_l: 0.0
  ki_r: 0.0
  ki_l: 0.0
  kd_r: 0.0
  kd_l: 0.0
  torque_r: 0.0
  torque_l: 0.0
module_d:
  theta: 1.0
  beta: 0.5
  kp_r: 0.0
  kp_l: 0.0
  ki_r: 0.0
  ki_l: 0.0
  kd_r: 0.0
  kd_l: 0.0
  torque_r: 0.0
  torque_l: 0.0
" -r 10
```
### Test: Echo a state
```bash
# Terminal 5: Echo motor state
ros2 topic echo /motor/state
```
### Check
In terminal 2 (start corgi_virtual_agent), it will keep printing:
```bash
TB_A: (1.0, 0.5);
TB_B: (1.0, 0.5);
TB_C: (1.0, 0.5);
TB_D: (1.0, 0.5);
```

## Related Packages

- `corgi_ros_bridge`: ROS ↔ FPGA gRPC interface
- Both packages share gRPC dependencies and custom core library
