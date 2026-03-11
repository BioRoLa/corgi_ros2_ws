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

### Required gRPC Libraries (Not Yet Installed)

This package requires gRPC and custom core libraries to be installed in `~/.grpc_local/`:

- Protobuf
- gRPC C++
- Custom core libraries:
  - `libregistration_grpc_proto.a`
  - `libconnection_grpc_proto.a`
  - `libserviceserving_grpc_proto.a`
  - `libstd_grpc_proto.a`
- `grpc_proto_lib`

### Installation Instructions

According to project docs, grpc_core should be installed to `~/.grpc_local`. Once installed:

```bash
cd ~/corgi_ws/corgi_ros_ws/
colcon build --packages-select corgi_virtual_agent
source install/setup.bash
```

## Usage (When Dependencies Available)

### Launch Virtual Agent

```bash
ros2 run corgi_virtual_agent corgi_virtual_agent
```

This will:

1. Start the mock FPGA driver
2. Subscribe to command topics via gRPC
3. Echo motor commands to console
4. Publish mock state messages back

### Test with corgi_ros_bridge

Once both packages are converted and built:

```bash
# Terminal 1: Start virtual agent
ros2 run corgi_virtual_agent corgi_virtual_agent

# Terminal 2: Start ROS bridge (when converted)
ros2 run corgi_ros_bridge corgi_ros_bridge
```

## Related Packages

- `corgi_ros_bridge`: ROS ↔ FPGA gRPC interface (also needs ROS2 conversion)
- Both packages share gRPC dependencies and custom core library

## Future Work

- [ ] Install gRPC dependencies to `~/.grpc_local`
- [ ] Convert `corgi_ros_bridge` to ROS2
- [ ] Test gRPC communication between virtual agent and bridge
- [ ] Create launch file for combined testing
