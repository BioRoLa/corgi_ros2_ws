# corgi_stair

Stair climbing algorithms with point cloud processing for Corgi quadruped robot.

## ROS 2 Status: ✅ COMPLETE

This package has been fully converted to ROS 2 Humble.

### What's Converted

- ✅ **Build system**: Migrated from catkin to ament_cmake
- ✅ **Library code**: `PlaneSegmentation`, `leg_info`, and `StairClimb` fully ported to ROS 2
- ✅ **PCL transforms**: Uses official `pcl_ros::transformPointCloud()` API
- ✅ **Headers exported**: All include files properly installed
- ✅ **CMake config**: Package can be found with `find_package(corgi_stair)`
- ✅ **Dependencies**: corgi_walk now available, enabling full stair climbing functionality

### Library Components

#### PlaneSegmentation

Point cloud plane segmentation for stair detection. Uses dependency injection for TF functionality.

**Key Features:**

- ROI filtering of point clouds
- Normal estimation
- Plane grouping (horizontal/vertical)
- Distance-based segmentation
- TF transform support (optional via parameter)

See [USAGE_EXAMPLE.md](USAGE_EXAMPLE.md) for ROS 2 usage patterns.

#### leg_info

Leg kinematic utilities (no ROS dependencies).

#### StairClimb

Complete stair climbing algorithm integrating:

- Plane segmentation for stair detection
- Walk gait trajectories from corgi_walk
- Leg kinematics and trajectory planning
- Bezier curve interpolation for smooth motion

### Not Yet Converted

- ⏳ **Executable nodes**: `stair_main_*.cpp` files still use ROS 1 APIs (need porting when used)

### Dependencies

**ROS 2 Packages:**

- rclcpp
- tf2_ros, tf2_eigen, tf2_geometry_msgs
- sensor_msgs, geometry_msgs, visualization_msgs
- pcl_conversions
- **pcl_ros** (official ROS 2 package - install with `sudo apt install ros-humble-pcl-ros`)
- corgi_msgs
- **corgi_walk** (gait generation library)

**External:**

- Eigen3
- PCL 1.12

### Building

```bash
cd ~/corgi_ws/corgi_ros_ws
colcon build --packages-select corgi_stair
```

### Migration from ROS 1

See [USAGE_EXAMPLE.md](USAGE_EXAMPLE.md) for details on the key API changes, particularly around TF buffer management.
