# Livox ROS Driver 2 (corgi-humble-mid360)

> **This is a trimmed fork of [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) used as a submodule in [BioRoLa/corgi_ros2_ws](https://github.com/BioRoLa/corgi_ros2_ws).**
>
> Scope reduction from upstream:
> - **ROS2 Humble only** — ROS1 support and all `launch_ROS1/` files removed
> - **Mid360 only** — HAP LiDAR config, launch files, and mixed-device files removed
> - **No `build.sh`** — build directly via `colcon build` in the parent workspace

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS requirements

  * Ubuntu 22.04 for ROS2 Humble

### 1.2 Install ROS2 Humble

Please refer to: [ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommended.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Source setup

This package is intended to be used as a submodule inside `corgi_ros2_ws`. Clone the parent workspace and initialise submodules:

```shell
git clone --recurse-submodules https://github.com/BioRoLa/corgi_ros2_ws.git
```

### 2.2 Build & install Livox-SDK2

Please follow the guidance in [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md).

### 2.3 Build

Build from the workspace root with colcon:

```shell
cd corgi_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2
```

### 2.4 Run Livox ROS Driver 2

```shell
source install/setup.bash
ros2 launch livox_ros_driver2 [launch file]
```

Available launch files (under `launch_ROS2/`):

| launch file name         | Description                                                         |
| ------------------------ | ------------------------------------------------------------------- |
| msg_MID360_launch.py     | Connect to Mid360, publish livox customized pointcloud data         |
| rviz_MID360_launch.py    | Connect to Mid360, publish pointcloud2 format data, autoload RViz2  |

Example:

```shell
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 3. Launch file and livox_ros_driver2 internal parameter configuration instructions

### 3.1 Launch file configuration instructions

Launch files are in the `launch_ROS2/` directory. Edit the parameters at the top of the launch file directly:

### 3.2 Livox ROS Driver 2 internal main parameter configuration instructions

All parameters are set at the top of each launch file. Commonly used parameters:

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Frequency of point cloud publish (Hz). Recommended: 5.0, 10.0, 20.0, 50.0. Maximum: 100.0 Hz. | 10.0    |
| multi_topic  | 0 -- All LiDARs share the same topic<br>1 -- Each LiDAR has its own topic | 0       |
| xfer_format  | 0 -- Livox pointcloud2 (PointXYZRTLT)<br>1 -- Livox customized pointcloud format | 0       |

  **Note :** Other parameters are not suggested to be changed unless fully understood.

***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) format:

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```

2. Livox customized data package format:

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

Customized Point Cloud (CustomPoint) format:

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

## 4. LiDAR config

The Mid360 config file is `config/MID360_config.json`. The `user_config_path` parameter in the launch file points to this file.

Edit the config to set your Mid360's IP address and host IP:

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.12",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**LiDAR configuration parameters:**

| Parameter           | Type   | Description                                                                                   | Default      |
| :------------------ | ------ | --------------------------------------------------------------------------------------------- | ------------ |
| ip                  | String | IP of the Mid360 LiDAR                                                                        | 192.168.1.12 |
| pcl_data_type       | Int    | 1 -- Cartesian coordinate (32 bits)<br>2 -- Cartesian coordinate (16 bits)<br>3 -- Spherical | 1            |
| pattern_mode        | Int    | 0 -- non-repeating scan<br>1 -- repeating scan<br>2 -- repeating scan (low rate)              | 0            |
| extrinsic_parameter |        | roll/pitch/yaw (float), x/y/z (int)                                                           |              |

## 5. Supported LiDAR

* Mid360

## 6. FAQ

### 6.1 No point cloud displayed in RViz?

Check "Global Options - Fixed Frame" in the RViz Display panel. Set it to `livox_frame` and enable the `PointCloud2` display.

### 6.2 Cannot open shared object file "liblivox_sdk_shared.so"?

Add `/usr/local/lib` to `LD_LIBRARY_PATH`:

```shell
# For current terminal only:
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

# Permanently (add to ~/.bashrc):
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
```
