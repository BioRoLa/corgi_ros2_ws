# PlaneSegmentation ROS 2 Usage Example

## Overview

The `PlaneSegmentation` class has been refactored for ROS 2 to use **dependency injection** for TF functionality. Instead of storing a `tf2_ros::Buffer` as a member variable (which requires a clock in ROS 2), the buffer is passed as a parameter to methods that need it.

## Why This Design?

In ROS 2, `tf2_ros::Buffer` requires a `rclcpp::Clock::SharedPtr` in its constructor. Since `PlaneSegmentation` is a utility library (not a node), we use dependency injection to allow calling nodes to manage the TF buffer lifecycle.

## Usage in ROS 2 Node

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "plane_segmentation.hpp"

class StairClimbingNode : public rclcpp::Node {
public:
    StairClimbingNode() : Node("stair_climbing") {
        // Initialize TF buffer with node's clock
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize plane segmentation algorithm
        plane_seg_ = std::make_shared<PlaneSegmentation>();

        // Subscribe to point cloud
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "camera/depth/points", 10,
            std::bind(&StairClimbingNode::cloudCallback, this, std::placeholders::_1));
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // Process with plane segmentation - pass tf_buffer to enable transform
        plane_seg_->setInputCloud(cloud, tf_buffer_.get());

        // Segment the planes
        PlaneDistances result = plane_seg_->segment_planes(cloud);

        // Use the results...
        RCLCPP_INFO(this->get_logger(), "Found %zu horizontal planes",
                    result.horizontal.size());
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<PlaneSegmentation> plane_seg_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StairClimbingNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Without TF Transform

If you don't need TF transformation (e.g., point cloud is already in the correct frame), simply pass `nullptr`:

```cpp
plane_seg_->setInputCloud(cloud);  // tf_buffer defaults to nullptr
// or explicitly:
plane_seg_->setInputCloud(cloud, nullptr);
```

## Key Differences from ROS 1

### ROS 1 Pattern (Old)

```cpp
// ROS 1: PlaneSegmentation manages its own tf::TransformListener
PlaneSegmentation plane_seg;
plane_seg.init_tf();  // Creates internal tf listener
plane_seg.setInputCloud(cloud);  // Uses internal tf buffer
```

### ROS 2 Pattern (New)

```cpp
// ROS 2: Node manages tf2_ros::Buffer, library uses it via parameter
tf2_ros::Buffer tf_buffer(node->get_clock());
tf2_ros::TransformListener tf_listener(tf_buffer);
PlaneSegmentation plane_seg;
plane_seg.setInputCloud(cloud, &tf_buffer);  // Dependency injection
```

## Benefits

1. **No clock dependency** - Library doesn't need to know about ROS 2 clocks
2. **Testable** - Easy to test without ROS nodes
3. **Flexible** - Calling code controls TF buffer lifecycle
4. **Clean separation** - Algorithm logic separated from ROS infrastructure
