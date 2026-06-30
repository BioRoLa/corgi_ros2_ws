#pragma once

#include "fusion/OuterEKF.hpp"
#include "common/Config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <corgi_msgs/msg/trigger_stamped.hpp>

#include <deque>
#include <mutex>

namespace fusion {

/// Ring-buffer entry: snapshot of inner-ESEKF state at a given timestamp
struct BufferedState {
    rclcpp::Time stamp;
    Eigen::Vector3f    p_odom;
    Eigen::Quaternionf q_odom;
};

class FusionNode : public rclcpp::Node {
public:
    explicit FusionNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

private:
    // ── Callbacks ──────────────────────────────────────────────
    void cb_ekf(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cb_lidar(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cb_trigger(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);

    // ── Helpers ────────────────────────────────────────────────
    /// Find the buffered state closest to `stamp`.
    /// Returns false if the buffer is empty or stamp is out of range.
    bool find_buffered_state(const rclcpp::Time& stamp, BufferedState& out) const;

    void publish_odom_mapping(const rclcpp::Time& stamp,
                               const Eigen::Vector3f& p_map,
                               const Eigen::Quaternionf& q_map);

    // ── Members ────────────────────────────────────────────────
    OuterEKF ekf_;
    OuterEKF::NoiseParams noise_params_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;

    bool is_triggered_ = false;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_mapping_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bv_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Ring buffer (max ~0.5 s of inner-ESEKF states)
    mutable std::mutex buffer_mutex_;
    std::deque<BufferedState> state_buffer_;
    static constexpr size_t BUFFER_MAX = 500;  // 0.5 s @ 500 Hz (with some margin)

    // Timing constants
    static constexpr float  DT_LIDAR_DEFAULT_S = 0.1f;   // nominal 10 Hz LiDAR period [s]
    static constexpr double DT_LIDAR_MAX_S      = 5.0;   // sanity cap for dt_lidar [s]
    static constexpr double ACCEPT_WINDOW_S     = 0.5;   // max age for buffered state match [s]

    // Camera-init → odom frame transform (computed at first lidar+EKF match).
    // Converts /lidar_odom (camera_init frame) into odom before updating the
    // map←odom outer EKF.
    bool lidar_frame_init_         = false;
    Eigen::Vector3f    t_co_;          // translation: camera_init origin expressed in odom
    Eigen::Quaternionf q_co_;          // rotation:    camera_init → odom

    // Timing
    rclcpp::Time last_ekf_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lidar_stamp_{0, 0, RCL_ROS_TIME};
    bool last_ekf_stamp_valid_    = false;
    bool last_lidar_stamp_valid_  = false;

    // Frame ids (configurable)
    std::string map_frame_;
    std::string odom_frame_;
};

}  // namespace fusion
