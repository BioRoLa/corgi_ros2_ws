#ifndef EVENT_WALK_NODE_HPP
#define EVENT_WALK_NODE_HPP

#include <array>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/contact_state_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"

#include "corgi_event_walk/event_walk_gait.hpp"

/**
 * EventWalkNode — thin ROS wrapper around EventWalkGait.
 *
 * All gait logic lives in EventWalkGait.  This node assembles ExternalInput
 * from subscriber callbacks, calls gait_.step(), and publishes the outputs.
 *
 * Topics published:
 *   walk/command      MotorCmdStamped  — motor commands (relayed by attitude_node)
 *   walk/phase        Int32            — 0=normal, 2=adjusting
 *   walk/swing_mask   Int32MultiArray  — per-leg 1=swing, 0=stance, -1=not in walk
 *   walk/swing_phase  Int32MultiArray  — same as swing_mask (legacy compat)
 *
 * Topics subscribed:
 *   trigger                       TriggerStamped
 *   motor/state                   MotorStateStamped
 *   imu                           ImuStamped  (unused; kept for remapping compat)
 *   odometry/legacy/contact       ContactStateStamped
 *   attitude/stable               Bool
 */
class EventWalkNode : public rclcpp::Node
{
public:
    explicit EventWalkNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
    void on_timer();

    // ── gait engine ──────────────────────────────────────────────────────────
    std::unique_ptr<EventWalkGait> gait_;

    // ── callback-stored external state ───────────────────────────────────────
    bool trigger_enable_{false};
    bool attitude_stable_{false};
    std::array<bool, 4> contact_{false, false, false, false};
    corgi_msgs::msg::MotorStateStamped motor_state_{};
    bool motor_state_valid_{false};

    // ── motor command buffer ─────────────────────────────────────────────────
    corgi_msgs::msg::MotorCmdStamped cmd_msg_;
    std::array<corgi_msgs::msg::MotorCmd *, 4> cmd_mods_;

    // ── ROS interfaces ───────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr    pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                pub_phase_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr      pub_swing_mask_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr      pub_swing_phase_;

    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr    sub_trigger_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr sub_state_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr        sub_imu_;
    rclcpp::Subscription<corgi_msgs::msg::ContactStateStamped>::SharedPtr sub_contact_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                sub_stable_;
};

#endif // EVENT_WALK_NODE_HPP
