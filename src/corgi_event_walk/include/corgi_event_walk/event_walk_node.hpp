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

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

/**
 * EventWalkNode
 *
 * Event-driven sequential walk gait for the Corgi quadruped.
 *
 * State machine:
 *   INIT → TRANSFORM → READY → WALK_LOOP → END
 *
 * WALK_LOOP iterates the leg sequence [FL(0), RR(2), FR(1), RL(3)]:
 *   For each leg:
 *     SWING      — execute Bezier swing trajectory at 1000 Hz
 *     TOUCHDOWN  — triggered by contact sensor OR duty completion
 *     ADJUSTING  — notify attitude_node (walk/phase=2), wait for stable flag
 *     → next leg
 *
 * Topics published:
 *   walk/command      MotorCmdStamped  — motor commands (relayed by attitude_node)
 *   walk/phase        Int32            — 0=normal, 2=adjusting
 *   walk/swing_mask   Int32MultiArray  — per-leg 1=swing, 0=stance
 *   walk/swing_phase  Int32MultiArray  — same as swing_mask (legacy compat)
 *
 * Topics subscribed:
 *   trigger                       TriggerStamped
 *   motor/state                   MotorStateStamped
 *   imu                           ImuStamped
 *   odometry/legacy/contact       ContactStateStamped
 *   attitude/stable               Bool
 */
class EventWalkNode : public rclcpp::Node
{
public:
    explicit EventWalkNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
    // ── state machine ────────────────────────────────────────────────────────
    enum class Phase {
        INIT,
        TRANSFORM,
        READY,
        PRE_SWING,   // body advances before hind-leg liftoff
        SWING,
        ADJUSTING,
        END
    };

    void on_timer();
    void do_transform();
    void do_pre_swing();
    void start_swing(int leg);
    void do_swing();
    void enter_adjusting();
    void publish_cmd();

    // publish helpers
    void publish_phase(int phase_val);
    void publish_swing_mask();

    // ── parameters ───────────────────────────────────────────────────────────
    bool   sim_;
    double velocity_;
    double stand_height_;
    double step_length_;
    double step_height_;
    int    sampling_rate_;
    int    max_adjust_steps_;

    // ── kinematics ───────────────────────────────────────────────────────────
    std::unique_ptr<LegModel> leg_model_;

    // ── leg state ────────────────────────────────────────────────────────────
    std::array<double, 4> theta_;   // current joint angles (internal convention)
    std::array<double, 4> beta_;

    // ── walk parameters ──────────────────────────────────────────────────────
    // Leg sequence: FL(0) → RR(2) → FR(1) → RL(3)
    static constexpr int LEG_SEQ[4] = {0, 2, 1, 3};
    int seq_idx_;           // index into LEG_SEQ
    int swing_leg_;         // which leg is currently swinging

    static constexpr double SWING_TIME = 0.2;  // 20% of gait cycle
    double dS_{0.0};        // distance per tick = velocity_ / sampling_rate_
    int swing_steps_;       // ticks per swing = SWING_TIME * step_length_ / dS_
    int swing_tick_;        // current tick within swing

    // PRE_SWING: body advances to shift CoM into support triangle before hind liftoff
    // Duration mirrors walk_gait.cpp rest_time = (1 - 4*SWING_TIME)/2 of one step cycle.
    int pre_swing_steps_{0};
    int pre_swing_tick_{0};
    bool needs_motor_sync_{true};  // true = start_swing() should re-sync from motor_state

    std::array<int, 4> swing_mask_;  // 1 = swinging

    // swing trajectory
    SwingProfile swing_profile_;
    std::array<double, 2> p_lo_, p_td_;

    // hip tracking (robot frame: +x=fwd, z=height)
    std::array<std::array<double, 2>, 4> hip_;       // [i] = {x, z}
    std::array<std::array<double, 2>, 4> foothold_;  // [i] = {x, z}

    // Actual step length completed by each leg, recorded at touchdown.
    // Hind legs (2=RR, 3=RL) inherit leg_step_length_[(leg+2)%4] (contralateral front)
    // for both foothold planning and swing_steps_ timing correction.
    std::array<double, 4> leg_step_length_;

    // hip_[] tracks world-frame hip positions continuously;
    // it advances by dS_ every tick so no separate body_x_ estimate is needed.

    // ── TRANSFORM phase ──────────────────────────────────────────────────────
    static constexpr double INIT_THETA = M_PI * 17.0 / 180.0;
    static constexpr double INIT_BETA  = 0.0;
    // Target stand-walk configuration (identical to walk_exp.cpp init_eta:
    // stand_height=0.25, step_length=0.3, swing_time=0.2)
    // Layout: {theta_0, beta_0, theta_1, beta_1, theta_2, beta_2, theta_3, beta_3}
    static constexpr double INIT_ETA[8] = {
        1.857467698281913,   0.4791102940603915,   // FL (0)
        1.6046663223045279,  0.12914729012802004,  // FR (1)
        1.6046663223045279, -0.12914729012802004,  // RR (2)
        1.857467698281913,  -0.4791102940603915    // RL (3)
    };

    int    transform_count_;
    int    transform_tick_;

    // ── external state ───────────────────────────────────────────────────────
    bool trigger_enable_{false};
    bool attitude_stable_{false};
    std::array<bool, 4> contact_{false, false, false, false};
    int adjust_tick_{0};

    // Latest motor state received from motor/state topic.
    // Used in start_swing() to re-sync theta_/beta_ from the actual motor
    // position after attitude_node may have modified them during ADJUSTING.
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

    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr  sub_trigger_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr sub_state_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr       sub_imu_;
    rclcpp::Subscription<corgi_msgs::msg::ContactStateStamped>::SharedPtr sub_contact_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               sub_stable_;

    Phase phase_{Phase::INIT};

    // BL/BW for hip geometry
    double BL_, BW_;
};

#endif // EVENT_WALK_NODE_HPP
