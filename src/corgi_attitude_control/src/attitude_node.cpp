/**
 * attitude_node.cpp
 *
 * ROS 2 node for corgi_attitude_control.
 *
 * Subscribes:
 *   walk/command    (MotorCmdStamped) — motor commands from event_walk
 *   walk/phase      (Int32)           — 0=STANCE/SWING passthrough, 2=ADJUSTING
 *   walk/swing_mask (Int32MultiArray) — per-leg swing flag: 1=swinging, 0=stance
 *   imu             (ImuStamped)      — orientation + angular velocity
 *   motor/state     (MotorStateStamped) — current joint angles
 *
 * Publishes:
 *   motor/command   (MotorCmdStamped) — relayed or corrected commands
 *   attitude/stable (Bool)            — true when body attitude is within thresholds
 *
 * Behaviour:
 *   walk/phase != 2  → relay walk/command straight to motor/command
 *   walk/phase == 2  → apply IMU-based leg-length correction for stance legs,
 *                      then publish motor/command; publish attitude/stable
 */

#include <cmath>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"

#include "corgi_attitude_control/body_leveling.hpp"

// ── helpers ──────────────────────────────────────────────────────────────────

static void quat_to_rp(double qx, double qy, double qz, double qw,
                       double & roll, double & pitch)
{
    // Standard aerospace ZYX convention (same as Eigen::Quaternion::toRotationMatrix)
    roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
    pitch = std::asin( std::max(-1.0, std::min(1.0, 2.0*(qw*qy - qz*qx))) );
}

// ── node class ───────────────────────────────────────────────────────────────

class AttitudeNode : public rclcpp::Node
{
public:
    explicit AttitudeNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : rclcpp::Node("attitude_node", opts)
    {
        // Parameters
        bool   sim          = this->declare_parameter("sim",          true);
        double BL           = this->declare_parameter("BL",           0.444);
        double BW           = this->declare_parameter("BW",           0.4);
        double stand_height = this->declare_parameter("stand_height", 0.25);
        double roll_thresh  = this->declare_parameter("roll_thresh",  0.052);
        double pitch_thresh = this->declare_parameter("pitch_thresh", 0.052);
        double omega_thresh = this->declare_parameter("omega_thresh", 0.05);
        double max_joint_rate = this->declare_parameter("max_joint_rate", 2.0);  // rad/s
        max_delta_ = max_joint_rate * 0.001;  // per 1-ms tick

        controller_ = std::make_unique<BodyLevelingController>(
            sim, BL, BW, stand_height, roll_thresh, pitch_thresh, omega_thresh);

        // Subscriptions
        sub_cmd_ = this->create_subscription<corgi_msgs::msg::MotorCmdStamped>(
            "walk/command", 5,
            [this](corgi_msgs::msg::MotorCmdStamped::SharedPtr msg){ walk_cmd_ = *msg; });

        sub_phase_ = this->create_subscription<std_msgs::msg::Int32>(
            "walk/phase", 5,
            [this](std_msgs::msg::Int32::SharedPtr msg){ walk_phase_ = msg->data; });

        sub_mask_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "walk/swing_mask", 5,
            [this](std_msgs::msg::Int32MultiArray::SharedPtr msg){
                if (msg->data.size() >= 4)
                    for (int i = 0; i < 4; ++i) swing_mask_[i] = msg->data[i];
            });

        sub_imu_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
            "imu", 5,
            [this](corgi_msgs::msg::ImuStamped::SharedPtr msg){ imu_ = *msg; });

        sub_state_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            "motor/state", 5,
            [this](corgi_msgs::msg::MotorStateStamped::SharedPtr msg){ motor_state_ = *msg; });

        // Publishers
        pub_cmd_    = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 5);
        pub_stable_ = this->create_publisher<std_msgs::msg::Bool>("attitude/stable", 5);

        // 1000 Hz control timer — use get_clock() so use_sim_time is respected
        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::microseconds(1000),
            std::bind(&AttitudeNode::on_timer, this));
    }

private:
    void on_timer()
    {
        double qx = imu_.orientation.x;
        double qy = imu_.orientation.y;
        double qz = imu_.orientation.z;
        double qw = imu_.orientation.w;

        double roll = 0.0, pitch = 0.0;
        // Only compute if we have a valid quaternion (norm ≈ 1)
        double norm2 = qx*qx + qy*qy + qz*qz + qw*qw;
        if (norm2 > 0.5) {
            quat_to_rp(qx, qy, qz, qw, roll, pitch);
        }

        double omega_x = imu_.angular_velocity.x;
        double omega_y = imu_.angular_velocity.y;

        // ── extract walk_cmd_ theta/beta in internal (LegModel) convention ──
        const std::array<const corgi_msgs::msg::MotorCmd *, 4> walk_mods = {
            &walk_cmd_.module_a, &walk_cmd_.module_b,
            &walk_cmd_.module_c, &walk_cmd_.module_d};

        std::array<double, 4> base_theta, base_beta;
        for (int i = 0; i < 4; ++i) {
            base_theta[i] = walk_mods[i]->theta;
            // Undo motor-cmd beta sign flip so controller works in LegModel convention
            base_beta[i] = (i == 0 || i == 3) ? -walk_mods[i]->beta : walk_mods[i]->beta;
        }

        // ── compute target correction delta (zero when not ADJUSTING) ────────
        // The slew is applied only to the correction on top of walk_cmd_, so the
        // swing-leg trajectory is always relayed exactly without any rate limiting.
        // When leaving ADJUSTING, snap correction to zero immediately so no
        // residual offset is applied during the following swing phase.
        if (prev_walk_phase_ == 2 && walk_phase_ != 2) {
            prev_corr_theta_.fill(0.0);
            prev_corr_beta_.fill(0.0);
        }
        prev_walk_phase_ = walk_phase_;

        std::array<double, 4> tgt_corr_theta = {}, tgt_corr_beta = {};
        if (walk_phase_ == 2) {
            auto corrected = controller_->compute(roll, pitch, swing_mask_,
                                                  base_theta, base_beta);
            for (int i = 0; i < 4; ++i) {
                tgt_corr_theta[i] = corrected[0][i] - base_theta[i];
                tgt_corr_beta[i]  = corrected[1][i] - base_beta[i];
            }
        }
        // else: target correction is 0 → slew ramps existing correction back to 0

        // ── slew-rate limit on correction only ───────────────────────────────
        auto slew = [this](double prev, double tgt) -> double {
            double diff = tgt - prev;
            if (diff >  max_delta_) diff =  max_delta_;
            if (diff < -max_delta_) diff = -max_delta_;
            return prev + diff;
        };
        for (int i = 0; i < 4; ++i) {
            prev_corr_theta_[i] = slew(prev_corr_theta_[i], tgt_corr_theta[i]);
            prev_corr_beta_[i]  = slew(prev_corr_beta_[i],  tgt_corr_beta[i]);
        }

        // ── assemble and publish motor command ───────────────────────────────
        auto out_msg = walk_cmd_;  // copy header + gains
        std::array<corgi_msgs::msg::MotorCmd *, 4> out_mods = {
            &out_msg.module_a, &out_msg.module_b,
            &out_msg.module_c, &out_msg.module_d};

        for (int i = 0; i < 4; ++i) {
            double out_theta = base_theta[i] + prev_corr_theta_[i];
            double out_beta  = base_beta[i]  + prev_corr_beta_[i];
            out_mods[i]->theta = out_theta;
            // Re-apply motor-cmd beta sign convention
            out_mods[i]->beta = (i == 0 || i == 3) ? -out_beta : out_beta;
        }
        pub_cmd_->publish(out_msg);

        // ── publish stability flag (only during ADJUSTING) ───────────────────
        if (walk_phase_ == 2) {
            std_msgs::msg::Bool stable_msg;
            stable_msg.data = controller_->is_stable(roll, pitch, omega_x, omega_y);
            pub_stable_->publish(stable_msg);
        }
    }

    // State
    int walk_phase_      = 0;
    int prev_walk_phase_ = 0;
    std::array<int, 4> swing_mask_ = {0, 0, 0, 0};
    // Slewed correction delta added on top of walk_cmd_ (never touches swing trajectory)
    std::array<double, 4> prev_corr_theta_ = {};
    std::array<double, 4> prev_corr_beta_  = {};
    double max_delta_ = 0.002;  // rad/tick, set from param in constructor
    corgi_msgs::msg::MotorCmdStamped      walk_cmd_;
    corgi_msgs::msg::ImuStamped           imu_;
    corgi_msgs::msg::MotorStateStamped    motor_state_;

    std::unique_ptr<BodyLevelingController> controller_;

    rclcpp::Subscription<corgi_msgs::msg::MotorCmdStamped>::SharedPtr   sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr               sub_phase_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr     sub_mask_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr        sub_imu_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr sub_state_;

    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              pub_stable_;

    rclcpp::TimerBase::SharedPtr timer_;
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AttitudeNode>();

    // Only wait for Webots /clock when running in simulation mode.
    // In real-hardware mode (use_sim_time=false) the system wall clock is used
    // and node->now() is already valid — no need to wait.
    bool use_sim_time = false;
    node->get_parameter_or("use_sim_time", use_sim_time, false);

    if (use_sim_time) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (node->now().seconds() > 0.0) {
                RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_INFO(node->get_logger(), "Real hardware mode: using system wall clock.");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
