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
 *   /ekf            (Odometry)        — optional orientation + angular velocity
 *   motor/state     (MotorStateStamped) — current joint angles
 *
 * Publishes:
 *   motor/command   (MotorCmdStamped) — relayed or corrected commands
 *   attitude/stable (Bool)            — true after debounced attitude stability
 *
 * Behaviour:
 *   walk/phase != 2  → relay walk/command straight to motor/command
 *   walk/phase == 2  → apply IMU-based leg-length correction for stance legs,
 *                      then publish motor/command; publish attitude/stable
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"

#include "corgi_attitude_control/body_leveling.hpp"
#include "corgi_utils/leg_model.hpp"

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
        double stand_height = this->declare_parameter("stand_height", 0.20);
        double roll_thresh  = this->declare_parameter("roll_thresh",  0.026);
        double pitch_thresh = this->declare_parameter("pitch_thresh", 0.026);
        double omega_thresh = this->declare_parameter("omega_thresh", 0.1);
        double max_joint_rate = this->declare_parameter("max_joint_rate", 2.0);  // rad/s
        attitude_source_ = this->declare_parameter("attitude_source", std::string("ekf"));
        stable_debounce_count_ = this->declare_parameter("stable_debounce_count", 30);
        adjust_min_ticks_ = this->declare_parameter("adjust_min_ticks", 30);
        nominal_stand_height_ = stand_height;
        max_delta_ = max_joint_rate * 0.001;  // per 1-ms tick

        if (attitude_source_ != "imu" && attitude_source_ != "ekf") {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid attitude_source='%s' (valid: imu|ekf), fallback to 'imu'",
                attitude_source_.c_str());
            attitude_source_ = "imu";
        }

        controller_ = std::make_unique<BodyLevelingController>(
            sim, BL, BW, stand_height, roll_thresh, pitch_thresh, omega_thresh);
        leg_model_ = std::make_unique<LegModel>(sim);

        hip_x_[0] =  BL / 2.0;  hip_y_[0] =  BW / 2.0;
        hip_x_[1] =  BL / 2.0;  hip_y_[1] = -BW / 2.0;
        hip_x_[2] = -BL / 2.0;  hip_y_[2] = -BW / 2.0;
        hip_x_[3] = -BL / 2.0;  hip_y_[3] =  BW / 2.0;

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

        sub_ekf_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ekf", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){ ekf_odom_ = *msg; });

        sub_state_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            "motor/state", 5,
            [this](corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
                motor_state_ = *msg;
                motor_state_valid_ = true;
            });

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
        double omega_x = imu_.angular_velocity.x;
        double omega_y = imu_.angular_velocity.y;

        if (attitude_source_ == "ekf") {
            qx = ekf_odom_.pose.pose.orientation.x;
            qy = ekf_odom_.pose.pose.orientation.y;
            qz = ekf_odom_.pose.pose.orientation.z;
            qw = ekf_odom_.pose.pose.orientation.w;
            omega_x = ekf_odom_.twist.twist.angular.x;
            omega_y = ekf_odom_.twist.twist.angular.y;
        }

        double roll = 0.0, pitch = 0.0;
        // Only compute if we have a valid quaternion (norm ≈ 1)
        double norm2 = qx*qx + qy*qy + qz*qz + qw*qw;
        if (norm2 > 0.5) {
            quat_to_rp(qx, qy, qz, qw, roll, pitch);
        }

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

        // During ADJUSTING, close the loop around actual motor angles.  The walk
        // command is a nominal touchdown pose; after attitude correction starts,
        // the measured pose is the correct linearisation point for the next IK
        // correction.
        std::array<double, 4> feedback_theta = base_theta;
        std::array<double, 4> feedback_beta  = base_beta;
        if (motor_state_valid_) {
            const std::array<const corgi_msgs::msg::MotorState *, 4> state_mods = {
                &motor_state_.module_a, &motor_state_.module_b,
                &motor_state_.module_c, &motor_state_.module_d};
            for (int i = 0; i < 4; ++i) {
                feedback_theta[i] = state_mods[i]->theta;
                feedback_beta[i]  = (i == 0 || i == 3) ? -state_mods[i]->beta : state_mods[i]->beta;
            }
        }

        // ── compute target correction delta (zero when not ADJUSTING) ────────
        // The slew is applied only to the correction on top of walk_cmd_, so the
        // swing-leg trajectory is always relayed exactly without any rate limiting.
        // When leaving ADJUSTING, snap correction to zero immediately so no
        // residual offset is applied during the following swing phase.
        const bool entering_adjusting = (prev_walk_phase_ != 2 && walk_phase_ == 2);
        const bool leaving_adjusting  = (prev_walk_phase_ == 2 && walk_phase_ != 2);
        if (leaving_adjusting) {
            prev_corr_theta_.fill(0.0);
            prev_corr_beta_.fill(0.0);
            stable_count_ = 0;
            adjust_tick_ = 0;
            adjust_reference_valid_ = false;
        }
        prev_walk_phase_ = walk_phase_;

        std::array<double, 4> tgt_corr_theta = {}, tgt_corr_beta = {};
        if (walk_phase_ == 2) {
            if (entering_adjusting || !adjust_reference_valid_) {
                latch_adjust_reference(roll, pitch, swing_mask_,
                                       feedback_theta, feedback_beta);
            }
            adjust_tick_++;
            auto corrected = compute_balanced_leveling(
                roll, pitch, swing_mask_, feedback_theta, feedback_beta);
            for (int i = 0; i < 4; ++i) {
                // Use the measured pose as the IK linearisation point, but
                // command an absolute correction relative to the gait command.
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

        // ── publish debounced stability flag ─────────────────────────────────
        std_msgs::msg::Bool stable_msg;
        if (walk_phase_ == 2) {
            const bool raw_stable = controller_->is_stable(roll, pitch, omega_x, omega_y);
            if (raw_stable) {
                stable_count_ = std::min(stable_count_ + 1, stable_debounce_count_);
            } else {
                stable_count_ = 0;
            }
            stable_msg.data =
                (adjust_tick_ >= adjust_min_ticks_) &&
                (stable_count_ >= stable_debounce_count_);
        } else {
            stable_count_ = 0;
            adjust_tick_ = 0;
            stable_msg.data = false;
        }
        pub_stable_->publish(stable_msg);
    }

    double tilt_delta_h(int leg, double roll, double pitch) const
    {
        return hip_y_[leg] * std::sin(roll) * std::cos(pitch)
             - hip_x_[leg] * std::sin(pitch);
    }

    double leg_depth(double theta, double beta)
    {
        leg_model_->forward(theta, beta);
        return -leg_model_->G[1];
    }

    double clamp_depth(double depth) const
    {
        return std::max(0.12, std::min(0.34, depth));
    }

    void latch_adjust_reference(
        double roll, double pitch,
        const std::array<int, 4>    & swing_mask,
        const std::array<double, 4> & theta,
        const std::array<double, 4> & beta)
    {
        std::array<double, 4> raw_ref = adjust_level_depth_ref_;
        double mean_ref = 0.0;
        int stance_count = 0;

        for (int i = 0; i < 4; ++i) {
            if (swing_mask[i] == 1) continue;

            leg_model_->forward(theta[i], beta[i]);
            adjust_level_x_ref_[i] = leg_model_->G[0];
            const double current_depth = -leg_model_->G[1];
            raw_ref[i] = current_depth - tilt_delta_h(i, roll, pitch);
            mean_ref += raw_ref[i];
            stance_count++;
        }

        if (stance_count == 0) {
            adjust_reference_valid_ = false;
            return;
        }

        mean_ref /= static_cast<double>(stance_count);
        for (int i = 0; i < 4; ++i) {
            if (swing_mask[i] == 1) continue;

            const double relative_offset = raw_ref[i] - mean_ref;
            adjust_level_depth_ref_[i] =
                clamp_depth(nominal_stand_height_ + relative_offset);
        }
        adjust_reference_valid_ = true;
    }

    std::array<std::array<double, 4>, 2> compute_balanced_leveling(
        double roll, double pitch,
        const std::array<int, 4>    & swing_mask,
        const std::array<double, 4> & current_theta,
        const std::array<double, 4> & current_beta)
    {
        std::array<double, 4> theta_out = current_theta;
        std::array<double, 4> beta_out  = current_beta;

        for (int i = 0; i < 4; ++i) {
            if (swing_mask[i] == 1) continue;

            const double target_x = adjust_level_x_ref_[i];
            const double target_depth = clamp_depth(
                adjust_level_depth_ref_[i] - tilt_delta_h(i, roll, pitch));

            leg_model_->forward(current_theta[i], current_beta[i]);
            const double G0_x = leg_model_->G[0];
            const double G0_y = leg_model_->G[1];
            const double dG_x_desired = target_x - G0_x;
            const double dG_y_desired = -target_depth - G0_y;

            const double eps = 1e-5;
            leg_model_->forward(current_theta[i] + eps, current_beta[i]);
            const double dGx_dth = (leg_model_->G[0] - G0_x) / eps;
            const double dGy_dth = (leg_model_->G[1] - G0_y) / eps;

            leg_model_->forward(current_theta[i], current_beta[i] + eps);
            const double dGx_dbe = (leg_model_->G[0] - G0_x) / eps;
            const double dGy_dbe = (leg_model_->G[1] - G0_y) / eps;

            const double det = dGx_dth * dGy_dbe - dGx_dbe * dGy_dth;
            double d_theta = 0.0;
            double d_beta  = 0.0;
            if (std::abs(det) > 1e-10) {
                d_theta = ( dGy_dbe * dG_x_desired - dGx_dbe * dG_y_desired) / det;
                d_beta  = (-dGy_dth * dG_x_desired + dGx_dth * dG_y_desired) / det;
            } else if (std::abs(dGy_dbe) > 1e-6) {
                d_beta = dG_y_desired / dGy_dbe;
            }

            theta_out[i] = current_theta[i] + d_theta;
            beta_out[i]  = current_beta[i]  + d_beta;
        }

        return {theta_out, beta_out};
    }

    // State
    int walk_phase_      = 0;
    int prev_walk_phase_ = 0;
    std::array<int, 4> swing_mask_ = {0, 0, 0, 0};
    // Slewed correction delta added on top of walk_cmd_ (never touches swing trajectory)
    std::array<double, 4> prev_corr_theta_ = {};
    std::array<double, 4> prev_corr_beta_  = {};
    std::array<double, 4> adjust_level_x_ref_ = {};
    std::array<double, 4> adjust_level_depth_ref_ = {};
    bool adjust_reference_valid_ = false;
    std::array<double, 4> hip_x_ = {};
    std::array<double, 4> hip_y_ = {};
    double nominal_stand_height_ = 0.20;
    double max_delta_ = 0.002;  // rad/tick, set from param in constructor
    int stable_debounce_count_ = 30;
    int adjust_min_ticks_ = 30;
    int stable_count_ = 0;
    int adjust_tick_ = 0;
    std::string attitude_source_ = "imu";
    corgi_msgs::msg::MotorCmdStamped      walk_cmd_;
    corgi_msgs::msg::ImuStamped           imu_;
    nav_msgs::msg::Odometry               ekf_odom_;
    corgi_msgs::msg::MotorStateStamped    motor_state_;
    bool motor_state_valid_ = false;

    std::unique_ptr<BodyLevelingController> controller_;
    std::unique_ptr<LegModel> leg_model_;

    rclcpp::Subscription<corgi_msgs::msg::MotorCmdStamped>::SharedPtr   sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr               sub_phase_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr     sub_mask_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr        sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            sub_ekf_;
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
