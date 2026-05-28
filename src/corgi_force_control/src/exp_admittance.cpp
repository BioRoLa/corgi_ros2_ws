#include <iostream>
#include <array>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_force_control/force_control.hpp"

class ExpAdmittanceNode : public rclcpp::Node {
public:
    ExpAdmittanceNode()
        : Node("exp_admittance"),
          sim_(false),
          trigger_(false),
          mg_(23.66 * 9.81)
    {
        RCLCPP_INFO(this->get_logger(), "Admittance Experiment Node Starts");

        this->get_parameter_or("use_sim_time", sim_, false);
        kinematics_ = std::make_unique<KinematicsHelper>(sim_);

        if (sim_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Webots clock...");
            while (rclcpp::ok()) {
                rclcpp::spin_some(this->get_node_base_interface());
                if (this->now().seconds() > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Real hardware mode: using system wall clock.");
        }

        imp_cmd_pub_ = this->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>(
            "impedance/command", 10);

        trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
            "trigger", 10,
            std::bind(&ExpAdmittanceNode::trigger_cb, this, std::placeholders::_1));

        motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            "motor/state", 10,
            std::bind(&ExpAdmittanceNode::motor_state_cb, this, std::placeholders::_1));

        imp_cmd_modules_ = {
            &imp_cmd_.module_a,
            &imp_cmd_.module_b,
            &imp_cmd_.module_c,
            &imp_cmd_.module_d
        };

        initialize_impedance_command();
    }

    void run() {
        execute_transform();

        rclcpp::Duration period(0, 1000000);
        rclcpp::Time next_time = this->now();

        RCLCPP_INFO(this->get_logger(), "Waiting for trigger to start admittance phase...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (trigger_) {
                execute_admittance_phase();
                break;
            }
            next_time += period;
            this->get_clock()->sleep_until(next_time);
        }
    }

private:
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
        trigger_ = msg->enable;
    }

    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
        motor_state_ = *msg;
    }

    void initialize_impedance_command() {
        for (auto& cmd : imp_cmd_modules_) {
            cmd->theta = 17.0 / 180.0 * M_PI;
            cmd->beta  = 0.0;
            cmd->fx    = 0.0;
            cmd->fy    = 0.0;
            cmd->mx    = 0.0;
            cmd->my    = 0.0;
            cmd->bx    = 0.0;
            cmd->by    = 0.0;
            cmd->kx    = 0.0;
            cmd->ky    = 0.0;
        }
    }

    void execute_transform() {
        const double theta_start    = 17.0 / 180.0 * M_PI;
        const double theta_target   = 80.0 / 180.0 * M_PI;
        const double beta_target    = 0.0 / 180.0 * M_PI;
        const int    transform_count = 3000;  // 3 seconds at 1 ms period

        const std::array<double, 4> beta_targets = {
            -beta_target, +beta_target, +beta_target, -beta_target
        };

        rclcpp::Duration period(0, 1000000);
        rclcpp::Time next_time = this->now();

        for (int count = 0; count < transform_count && rclcpp::ok(); ++count) {
            rclcpp::spin_some(this->get_node_base_interface());
            double ratio = static_cast<double>(count) / transform_count;
            for (int i = 0; i < 4; ++i) {
                imp_cmd_modules_[i]->theta = theta_start + ratio * (theta_target - theta_start);
                imp_cmd_modules_[i]->beta  = ratio * beta_targets[i];
            }
            imp_cmd_.header.stamp = this->now();
            imp_cmd_pub_->publish(imp_cmd_);
            next_time += period;
            if (!this->get_clock()->sleep_until(next_time)) {
                RCLCPP_WARN(this->get_logger(), "Sleep until failed!"); break;
            }
        }

        // Ensure final values are exactly at target
        for (int i = 0; i < 4; ++i) {
            imp_cmd_modules_[i]->theta = theta_target;
            imp_cmd_modules_[i]->beta  = beta_targets[i];
        }

        RCLCPP_INFO(this->get_logger(), "Transform complete.");
    }

    void execute_admittance_phase() {
        // ── Admittance parameters ─────────────────────────────────────────────
        const double my_val = 0.5, by_val = 100.0, ky_val = 500.0;
        const double mx_val = 0.5, bx_val = 100.0, kx_val = 2500.0;

        // ── fy step sequence ──────────────────────────────────────────────────
        struct FyStep { int time_ms; double fy_N; const char* label; };
        const std::vector<FyStep> fy_sequence = {
            {    0,  50.0, "baseline"},
            { 3000,  50.0, "step-down"},
            { 6000,  50.0, "return baseline"},
            { 9000,  50.0, "step-down"},
        };
        const int max_count  = 12000;  // 12 s total
        const int fy_ramp_ms = 300;    // initial 0 -> baseline ramp

        const std::array<double, 4> theta_init = {
            motor_state_.module_a.theta, motor_state_.module_b.theta,
            motor_state_.module_c.theta, motor_state_.module_d.theta
        };
        const std::array<double, 4> beta_init = {
            motor_state_.module_a.beta, motor_state_.module_b.beta,
            motor_state_.module_c.beta, motor_state_.module_d.beta
        };

        // Set admittance parameters for all modules (fy starts at 0, ramped below)
        for (int i = 0; i < 4; ++i) {
            imp_cmd_modules_[i]->theta = theta_init[i];
            imp_cmd_modules_[i]->beta  = beta_init[i];
            imp_cmd_modules_[i]->fy   = 0.0;
            imp_cmd_modules_[i]->fx   = 0.0;
            imp_cmd_modules_[i]->mx   = mx_val;
            imp_cmd_modules_[i]->my   = my_val;
            imp_cmd_modules_[i]->bx   = bx_val;
            imp_cmd_modules_[i]->by   = by_val;
            imp_cmd_modules_[i]->kx   = kx_val;
            imp_cmd_modules_[i]->ky   = ky_val;
        }

        rclcpp::Duration period(0, 1000000);
        rclcpp::Time next_time = this->now();

        // ── Initial ramp: 0 → baseline ───────────────────────────────────────
        const double fy_baseline = fy_sequence[0].fy_N;
        RCLCPP_INFO(this->get_logger(),
            "fy ramp: 0 -> %.1f N over %d ms", fy_baseline, fy_ramp_ms);
        for (int r = 0; r < fy_ramp_ms && rclcpp::ok(); ++r) {
            rclcpp::spin_some(this->get_node_base_interface());
            const double fy = fy_baseline * static_cast<double>(r) / fy_ramp_ms;
            for (auto& cmd : imp_cmd_modules_) { cmd->fy = fy; }
            imp_cmd_.header.stamp = this->now();
            imp_cmd_pub_->publish(imp_cmd_);
            next_time += period;
            if (!this->get_clock()->sleep_until(next_time)) break;
        }
        for (auto& cmd : imp_cmd_modules_) { cmd->fy = fy_baseline; }
        RCLCPP_INFO(this->get_logger(), "fy ramp complete. Entering step-sequence loop.");

        // ── Step-sequence loop ────────────────────────────────────────────────
        int    loop_count = 0;
        int    step_idx   = 0;
        double fy_now     = fy_baseline;

        while (rclcpp::ok() && loop_count < max_count) {
            rclcpp::spin_some(this->get_node_base_interface());

            // Advance to next step when time threshold is reached
            if (step_idx + 1 < static_cast<int>(fy_sequence.size()) &&
                loop_count >= fy_sequence[step_idx + 1].time_ms) {
                ++step_idx;
                fy_now = fy_sequence[step_idx].fy_N;
                for (auto& cmd : imp_cmd_modules_) { cmd->fy = fy_now; }
                RCLCPP_INFO(this->get_logger(),
                    "[t=%ds] fy -> %.1f N  [%s]  (dy_ss=%.1f cm)",
                    loop_count / 1000, fy_now, fy_sequence[step_idx].label,
                    (fy_now - fy_baseline) / ky_val * 100.0);
            }

            if (loop_count % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[t=%ds] fy=%.1f N", loop_count / 1000, fy_now);
            }

            imp_cmd_.header.stamp = this->now();
            imp_cmd_pub_->publish(imp_cmd_);

            ++loop_count;
            next_time += period;
            if (!this->get_clock()->sleep_until(next_time)) {
                RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Admittance phase complete.");
    }

    // Kinematics helper
    std::unique_ptr<KinematicsHelper> kinematics_;

    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped           imp_cmd_;
    corgi_msgs::msg::MotorStateStamped             motor_state_;
    std::vector<corgi_msgs::msg::ImpedanceCmd*>    imp_cmd_modules_;

    // ROS2 interfaces
    rclcpp::Publisher<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr  imp_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr    trigger_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;

    // State
    bool   sim_;
    bool   trigger_;
    double mg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExpAdmittanceNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
