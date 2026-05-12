#include <iostream>
#include <array>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/force_state_stamped.hpp"
#include "corgi_msgs/msg/gmo_contact_state_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_force_control/force_control.hpp"

// ============================================================
// Admittance Control Experiment Node
//
// Purpose: Validate admittance control — each leg self-adjusts its
//   contact position until the measured contact force matches the
//   desired load-sharing force (robot weight / 4 per leg).
//
// Topic graph:
//   [this node] --impedance/command--> [admittance_control_node]
//   [leg_odom]  --gmo/contact_state--> [this node]
//   [this node] --force/state--------> [admittance_control_node]
//
// Sign convention for force/state.fy (force_estimation output):
//   fy = -force_est(1,0) - mass_leg * gravity_    (gravity_ = -9.81)
//   At equilibrium: force_est(1,0) ≈ -mg/4  =>  fy ≈ +mg/4 + mass_leg*9.81  (POSITIVE)
//
//   => imp_cmd.fy must also be POSITIVE to have force_err ≈ 0 at equilibrium
//   => force_err = imp.fy - force.fy
//   => body lifted: force.fy drops  => force_err > 0  => legs EXTEND  ✓
//   => extra load:  force.fy rises  => force_err < 0  => legs RETRACT ✓
// ============================================================

class ExpAdmittanceNode : public rclcpp::Node {
public:
    ExpAdmittanceNode()
        : Node("exp_admittance"),
          sim_(false),
          trigger_(false),
          admittance_active_(false),
          mg_(23.66 * 9.81),   // robot weight [N] — adjust to actual mass
          mass_leg_(0.68),     // leg module mass [kg] — must match force_estimation.cpp mass_
          gmo_force_sign_(1.0), // sign: force/state.fy = gmo_force_sign_ * rm_force
          rm_force_max_(200.0)   // clamp: ignore rm_force above this value [N]
    {
        RCLCPP_INFO(this->get_logger(), "Admittance Experiment Node Starts");

        this->get_parameter_or("use_sim_time", sim_, false);
        this->get_parameter_or("force_source", force_source_, std::string("estimation"));
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

        if (force_source_ == "gmo") {
            // In GMO mode, this node bridges gmo/contact_state to force/state.
            // Do not run force_estimation_node simultaneously to avoid topic conflicts.
            force_state_pub_ = this->create_publisher<corgi_msgs::msg::ForceStateStamped>(
                "force/state", 10);
        } else {
            RCLCPP_INFO(this->get_logger(), "Force source = estimation: expect force/state from force_estimation_node.");
        }

        trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
            "trigger", 10,
            std::bind(&ExpAdmittanceNode::trigger_cb, this, std::placeholders::_1));

        contact_state_sub_ = this->create_subscription<corgi_msgs::msg::GMOContactStateStamped>(
            "gmo/contact_state", 10,
            std::bind(&ExpAdmittanceNode::contact_state_cb, this, std::placeholders::_1));

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
        execute_initialization_phase();

        rclcpp::Duration period(0, 1000000); // 1ms
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
    // ----------------------------------------------------------
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
        trigger_ = msg->enable;
    }

    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
        motor_state_ = *msg;
    }

    // Convert one module's rm_force to fy with contact guard and saturation.
    // Returns 0.0 if not in contact or value is out of range.
    double safe_fy(const corgi_msgs::msg::GMOContactState& m) const {
        if (!m.contact) return 0.0;
        double f = m.rm_force;
        if (f < 0.0 || f > rm_force_max_) return 0.0;
        return gmo_force_sign_ * f;
    }

    void contact_state_cb(const corgi_msgs::msg::GMOContactStateStamped::SharedPtr msg) {
        contact_state_ = *msg;

        if (force_source_ != "gmo" || !force_state_pub_) {
            return;
        }

        force_state_.header = msg->header;

        if (!admittance_active_) {
            // During init: zero force feedback so force_err = 0 throughout
            force_state_.module_a.fx = 0.0; force_state_.module_a.fy = 0.0;
            force_state_.module_b.fx = 0.0; force_state_.module_b.fy = 0.0;
            force_state_.module_c.fx = 0.0; force_state_.module_c.fy = 0.0;
            force_state_.module_d.fx = 0.0; force_state_.module_d.fy = 0.0;
        } else {
            // Bridge: gmo/contact_state -> force/state (with contact guard + saturation)
            force_state_.module_a.fy = safe_fy(msg->module_a);
            force_state_.module_b.fy = safe_fy(msg->module_b);
            force_state_.module_c.fy = safe_fy(msg->module_c);
            force_state_.module_d.fy = safe_fy(msg->module_d);
            force_state_.module_a.fx = 0.0;
            force_state_.module_b.fx = 0.0;
            force_state_.module_c.fx = 0.0;
            force_state_.module_d.fx = 0.0;
        }

        force_state_pub_->publish(force_state_);
    }

    // ----------------------------------------------------------
    // Set impedance command for initialization.
    // M=B=K=0: admittance_control returns imp_cmd.theta/beta directly
    // => pure position tracking during stand-up, no force feedback.
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

    // ----------------------------------------------------------
    // Phase 1: raise robot to standing height (same as exp_height)
    void execute_initialization_phase() {
        RCLCPP_INFO(this->get_logger(), "Initialization phase: standing up...");

        // Sync starting theta/beta from actual motor state so we don't
        // retract the leg before extending (which would cause the body to drop).
        // Wait a fixed 500 ms for motor/state to arrive (time-based, not value-based,
        // because the sim may legitimately report theta=0.0 at startup).
        for (int wait = 0; wait < 50 && rclcpp::ok(); wait++) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        // Clamp theta to theta_min (17 deg) so legmodel.move() has a valid contact rim.
        // Webots may initialise joints to 0, which is below the physical minimum.
        const double theta_min = 17.0 / 180.0 * M_PI;
        imp_cmd_modules_[0]->theta = std::max(motor_state_.module_a.theta, theta_min);
        imp_cmd_modules_[0]->beta  = motor_state_.module_a.beta;
        imp_cmd_modules_[1]->theta = std::max(motor_state_.module_b.theta, theta_min);
        imp_cmd_modules_[1]->beta  = motor_state_.module_b.beta;
        imp_cmd_modules_[2]->theta = std::max(motor_state_.module_c.theta, theta_min);
        imp_cmd_modules_[2]->beta  = motor_state_.module_c.beta;
        imp_cmd_modules_[3]->theta = std::max(motor_state_.module_d.theta, theta_min);
        imp_cmd_modules_[3]->beta  = motor_state_.module_d.beta;
        RCLCPP_INFO(this->get_logger(), "Init theta from motor/state: A=%.3f B=%.3f C=%.3f D=%.3f",
            imp_cmd_modules_[0]->theta, imp_cmd_modules_[1]->theta,
            imp_cmd_modules_[2]->theta, imp_cmd_modules_[3]->theta);

        LegModel& legmodel = kinematics_->get_leg_model();
        std::array<double, 2> eta;
        double h = 0.24; // target standing height [m]

        // Calculate current height and remaining height to reach target.
        legmodel.contact_map(imp_cmd_modules_[1]->theta, imp_cmd_modules_[1]->beta);
        double current_h = -legmodel.contact_p[1]; // contact_p[1] is negative (downward)
        double delta_h = h - current_h;
        if (delta_h < 0.0) delta_h = 0.0; // already at or above target
        int steps = static_cast<int>(delta_h / 0.0003) + 1; // ~0.3mm per step (5x faster)
        RCLCPP_INFO(this->get_logger(), "Current height=%.4f m, target=%.4f m, steps=%d",
            current_h, h, steps);

        rclcpp::Duration period(0, 1000000); // 1ms
        rclcpp::Time next_time = this->now();

        for (int i = 0; i < steps; i++) {
            rclcpp::spin_some(this->get_node_base_interface());

            eta = legmodel.move(imp_cmd_modules_[1]->theta, imp_cmd_modules_[1]->beta, {0.0, delta_h / steps});

            imp_cmd_modules_[0]->theta = eta[0];
            imp_cmd_modules_[1]->theta = eta[0];
            imp_cmd_modules_[2]->theta = eta[0];
            imp_cmd_modules_[3]->theta = eta[0];

            imp_cmd_modules_[0]->beta = -eta[1];
            imp_cmd_modules_[1]->beta =  eta[1];
            imp_cmd_modules_[2]->beta =  eta[1];
            imp_cmd_modules_[3]->beta = -eta[1];

            imp_cmd_.header.stamp = this->now();
            imp_cmd_pub_->publish(imp_cmd_);

            next_time += period;
            if (!this->get_clock()->sleep_until(next_time)) {
                RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Initialization phase complete. Final height=%.4f m", h);
    }

    // ----------------------------------------------------------
    // Phase 2: admittance control validation
    //   force_estimation sign: fy = -force_est(1,0) - mass_leg * gravity_
    //   gravity_ = -9.81  =>  equilibrium fy ≈ +mg/4 + mass_leg*9.81  (POSITIVE)
    //   => imp_cmd.fy must also be POSITIVE to have force_err ≈ 0 at equilibrium
    void execute_admittance_phase() {
        // Target fy matches force_estimation equilibrium output (POSITIVE).
        // mass_leg_ must match mass_ in force_estimation.cpp (default 0.68 kg).
        const double fy_target = mg_ / 4.0; // + mass_leg_ * 9.81;
        RCLCPP_INFO(this->get_logger(),
            "Admittance phase: fy_target = %.2f N  (mg/4=%.2f + grav_comp=%.2f)",
            fy_target, mg_ / 4.0, mass_leg_ * 9.81);

        // Set gains BEFORE enabling force feedback.
        for (auto& cmd : imp_cmd_modules_) {
            cmd->fy = fy_target;  // POSITIVE: matches force_estimation sign convention
            cmd->fx = 0.0;
            cmd->mx = 0.0;
            cmd->my = 0.0;
            if (sim_) {
                cmd->bx = 200.0;
                cmd->by = 200.0;
            } else {
                cmd->bx = 30.0;   // real hardware: lower gains to avoid oscillation
                cmd->by = 30.0;
            }
            cmd->kx = 50.0;
            cmd->ky = 100.0;
        }

        // Now enable live GMO force feedback
        admittance_active_ = true;

        // Wait 200 ms for force_state to reach steady-state before the filter starts.
        // Without this, force_err starts large (~fy_target) and drives an initial
        // transient leg extension before the filter sees real contact forces.
        RCLCPP_INFO(this->get_logger(), "Waiting 200ms for force_state to stabilise...");
        for (int w = 0; w < 200 && rclcpp::ok(); w++) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }

        rclcpp::Duration period(0, 1000000); // 1ms
        rclcpp::Time next_time = this->now();

        int loop_count = 0;
        const int max_count = 30000; // 30 seconds

        while (rclcpp::ok() && loop_count < max_count) {
            rclcpp::spin_some(this->get_node_base_interface());

            // Log force error periodically
            if (loop_count % 500 == 0) {
                double rm_a = contact_state_.module_a.rm_force;
                double rm_b = contact_state_.module_b.rm_force;
                double rm_c = contact_state_.module_c.rm_force;
                double rm_d = contact_state_.module_d.rm_force;
                if (force_source_ == "gmo") {
                    RCLCPP_INFO(this->get_logger(),
                        "[%d][gmo] rm_force: A=%.2f  B=%.2f  C=%.2f  D=%.2f  (target=%.2f)",
                        loop_count, rm_a, rm_b, rm_c, rm_d, fy_target);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "[%d][estimation] fy_target=%.2f (force/state from force_estimation_node)",
                        loop_count, fy_target);
                }
            }

            imp_cmd_.header.stamp = this->now();
            imp_cmd_pub_->publish(imp_cmd_);

            loop_count++;
            next_time += period;
            if (!this->get_clock()->sleep_until(next_time)) {
                RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Admittance phase complete.");
    }

    // ----------------------------------------------------------
    // Kinematics helper
    std::unique_ptr<KinematicsHelper> kinematics_;

    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped           imp_cmd_;
    corgi_msgs::msg::ForceStateStamped             force_state_;
    corgi_msgs::msg::GMOContactStateStamped        contact_state_;
    corgi_msgs::msg::MotorStateStamped             motor_state_;
    std::vector<corgi_msgs::msg::ImpedanceCmd*>    imp_cmd_modules_;

    // ROS2 interfaces
    rclcpp::Publisher<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr  imp_cmd_pub_;
    rclcpp::Publisher<corgi_msgs::msg::ForceStateStamped>::SharedPtr    force_state_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr    trigger_sub_;
    rclcpp::Subscription<corgi_msgs::msg::GMOContactStateStamped>::SharedPtr contact_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;

    // State
    bool   sim_;
    bool   trigger_;
    bool   admittance_active_; // false during init: publish zero force feedback
    std::string force_source_; // gmo | estimation
    double mg_;               // robot weight [N]
    double mass_leg_;         // leg module mass [kg], must match force_estimation.cpp mass_
    double gmo_force_sign_;   // sign convention: force/state.fy = sign * rm_force (gmo mode)
    double rm_force_max_;     // sanity clamp [N]: reject values above this (gmo mode)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExpAdmittanceNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
