#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

#include "corgi_event_walk/event_walk_gait.hpp"

class EventWalkExpNode : public rclcpp::Node
{
public:
    EventWalkExpNode()
    : rclcpp::Node("event_walk_exp")
    {
        bool   sim            = this->declare_parameter("sim", true);
        double velocity       = this->declare_parameter("velocity", 0.1);
        double stand_height   = this->declare_parameter("stand_height", 0.20);
        double step_length    = this->declare_parameter("step_length", 0.2);
        double step_height    = this->declare_parameter("step_height", 0.06);
        double con_bias       = this->declare_parameter("con_bias", 0.0);
        int    sampling_rate  = this->declare_parameter("sampling_rate", 1000);
        int    max_adj_steps  = this->declare_parameter("max_adjust_steps", 1);
        double BL             = this->declare_parameter("BL", 0.444);
        double BW             = this->declare_parameter("BW", 0.4);

        gait_ = std::make_unique<EventWalkGait>(
            sim, velocity, stand_height, step_length, step_height, con_bias,
            sampling_rate, max_adj_steps, BL, BW);

        pub_cmd_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>(
            "motor/command", 10);

        sub_trigger_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
            "trigger", 10,
            [this](const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
                trigger_enable_ = msg->enable;
            });

        sub_state_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            "motor/state", 10,
            [this](const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
                motor_state_ = *msg;
                motor_state_valid_ = true;
            });

        cmd_mods_ = {
            &cmd_msg_.module_a,
            &cmd_msg_.module_b,
            &cmd_msg_.module_c,
            &cmd_msg_.module_d
        };

        for (int i = 0; i < 4; ++i) {
            cmd_mods_[i]->kp_r = 90;
            cmd_mods_[i]->ki_r = 0;
            cmd_mods_[i]->kd_r = 1.75;
            cmd_mods_[i]->kp_l = 90;
            cmd_mods_[i]->ki_l = 0;
            cmd_mods_[i]->kd_l = 1.75;
            cmd_mods_[i]->torque_r = 0;
            cmd_mods_[i]->torque_l = 0;
        }

        int period_us = 1000000 / sampling_rate;
        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::microseconds(period_us),
            std::bind(&EventWalkExpNode::on_timer, this));

        RCLCPP_INFO(
            this->get_logger(),
            "event_walk_exp started: stand_height=%.3f step_length=%.3f step_height=%.3f velocity=%.3f",
            stand_height, step_length, step_height, velocity);
    }

private:
    void on_timer()
    {
        EventWalkGait::ExternalInput input;
        input.trigger = trigger_enable_;

        // Pure open-loop event-walk experiment:
        // skip attitude waiting so the gait keeps progressing.
        input.attitude_stable = true;

        // No contact feedback in this experiment.
        input.contact = {false, false, false, false};

        input.motor_state_valid = motor_state_valid_;
        if (motor_state_valid_) {
            const std::array<const corgi_msgs::msg::MotorState*, 4> smods = {
                &motor_state_.module_a,
                &motor_state_.module_b,
                &motor_state_.module_c,
                &motor_state_.module_d
            };
            for (int i = 0; i < 4; ++i) {
                input.motor_theta[i] = smods[i]->theta;
                input.motor_beta[i]  = smods[i]->beta;
            }
        }

        auto out = gait_->step(input);

        cmd_msg_.header.stamp = this->now();
        for (int i = 0; i < 4; ++i) {
            cmd_mods_[i]->theta = out.theta[i];
            cmd_mods_[i]->beta  = (i == 0 || i == 3) ? -out.beta[i] : out.beta[i];
        }

        pub_cmd_->publish(cmd_msg_);
    }

    bool trigger_enable_{false};
    bool motor_state_valid_{false};

    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::MotorCmdStamped cmd_msg_;

    std::array<corgi_msgs::msg::MotorCmd*, 4> cmd_mods_{};

    std::unique_ptr<EventWalkGait> gait_;

    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr pub_cmd_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr sub_trigger_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr sub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventWalkExpNode>();

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
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}