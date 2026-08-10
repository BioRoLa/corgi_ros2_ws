/**
 * event_walk_node.cpp
 *
 * Thin ROS 2 wrapper around EventWalkGait.
 *
 * on_timer() assembles ExternalInput from subscriber callbacks,
 * calls gait_->step(), then publishes the resulting GaitOutput.
 */

#include "corgi_event_walk/event_walk_node.hpp"

// ── constructor ──────────────────────────────────────────────────────────────

EventWalkNode::EventWalkNode(const rclcpp::NodeOptions & opts)
: rclcpp::Node("event_walk_node", opts)
{
    // ── declare parameters ────────────────────────────────────────────────
    bool   sim            = this->declare_parameter("sim",               true);
    double velocity       = this->declare_parameter("velocity",          0.1);
    double stand_height   = this->declare_parameter("stand_height",      0.20);
    double step_length    = this->declare_parameter("step_length",       0.25);
    double step_height    = this->declare_parameter("step_height",       0.06);
    double con_bias       = this->declare_parameter("con_bias",          0.0);
    int    sampling_rate  = this->declare_parameter("sampling_rate",     1000);
    int    max_adj_steps  = this->declare_parameter("max_adjust_steps",  3000);
    double BL             = this->declare_parameter("BL",                0.444);
    double BW             = this->declare_parameter("BW",                0.4);
    std::string contact_source = this->declare_parameter("contact_source", "gmo");
    double probe_speed    = this->declare_parameter("probe_speed",       0.15);
    int    max_probe_steps = this->declare_parameter("max_probe_steps",  500);
    double contact_swing_accept_ratio =
        this->declare_parameter("contact_swing_accept_ratio", 0.5);
    int contact_on_count  = this->declare_parameter("contact_on_count",  5);
    int contact_off_count = this->declare_parameter("contact_off_count", 2);
    double pre_swing_advance_scale =
        this->declare_parameter("pre_swing_advance_scale", 1.5);
    max_cycles_ = this->declare_parameter("max_cycles", 5);

    // ── create gait engine ────────────────────────────────────────────────
    gait_ = std::make_unique<EventWalkGait>(
        sim, velocity, stand_height, step_length, step_height, con_bias,
        sampling_rate, max_adj_steps, BL, BW,
        probe_speed, max_probe_steps, contact_swing_accept_ratio,
        contact_on_count, contact_off_count, pre_swing_advance_scale);

    // ── motor command buffer (gains fixed once) ───────────────────────────
    cmd_mods_ = {&cmd_msg_.module_a, &cmd_msg_.module_b,
                 &cmd_msg_.module_c, &cmd_msg_.module_d};
    for (int i = 0; i < 4; ++i) {
        cmd_mods_[i]->kp_r = 90;   cmd_mods_[i]->kp_l = 90;
        cmd_mods_[i]->ki_r = 0;    cmd_mods_[i]->ki_l = 0;
        cmd_mods_[i]->kd_r = 1.75; cmd_mods_[i]->kd_l = 1.75;
        cmd_mods_[i]->torque_r = 0; cmd_mods_[i]->torque_l = 0;
    }

    // ── publishers ────────────────────────────────────────────────────────
    pub_cmd_         = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>("walk/command", 5);
    pub_phase_       = this->create_publisher<std_msgs::msg::Int32>("walk/phase", 5);
    pub_swing_mask_  = this->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_mask", 5);
    pub_swing_phase_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 5);
    pub_leg_state_   = this->create_publisher<std_msgs::msg::Int32MultiArray>("walk/leg_state", 5);

    // ── subscriptions ─────────────────────────────────────────────────────
    sub_trigger_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 5,
        [this](corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
            trigger_enable_ = msg->enable;
        });

    sub_state_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 5,
        [this](corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
            motor_state_       = *msg;
            motor_state_valid_ = true;
        });

    sub_imu_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 5,
        [this](corgi_msgs::msg::ImuStamped::SharedPtr) {
            // IMU is used by attitude_node; kept here for topic remapping compat.
        });

    if (contact_source == "legacy" || contact_source == "both") {
        sub_contact_ = this->create_subscription<corgi_msgs::msg::ContactStateStamped>(
            "odometry/legacy/contact", 5,
            [this](corgi_msgs::msg::ContactStateStamped::SharedPtr msg) {
                contact_[0] = msg->module_a.contact;
                contact_[1] = msg->module_b.contact;
                contact_[2] = msg->module_c.contact;
                contact_[3] = msg->module_d.contact;
                contact_valid_ = true;
            });
    }

    if (contact_source == "gmo" || contact_source == "both") {
        sub_gmo_contact_ = this->create_subscription<corgi_msgs::msg::GMOContactStateStamped>(
            "/gmo/contact_state", 5,
            [this](corgi_msgs::msg::GMOContactStateStamped::SharedPtr msg) {
                contact_[0] = msg->module_a.contact;
                contact_[1] = msg->module_b.contact;
                contact_[2] = msg->module_c.contact;
                contact_[3] = msg->module_d.contact;
                contact_valid_ = true;
            });
    }

    sub_stable_ = this->create_subscription<std_msgs::msg::Bool>(
        "attitude/stable", 5,
        [this](std_msgs::msg::Bool::SharedPtr msg) {
            attitude_stable_ = msg->data;
        });

    // ── 1000 Hz control timer ─────────────────────────────────────────────
    int period_us = 1000000 / sampling_rate;
    timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        std::chrono::microseconds(period_us),
        std::bind(&EventWalkNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
        "EventWalkNode started (via EventWalkGait). "
        "stand_height=%.3f step_length=%.3f step_height=%.3f velocity=%.3f",
        stand_height, step_length, step_height, velocity);
}

// ── control timer ─────────────────────────────────────────────────────────────

void EventWalkNode::on_timer()
{
    // ── assemble ExternalInput from latest callback data ──────────────────
    EventWalkGait::ExternalInput input;
    input.trigger          = trigger_enable_;
    // Only forward attitude_stable when the gait is actually in ADJUSTING phase.
    // Stale "true" values from a previous ADJUSTING cycle must not pre-trigger
    // the latch on the very first tick of the next adjustment window.
    input.attitude_stable  = gait_->is_adjusting() && attitude_stable_;
    input.contact          = contact_;
    input.contact_enabled  = contact_valid_;
    input.motor_state_valid = motor_state_valid_;

    if (motor_state_valid_) {
        const std::array<const corgi_msgs::msg::MotorState *, 4> smods = {
            &motor_state_.module_a, &motor_state_.module_b,
            &motor_state_.module_c, &motor_state_.module_d};
        for (int i = 0; i < 4; ++i) {
            input.motor_theta[i] = smods[i]->theta;
            input.motor_beta[i]  = smods[i]->beta;  // motor convention (legs 0,3 negated)
        }
    }

    // ── step the gait engine ──────────────────────────────────────────────
    auto out = gait_->step(input);

    // ── publish motor command ─────────────────────────────────────────────
    cmd_msg_.header.stamp = this->now();
    for (int i = 0; i < 4; ++i) {
        cmd_mods_[i]->theta = out.theta[i];
        // Apply motor convention: legs 0,3 (FL,RL) carry negative beta
        cmd_mods_[i]->beta = (i == 0 || i == 3) ? -out.beta[i] : out.beta[i];
    }
    pub_cmd_->publish(cmd_msg_);

    // ── publish walk/phase ────────────────────────────────────────────────
    std_msgs::msg::Int32 phase_msg;
    phase_msg.data = out.phase_int;
    pub_phase_->publish(phase_msg);

    // ── publish walk/swing_mask (-1 when not actively walking) ───────────
    std_msgs::msg::Int32MultiArray mask_msg;
    mask_msg.data.resize(4);
    for (int i = 0; i < 4; ++i)
        mask_msg.data[i] = out.in_walk ? out.swing_mask[i] : -1;
    pub_swing_mask_->publish(mask_msg);
    pub_swing_phase_->publish(mask_msg);   // legacy compat topic

    // Per-leg gait state, ordered FL, FR, RR, RL.
    std_msgs::msg::Int32MultiArray leg_state_msg;
    leg_state_msg.data.assign(out.leg_state.begin(), out.leg_state.end());
    pub_leg_state_->publish(leg_state_msg);

    if (!shutdown_requested_ &&
        max_cycles_ > 0 &&
        gait_->completed_cycles() >= max_cycles_) {
        shutdown_requested_ = true;
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(),
            "Completed %d gait cycles; shutting down event_walk_node.",
            gait_->completed_cycles());
        rclcpp::shutdown();
    }
}
