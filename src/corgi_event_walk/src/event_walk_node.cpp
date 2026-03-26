/**
 * event_walk_node.cpp
 *
 * Implementation of the event-driven sequential walk for the Corgi quadruped.
 *
 * Design summary
 * ──────────────
 * A 1000 Hz wall-timer drives the inner servo loop.  The "events" that advance
 * the outer state machine are:
 *   • SWING → TOUCHDOWN  : swing trajectory finished (duty full) OR contact sensor fires
 *   • TOUCHDOWN → ADJUSTING : entered automatically on touchdown
 *   • ADJUSTING → (next leg) : attitude_node publishes attitude/stable=true
 *                               OR max_adjust_steps exceeded
 *
 * Leg numbering (matches corgi_gait_selector / walk_gait)
 *   0 = FL (+BL/2, +BW/2)
 *   1 = FR (+BL/2, -BW/2)
 *   2 = RR (-BL/2, -BW/2)
 *   3 = RL (-BL/2, +BW/2)
 *
 * Walk sequence: FL(0) → RR(2) → FR(1) → RL(3)
 */

#include "corgi_event_walk/event_walk_node.hpp"

#include <cmath>
#include <algorithm>
#include <string>

// ── constants ────────────────────────────────────────────────────────────────
constexpr int EventWalkNode::LEG_SEQ[4];   // define storage

// Touch-rim search order (same as walk_gait.cpp)
static const std::string TOUCH_RIM_LIST[5] = {"G", "L_l", "L_r", "U_l", "U_r"};
static const int         TOUCH_RIM_IDX[5]  = {  3,     2,     4,     1,    5};

// ── constructor ──────────────────────────────────────────────────────────────

EventWalkNode::EventWalkNode(const rclcpp::NodeOptions & opts)
: rclcpp::Node("event_walk_node", opts)
{
    // ── declare parameters ────────────────────────────────────────────────
    sim_               = this->declare_parameter("sim",               true);
    velocity_          = this->declare_parameter("velocity",          0.1);
    stand_height_      = this->declare_parameter("stand_height",      0.25);
    step_length_       = this->declare_parameter("step_length",       0.3);
    step_height_       = this->declare_parameter("step_height",       0.04);
    sampling_rate_     = this->declare_parameter("sampling_rate",     1000);
    max_adjust_steps_  = this->declare_parameter("max_adjust_steps",  3000);
    BL_                = this->declare_parameter("BL",                0.444);
    BW_                = this->declare_parameter("BW",                0.4);

    leg_model_ = std::make_unique<LegModel>(sim_);

    // dS per tick and swing step count — matches walk_gait.cpp semantics:
    // swing occupies SWING_TIME fraction of one step cycle (step_length / dS ticks total)
    dS_ = velocity_ / static_cast<double>(sampling_rate_);
    swing_steps_ = static_cast<int>(SWING_TIME * step_length_ / dS_);

    transform_count_ = 5 * sampling_rate_;  // 5 s
    transform_tick_  = 0;

    // Initialise hip positions (world frame; hip_[i][0] advances each tick by dS_)
    hip_[0] = { BL_/2.0,  stand_height_};
    hip_[1] = { BL_/2.0,  stand_height_};
    hip_[2] = {-BL_/2.0,  stand_height_};
    hip_[3] = {-BL_/2.0,  stand_height_};

    // Foothold: initially directly below each hip
    for (int i = 0; i < 4; ++i) {
        foothold_[i] = {hip_[i][0], 0.0};
    }

    // Step length per leg — initialised to parameter value; updated at each touchdown.
    leg_step_length_.fill(step_length_);

    // Initialise joint angles to wheel-mode (INIT_THETA, 0)
    theta_.fill(INIT_THETA);
    beta_.fill(INIT_BETA);
    swing_mask_.fill(0);
    seq_idx_    = 0;
    swing_leg_  = LEG_SEQ[0];
    swing_tick_ = 0;

    // Motor command buffer - set shared gains once
    cmd_mods_ = {&cmd_msg_.module_a, &cmd_msg_.module_b,
                 &cmd_msg_.module_c, &cmd_msg_.module_d};
    for (int i = 0; i < 4; ++i) {
        cmd_mods_[i]->kp_r = 90;  cmd_mods_[i]->kp_l = 90;
        cmd_mods_[i]->ki_r = 0;    cmd_mods_[i]->ki_l = 0;
        cmd_mods_[i]->kd_r = 1.75; cmd_mods_[i]->kd_l = 1.75;
        cmd_mods_[i]->torque_r = 0; cmd_mods_[i]->torque_l = 0;
    }

    // ── publishers ────────────────────────────────────────────────────────
    pub_cmd_         = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>("walk/command", 5);
    pub_phase_       = this->create_publisher<std_msgs::msg::Int32>("walk/phase", 5);
    pub_swing_mask_  = this->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_mask", 5);
    pub_swing_phase_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 5);

    // ── subscriptions ─────────────────────────────────────────────────────
    sub_trigger_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 5,
        [this](corgi_msgs::msg::TriggerStamped::SharedPtr msg){
            trigger_enable_ = msg->enable;
        });

    sub_state_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 5,
        [this](corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
            motor_state_       = *msg;
            motor_state_valid_ = true;
        });

    sub_imu_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 5,
        [this](corgi_msgs::msg::ImuStamped::SharedPtr){
            // IMU used by attitude_node, not directly here
        });

    sub_contact_ = this->create_subscription<corgi_msgs::msg::ContactStateStamped>(
        "odometry/legacy/contact", 5,
        [this](corgi_msgs::msg::ContactStateStamped::SharedPtr msg){
            contact_[0] = msg->module_a.contact;
            contact_[1] = msg->module_b.contact;
            contact_[2] = msg->module_c.contact; 
            contact_[3] = msg->module_d.contact; 
        });

    sub_stable_ = this->create_subscription<std_msgs::msg::Bool>(
        "attitude/stable", 5,
        [this](std_msgs::msg::Bool::SharedPtr msg){
            attitude_stable_ = msg->data;
        });

    // ── control timer — use get_clock() so use_sim_time is respected ──────
    int period_us = 1000000 / sampling_rate_;
    timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        std::chrono::microseconds(period_us),
        std::bind(&EventWalkNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
        "EventWalkNode started. stand_height=%.3f step_length=%.3f step_height=%.3f velocity=%.3f",
        stand_height_, step_length_, step_height_, velocity_);
}

// ── control timer callback ────────────────────────────────────────────────────

void EventWalkNode::on_timer()
{
    switch (phase_)
    {
    case Phase::INIT:
        // Immediately enter transform on first tick
        transform_tick_ = 0;
        phase_ = Phase::TRANSFORM;
        publish_phase(0);
        publish_swing_mask();
        break;

    case Phase::TRANSFORM:
        do_transform();
        break;

    case Phase::READY:
        // Hold stand pose, wait for trigger
        publish_phase(0);
        publish_swing_mask();
        publish_cmd();  // keep sending commands so legs don't drift after TRANSFORM
        if (trigger_enable_) {
            RCLCPP_INFO(this->get_logger(), "Trigger received — starting walk sequence.");
            seq_idx_   = 0;
            swing_leg_ = LEG_SEQ[seq_idx_];
            start_swing(swing_leg_);
            phase_ = Phase::SWING;
        }
        break;

    case Phase::PRE_SWING:
        do_pre_swing();
        break;

    case Phase::SWING:
        do_swing();
        break;

    case Phase::ADJUSTING:
        publish_phase(2);
        publish_swing_mask();

        // Body is stationary during ADJUSTING — no hip advancement.
        // attitude_node adjusts leg depths; start_swing() will re-sync
        // from motor_state to plan the next leg based on adjusted pose.

        adjust_tick_++;

        if (attitude_stable_ || adjust_tick_ >= max_adjust_steps_) {
            if (adjust_tick_ >= max_adjust_steps_) {
                RCLCPP_WARN(this->get_logger(),
                    "Attitude adjustment timed out for leg %d", swing_leg_);
            }
            // Advance to next leg in sequence
            seq_idx_ = (seq_idx_ + 1) % 4;
            swing_leg_ = LEG_SEQ[seq_idx_];

            if (seq_idx_ == 0) {
                // Completed one full gait cycle — check stop condition or loop
                RCLCPP_INFO(this->get_logger(), "Completed one gait cycle.");
                if (!trigger_enable_) {
                    RCLCPP_INFO(this->get_logger(), "Trigger released — stopping.");
                    phase_ = Phase::END;
                    publish_phase(0);
                    return;
                }
            }

            attitude_stable_ = false;
            // Hind legs (RR=2, RL=3): advance body into support triangle before liftoff.
            // Front legs can swing immediately — CoM is already supported.
            if (swing_leg_ == 2 || swing_leg_ == 3) {
                const double eff = leg_step_length_[(swing_leg_ + 2) % 4];
                pre_swing_steps_ = static_cast<int>((1.0 - 4.0 * SWING_TIME) / 2.0 * eff / dS_);
                pre_swing_tick_  = 0;
                needs_motor_sync_ = true;   // do_pre_swing() will sync on tick 0
                phase_ = Phase::PRE_SWING;
            } else {
                needs_motor_sync_ = true;   // start_swing() will sync from motor_state
                pre_swing_steps_ = 0;       // no PRE_SWING for front legs
                start_swing(swing_leg_);
                phase_ = Phase::SWING;
            }
        }
        // While adjusting, keep publishing the current hold pose for attitude_node
        publish_cmd();
        break;

    case Phase::END:
        // Hold current position
        publish_phase(0);
        publish_swing_mask();
        publish_cmd();
        break;
    }
}

// ── TRANSFORM phase ───────────────────────────────────────────────────────────

void EventWalkNode::do_transform()
{
    double ratio = static_cast<double>(transform_tick_) / static_cast<double>(transform_count_);
    ratio = std::min(ratio, 1.0);

    for (int i = 0; i < 4; ++i) {
        theta_[i] = INIT_THETA + ratio * (INIT_ETA[i*2] - INIT_THETA);
        // Apply the same beta sign convention as walk_exp.cpp:
        //   walk_exp negates beta for i=0,3 in TRANSFORM; publish_cmd() negates again
        //   → motor receives INIT_ETA[i*2+1] β directly, matching physical hardware.
        double raw_beta = INIT_BETA + ratio * (INIT_ETA[i*2+1] - INIT_BETA);
        beta_[i] = (i == 1 || i == 2) ? raw_beta : -raw_beta;
    }

    transform_tick_++;
    publish_phase(0);
    publish_swing_mask();
    publish_cmd();

    if (transform_tick_ >= transform_count_) {
        // Snap to exact final values
        for (int i = 0; i < 4; ++i) {
            theta_[i] = INIT_ETA[i*2];
            beta_[i]  = (i == 1 || i == 2) ? INIT_ETA[i*2+1] : -INIT_ETA[i*2+1];
        }
        RCLCPP_INFO(this->get_logger(), "TRANSFORM done → READY (waiting for trigger)");
        phase_ = Phase::READY;
    }
}

// ── PRE_SWING phase ──────────────────────────────────────────────────────────

void EventWalkNode::do_pre_swing()
{
    // On first tick: re-sync theta_/beta_ from actual motor state so that the
    // move() accumulation below starts from attitude-adjusted angles.
    // After this point needs_motor_sync_ is cleared so start_swing() will NOT
    // overwrite the angles we accumulate via move() during PRE_SWING ticks.
    if (pre_swing_tick_ == 0 && needs_motor_sync_ && motor_state_valid_) {
        const std::array<const corgi_msgs::msg::MotorState *, 4> smods = {
            &motor_state_.module_a, &motor_state_.module_b,
            &motor_state_.module_c, &motor_state_.module_d};
        for (int i = 0; i < 4; ++i) {
            theta_[i] = smods[i]->theta;
            beta_[i]  = (i == 0 || i == 3) ? -smods[i]->beta : smods[i]->beta;
        }
        needs_motor_sync_ = false;
    }

    // Advance all hips and update stance IK — body moves forward while all 4 legs
    // remain planted, shifting CoM into the front-three support triangle.
    for (int i = 0; i < 4; ++i) hip_[i][0] += dS_;
    for (int i = 0; i < 4; ++i) {
        auto eta = leg_model_->move(theta_[i], beta_[i], {dS_, 0.0});
        theta_[i] = eta[0];
        beta_[i]  = eta[1];
    }
    publish_phase(0);
    publish_swing_mask();
    publish_cmd();
    pre_swing_tick_++;

    if (pre_swing_tick_ >= pre_swing_steps_) {
        start_swing(swing_leg_);  // needs_motor_sync_==false → skip re-sync inside
        phase_ = Phase::SWING;
    }
}

// ── SWING phase ───────────────────────────────────────────────────────────────

void EventWalkNode::start_swing(int leg)
{
    swing_leg_  = leg;
    swing_tick_ = 0;
    swing_mask_.fill(0);
    swing_mask_[leg] = 1;
    attitude_stable_ = false;

    // Re-sync ALL legs' theta_/beta_ from actual motor state only when needed.
    // During ADJUSTING, attitude_node may have modified any leg's angles (depth correction).
    // Syncing all legs ensures stance IK in do_swing() starts from the correct
    // attitude-adjusted angles, not the stale snapshot from the previous touchdown.
    // When called from do_pre_swing() (after PRE_SWING ticks), needs_motor_sync_ is
    // false — the move()-accumulated angles must be preserved as-is.
    if (needs_motor_sync_ && motor_state_valid_) {
        const std::array<const corgi_msgs::msg::MotorState *, 4> smods = {
            &motor_state_.module_a, &motor_state_.module_b,
            &motor_state_.module_c, &motor_state_.module_d};
        for (int i = 0; i < 4; ++i) {
            theta_[i] = smods[i]->theta;
            // Undo beta sign convention: publish_cmd() negates beta for legs 0,3.
            beta_[i] = (i == 0 || i == 3) ? -smods[i]->beta : smods[i]->beta;
        }
        needs_motor_sync_ = false;
    }

    // Liftoff point from FK — matches walk_gait.cpp p_lo calculation
    leg_model_->forward(theta_[leg], beta_[leg]);
    p_lo_ = {hip_[leg][0] + leg_model_->G[0],
              hip_[leg][1] + leg_model_->G[1]};

    // Effective step length for this swing:
    // Hind legs (2=RR, 3=RL) inherit the contralateral front leg's actual step length
    // (mirrors walk_gait.cpp: step_length = current_step_length[(i+2)%4]).
    // swing_steps_ is recomputed here so the Bezier progress rate scales accordingly —
    // this is the event_walk equivalent of walk_gait's incre_duty = dS / step_length.
    const double effective_step_length =
        (leg == 2 || leg == 3) ? leg_step_length_[(leg + 2) % 4] : step_length_;
    swing_steps_ = static_cast<int>(SWING_TIME * effective_step_length / dS_);

    // Foothold world-x: during swing, hip advances by swing_time * effective_step_length.
    // To land at (1-swing_time)/2 * eff ahead of hip AT touchdown,
    // foothold must be (1+swing_time)/2 * eff ahead of hip AT liftoff.
    //
    // CORRECTION for PRE_SWING: hip has already advanced pre_swing_steps_*dS_ before
    // liftoff (during PRE_SWING body walk). The foothold world position is unchanged,
    // so the offset from current hip is reduced by the PRE_SWING travel distance.
    const double pre_swing_dist  = static_cast<double>(pre_swing_steps_) * dS_;
    const double foothold_offset = effective_step_length * (1.0 + SWING_TIME) / 2.0 - pre_swing_dist;
    double td_x = hip_[leg][0] + foothold_offset;
    double td_z = 0.0;

    // Contact rim search — contact_p_local is foot position relative to the hip AT TOUCHDOWN.
    // hip_td = hip_lo + swing_time * effective_step_length
    // → foot is ahead of hip_td by stance_half = (1-swing_time)/2 * eff - pre_swing_dist
    //   (pre_swing body travel is accounted for: stance feet are effectively closer to hip)
    const double stance_half = effective_step_length * (1.0 - SWING_TIME) / 2.0 - pre_swing_dist;
    int found_rim = 3;  // default G
    for (int j = 0; j < 5; ++j) {
        double contact_height = (j == 0) ? leg_model_->r : leg_model_->radius;
        std::array<double, 2> contact_p_local = {stance_half, -stand_height_ + contact_height};
        auto eta = leg_model_->inverse(contact_p_local, TOUCH_RIM_LIST[j]);
        leg_model_->contact_map(eta[0], eta[1]);
        if (leg_model_->rim == TOUCH_RIM_IDX[j]) {
            found_rim = leg_model_->rim;
            leg_model_->forward(eta[0], eta[1]);
            // Compute world-frame G position at touchdown
            if (found_rim == 3) {
                td_x = hip_[leg][0] + foothold_offset;
                td_z = leg_model_->r;
            } else if (found_rim == 2) {
                td_x = hip_[leg][0] + foothold_offset + leg_model_->G[0] - leg_model_->L_l[0];
                td_z = leg_model_->G[1] - leg_model_->L_l[1] + leg_model_->radius;
            } else if (found_rim == 4) {
                td_x = hip_[leg][0] + foothold_offset + leg_model_->G[0] - leg_model_->L_r[0];
                td_z = leg_model_->G[1] - leg_model_->L_r[1] + leg_model_->radius;
            } else if (found_rim == 1) {
                td_x = hip_[leg][0] + foothold_offset + leg_model_->G[0] - leg_model_->U_l[0];
                td_z = leg_model_->G[1] - leg_model_->U_l[1] + leg_model_->radius;
            } else if (found_rim == 5) {
                td_x = hip_[leg][0] + foothold_offset + leg_model_->G[0] - leg_model_->U_r[0];
                td_z = leg_model_->G[1] - leg_model_->U_r[1] + leg_model_->radius;
            }
            break;
        }
    }

    p_td_ = {td_x, td_z};
    foothold_[leg] = {td_x, 0.0};

    swing_profile_ = SwingProfile(p_lo_, p_td_, step_height_, 1);

    RCLCPP_INFO(this->get_logger(),
        "Start swing leg %d: p_lo=(%.3f,%.3f) p_td=(%.3f,%.3f) swing_steps=%d foothold_offset=%.3f",
        leg, p_lo_[0], p_lo_[1], p_td_[0], p_td_[1], swing_steps_, foothold_offset);
}

void EventWalkNode::do_swing()
{
    // ── advance all hips forward each tick (continuous body locomotion) ───────
    // Mirrors walk_gait.cpp: next_hip[i][0] += dS every tick.
    for (int i = 0; i < 4; ++i) hip_[i][0] += dS_;

    double progress = static_cast<double>(swing_tick_) / static_cast<double>(swing_steps_);
    progress = std::min(progress, 1.0);

    // ── swing leg: follow Bezier trajectory ──────────────────────────────────
    auto foot_world = swing_profile_.getFootendPoint(progress);

    // Convert world foot position to hip-frame for inverse kinematics
    std::array<double, 2> foot_hip = {
        foot_world[0] - hip_[swing_leg_][0],
        foot_world[1] - hip_[swing_leg_][1]};

    auto eta_swing = leg_model_->inverse(foot_hip, "G");
    theta_[swing_leg_] = eta_swing[0];
    beta_[swing_leg_]  = eta_swing[1];

    // ── stance legs: incremental IK as hip slides forward over fixed foot ─────
    // Mirrors walk_gait.cpp: leg_model.move(theta, beta, {next_hip - hip}).
    for (int i = 0; i < 4; ++i) {
        if (i == swing_leg_) continue;
        auto eta_st = leg_model_->move(theta_[i], beta_[i], {dS_, 0.0});
        theta_[i] = eta_st[0];
        beta_[i]  = eta_st[1];
    }

    publish_phase(0);
    publish_swing_mask();
    publish_cmd();

    swing_tick_++;

    // ── touchdown detection ───────────────────────────────────────────────────
    bool duty_done   = (swing_tick_ >= swing_steps_);
    bool contact_hit = contact_[swing_leg_];  // sensor fires

    if (duty_done || contact_hit) {
        // Record actual step length for this leg: body travelled swing_tick_ * dS_ during
        // the swing phase, which represents SWING_TIME fraction of the full step cycle.
        // Actual step length = travel / SWING_TIME.  Hind legs will inherit this value.
        leg_step_length_[swing_leg_] = static_cast<double>(swing_tick_) * dS_ / SWING_TIME;

        if (contact_hit) {
            RCLCPP_INFO(this->get_logger(),
                "Leg %d touchdown via CONTACT sensor (tick=%d, eff_step=%.4f)",
                swing_leg_, swing_tick_, leg_step_length_[swing_leg_]);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Leg %d touchdown via DUTY completion (eff_step=%.4f)",
                swing_leg_, leg_step_length_[swing_leg_]);
        }

        // Snap swing leg to final stance IK at the planned touchdown position.
        // hip_[swing_leg_] already reflects the accumulated dS_ advances.
        auto eta_td = leg_model_->inverse(
            {p_td_[0] - hip_[swing_leg_][0], p_td_[1] - hip_[swing_leg_][1]}, "G");
        theta_[swing_leg_] = eta_td[0];
        beta_[swing_leg_]  = eta_td[1];

        swing_mask_.fill(0);
        enter_adjusting();
    }
}

// ── ADJUSTING phase ───────────────────────────────────────────────────────────

void EventWalkNode::enter_adjusting()
{
    adjust_tick_ = 0;
    attitude_stable_ = false;
    phase_ = Phase::ADJUSTING;

    RCLCPP_INFO(this->get_logger(),
        "Leg %d landed — entering ADJUSTING phase", swing_leg_);
}

// ── command publishing ────────────────────────────────────────────────────────

void EventWalkNode::publish_cmd()
{
    cmd_msg_.header.stamp = this->now();
    for (int i = 0; i < 4; ++i) {
        cmd_mods_[i]->theta = theta_[i];
        // Apply beta sign convention (same as walk_exp.cpp):
        // legs 0,3 (FL,RL) use negative beta; legs 1,2 (FR,RR) use positive beta
        if (i == 0 || i == 3) {
            cmd_mods_[i]->beta = -beta_[i];
        } else {
            cmd_mods_[i]->beta =  beta_[i];
        }
    }
    pub_cmd_->publish(cmd_msg_);
}

void EventWalkNode::publish_phase(int phase_val)
{
    std_msgs::msg::Int32 msg;
    msg.data = phase_val;
    pub_phase_->publish(msg);
}

void EventWalkNode::publish_swing_mask()
{
    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
        msg.data[i] = (phase_ == Phase::SWING || phase_ == Phase::ADJUSTING)
                      ? swing_mask_[i]
                      : -1;
    }
    pub_swing_mask_->publish(msg);
    pub_swing_phase_->publish(msg);  // legacy compat topic
}
