/**
 * event_walk_gait.cpp
 *
 * State machine implementation for EventWalkGait.
 * Extracted from event_walk_node.cpp so it can be used without ROS.
 */

#include "corgi_event_walk/event_walk_gait.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

// ── Touch-rim search order (same as walk_gait.cpp / event_walk_node.cpp) ─────
static const std::string TOUCH_RIM_LIST[5] = {"G", "L_l", "L_r", "U_l", "U_r"};
static const int         TOUCH_RIM_IDX[5]  = {  3,     2,     4,     1,     5};

// out-of-line storage for constexpr arrays (required pre-C++17 inline vars)
constexpr int    EventWalkGait::LEG_SEQ[4];
constexpr double EventWalkGait::INIT_ETA[8];

// ── constructor ───────────────────────────────────────────────────────────────

EventWalkGait::EventWalkGait(bool   sim,
                             double velocity,
                             double stand_height,
                             double step_length,
                             double step_height,
                             int    sampling_rate,
                             int    max_adjust_steps,
                             double BL,
                             double BW)
: sim_(sim)
, velocity_(velocity)
, stand_height_(stand_height)
, step_length_(step_length)
, step_height_(step_height)
, sampling_rate_(sampling_rate)
, max_adjust_steps_(max_adjust_steps)
, BL_(BL)
, BW_(BW)
{
    leg_model_ = std::make_unique<LegModel>(sim_);

    dS_          = velocity_ / static_cast<double>(sampling_rate_);
    swing_steps_ = static_cast<int>(SWING_TIME * step_length_ / dS_);

    transform_count_ = 3 * sampling_rate_;   // 3 s

    hip_[0] = { BL_/2.0,  stand_height_ };
    hip_[1] = { BL_/2.0,  stand_height_ };
    hip_[2] = {-BL_/2.0,  stand_height_ };
    hip_[3] = {-BL_/2.0,  stand_height_ };

    for (int i = 0; i < 4; ++i) foothold_[i] = {hip_[i][0], 0.0};
    leg_step_length_.fill(step_length_);

    theta_.fill(INIT_THETA);
    beta_.fill(INIT_BETA);
    swing_mask_.fill(0);
    swing_leg_ = LEG_SEQ[0];
}

// ── public API ────────────────────────────────────────────────────────────────

void EventWalkGait::set_velocity(double v)
{
    velocity_    = v;
    dS_          = velocity_ / static_cast<double>(sampling_rate_);
    swing_steps_ = static_cast<int>(SWING_TIME * step_length_ / dS_);
}

void EventWalkGait::set_stand_height(double h)
{
    stand_height_ = h;
    for (int i = 0; i < 4; ++i) hip_[i][1] = stand_height_;
}

std::array<int,4> EventWalkGait::get_swing_phase() const { return swing_mask_; }
bool EventWalkGait::is_adjusting() const { return phase_ == Phase::ADJUSTING; }
bool EventWalkGait::is_ready()     const { return phase_ == Phase::READY; }
bool EventWalkGait::is_ended()     const { return phase_ == Phase::END;   }

// ── main step ─────────────────────────────────────────────────────────────────

EventWalkGait::GaitOutput EventWalkGait::step(const ExternalInput & input)
{
    GaitOutput out;

    switch (phase_)
    {
    case Phase::INIT:
        transform_tick_ = 0;
        phase_ = Phase::TRANSFORM;
        fill_output(out);
        out.phase_int = 0;
        out.in_walk   = false;
        break;

    case Phase::TRANSFORM:
        do_transform(input, out);
        break;

    case Phase::READY:
        fill_output(out);
        out.phase_int = 0;
        out.in_walk   = false;
        if (input.trigger) {
            std::printf("[EventWalkGait] Trigger received — starting walk.\n");
            seq_idx_   = 0;
            swing_leg_ = LEG_SEQ[seq_idx_];
            start_swing(swing_leg_, input);
            phase_ = Phase::SWING;
        }
        break;

    case Phase::PRE_SWING:
        do_pre_swing(input, out);
        break;

    case Phase::SWING:
        do_swing(input, out);
        break;

    case Phase::ADJUSTING:
    {
        // Accumulate attitude_stable signal via latch (reset in start_swing)
        if (input.attitude_stable) attitude_stable_latch_ = true;

        adjust_tick_++;
        fill_output(out);
        out.phase_int = 2;
        out.in_walk   = true;

        if (attitude_stable_latch_ || adjust_tick_ >= max_adjust_steps_) {
            if (adjust_tick_ >= max_adjust_steps_ && !attitude_stable_latch_) {
                std::printf("[EventWalkGait] WARN: Attitude adjustment timed out for leg %d\n",
                    swing_leg_);
            }

            seq_idx_   = (seq_idx_ + 1) % 4;
            swing_leg_ = LEG_SEQ[seq_idx_];

            if (seq_idx_ == 0) {
                std::printf("[EventWalkGait] Completed one gait cycle.\n");
                if (!input.trigger) {
                    std::printf("[EventWalkGait] Trigger released — stopping.\n");
                    phase_ = Phase::END;
                    out.phase_int = 0;
                    out.in_walk   = false;
                    return out;
                }
            }

            // Hind legs (RR=2, RL=3) need PRE_SWING body advance
            if (swing_leg_ == 2 || swing_leg_ == 3) {
                const double eff = leg_step_length_[(swing_leg_ + 2) % 4];
                pre_swing_steps_ = static_cast<int>(
                    (1.0 - 4.0 * SWING_TIME) / 2.0 * eff / dS_);
                pre_swing_tick_  = 0;
                needs_motor_sync_ = true;
                phase_ = Phase::PRE_SWING;
            } else {
                needs_motor_sync_ = true;
                pre_swing_steps_  = 0;
                start_swing(swing_leg_, input);
                phase_ = Phase::SWING;
            }
        }
        break;
    }

    case Phase::END:
        fill_output(out);
        out.phase_int = 0;
        out.in_walk   = false;
        break;
    }

    return out;
}

// ── private phase implementations ─────────────────────────────────────────────

void EventWalkGait::do_transform(const ExternalInput & /*input*/, GaitOutput & out)
{
    double ratio = static_cast<double>(transform_tick_) /
                   static_cast<double>(transform_count_);
    ratio = std::min(ratio, 1.0);

    for (int i = 0; i < 4; ++i) {
        theta_[i] = INIT_THETA + ratio * (INIT_ETA[i*2]   - INIT_THETA);
        double raw_beta = INIT_BETA + ratio * (INIT_ETA[i*2+1] - INIT_BETA);
        beta_[i] = (i == 1 || i == 2) ? raw_beta : -raw_beta;  // internal convention
    }

    transform_tick_++;
    fill_output(out);
    out.phase_int = 0;
    out.in_walk   = false;

    if (transform_tick_ >= transform_count_) {
        for (int i = 0; i < 4; ++i) {
            theta_[i] = INIT_ETA[i*2];
            beta_[i]  = (i == 1 || i == 2) ? INIT_ETA[i*2+1] : -INIT_ETA[i*2+1];
        }
        std::printf("[EventWalkGait] TRANSFORM done -> READY (waiting for trigger)\n");
        phase_ = Phase::READY;
    }
}

void EventWalkGait::sync_from_motor(const ExternalInput & input)
{
    for (int i = 0; i < 4; ++i) {
        theta_[i] = input.motor_theta[i];
        // Undo motor convention (legs 0,3 are negated on the motor bus)
        beta_[i] = (i == 0 || i == 3) ? -input.motor_beta[i] : input.motor_beta[i];
    }
}

void EventWalkGait::do_pre_swing(const ExternalInput & input, GaitOutput & out)
{
    // On first tick: sync from actual motor state so move() starts from
    // attitude-adjusted angles.
    if (pre_swing_tick_ == 0 && needs_motor_sync_ && input.motor_state_valid) {
        sync_from_motor(input);
        needs_motor_sync_ = false;
    }

    // Advance all hips and update stance IK
    for (int i = 0; i < 4; ++i) hip_[i][0] += dS_;
    for (int i = 0; i < 4; ++i) {
        auto eta = leg_model_->move(theta_[i], beta_[i], {dS_, 0.0});
        theta_[i] = eta[0];
        beta_[i]  = eta[1];
    }
    fill_output(out);
    out.phase_int = 0;
    out.in_walk   = false;
    pre_swing_tick_++;

    if (pre_swing_tick_ >= pre_swing_steps_) {
        start_swing(swing_leg_, input);  // needs_motor_sync_==false → skip re-sync
        phase_ = Phase::SWING;
    }
}

void EventWalkGait::start_swing(int leg, const ExternalInput & input)
{
    swing_leg_  = leg;
    swing_tick_ = 0;
    swing_mask_.fill(0);
    swing_mask_[leg] = 1;
    attitude_stable_latch_ = false;  // reset latch for new adjustment cycle

    if (needs_motor_sync_ && input.motor_state_valid) {
        sync_from_motor(input);
        needs_motor_sync_ = false;
    }

    // Liftoff point from FK
    leg_model_->forward(theta_[leg], beta_[leg]);
    p_lo_ = {hip_[leg][0] + leg_model_->G[0],
              hip_[leg][1] + leg_model_->G[1]};

    // Effective step length (hind legs inherit contralateral front)
    const double eff =
        (leg == 2 || leg == 3) ? leg_step_length_[(leg + 2) % 4] : step_length_;
    swing_steps_ = static_cast<int>(SWING_TIME * eff / dS_);

    const double pre_dist      = static_cast<double>(pre_swing_steps_) * dS_;
    const double foot_offset   = eff * (1.0 + SWING_TIME) / 2.0 - pre_dist;
    double td_x = hip_[leg][0] + foot_offset;
    double td_z = 0.0;

    // Contact-rim search
    const double stance_half = eff * (1.0 - SWING_TIME) / 2.0 - pre_dist;
    for (int j = 0; j < 5; ++j) {
        double contact_height = (j == 0) ? leg_model_->r : leg_model_->radius;
        std::array<double,2> cp_local = {stance_half, -stand_height_ + contact_height};
        auto eta = leg_model_->inverse(cp_local, TOUCH_RIM_LIST[j]);
        leg_model_->contact_map(eta[0], eta[1]);
        if (leg_model_->rim == TOUCH_RIM_IDX[j]) {
            int found_rim = leg_model_->rim;
            leg_model_->forward(eta[0], eta[1]);
            if      (found_rim == 3) { td_x = hip_[leg][0] + foot_offset; td_z = leg_model_->r; }
            else if (found_rim == 2) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->L_l[0]; td_z = leg_model_->G[1] - leg_model_->L_l[1] + leg_model_->radius; }
            else if (found_rim == 4) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->L_r[0]; td_z = leg_model_->G[1] - leg_model_->L_r[1] + leg_model_->radius; }
            else if (found_rim == 1) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->U_l[0]; td_z = leg_model_->G[1] - leg_model_->U_l[1] + leg_model_->radius; }
            else if (found_rim == 5) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->U_r[0]; td_z = leg_model_->G[1] - leg_model_->U_r[1] + leg_model_->radius; }
            break;
        }
    }

    p_td_ = {td_x, td_z};
    foothold_[leg] = {td_x, 0.0};
    swing_profile_ = SwingProfile(p_lo_, p_td_, step_height_, 1);

    std::printf(
        "[EventWalkGait] Start swing leg %d: p_lo=(%.3f,%.3f) p_td=(%.3f,%.3f) swing_steps=%d\n",
        leg, p_lo_[0], p_lo_[1], p_td_[0], p_td_[1], swing_steps_);
}

void EventWalkGait::do_swing(const ExternalInput & input, GaitOutput & out)
{
    for (int i = 0; i < 4; ++i) hip_[i][0] += dS_;

    double progress = static_cast<double>(swing_tick_) /
                      static_cast<double>(swing_steps_);
    progress = std::min(progress, 1.0);

    // Swing leg: follow Bezier trajectory
    auto foot_world = swing_profile_.getFootendPoint(progress);
    std::array<double,2> foot_hip = {
        foot_world[0] - hip_[swing_leg_][0],
        foot_world[1] - hip_[swing_leg_][1]};
    auto eta_sw = leg_model_->inverse(foot_hip, "G");
    theta_[swing_leg_] = eta_sw[0];
    beta_[swing_leg_]  = eta_sw[1];

    // Stance legs: incremental IK as hip slides over fixed foot
    for (int i = 0; i < 4; ++i) {
        if (i == swing_leg_) continue;
        auto eta_st = leg_model_->move(theta_[i], beta_[i], {dS_, 0.0});
        theta_[i] = eta_st[0];
        beta_[i]  = eta_st[1];
    }

    fill_output(out);
    out.phase_int = 0;
    out.in_walk   = true;
    swing_tick_++;

    bool duty_done   = (swing_tick_ >= swing_steps_);
    bool contact_hit = input.contact[swing_leg_];

    if (duty_done || contact_hit) {
        leg_step_length_[swing_leg_] =
            static_cast<double>(swing_tick_) * dS_ / SWING_TIME;

        if (contact_hit) {
            std::printf("[EventWalkGait] Leg %d touchdown via CONTACT (tick=%d, eff=%.4f)\n",
                swing_leg_, swing_tick_, leg_step_length_[swing_leg_]);
        } else {
            std::printf("[EventWalkGait] Leg %d touchdown via DUTY (eff=%.4f)\n",
                swing_leg_, leg_step_length_[swing_leg_]);
        }

        // Snap swing leg to planned touchdown IK
        auto eta_td = leg_model_->inverse(
            {p_td_[0] - hip_[swing_leg_][0], p_td_[1] - hip_[swing_leg_][1]}, "G");
        theta_[swing_leg_] = eta_td[0];
        beta_[swing_leg_]  = eta_td[1];

        swing_mask_.fill(0);
        enter_adjusting();
        // Update output so this tick already reflects ADJUSTING
        fill_output(out);
        out.phase_int = 2;
        out.in_walk   = true;
    }
}

void EventWalkGait::enter_adjusting()
{
    adjust_tick_ = 0;
    phase_ = Phase::ADJUSTING;
    std::printf("[EventWalkGait] Leg %d landed — entering ADJUSTING phase\n", swing_leg_);
}

void EventWalkGait::fill_output(GaitOutput & out) const
{
    out.theta      = theta_;
    out.beta       = beta_;
    out.swing_mask = swing_mask_;
}
