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
                             double con_bias,
                             int    sampling_rate,
                             int    max_adjust_steps,
                             double BL,
                             double BW,
                             double probe_speed,
                             int    max_probe_steps,
                             double contact_swing_accept_ratio,
                             int    contact_on_count,
                             int    contact_off_count,
                             double pre_swing_advance_scale)
: sim_(sim)
, velocity_(velocity)
, stand_height_(stand_height)
, step_length_(step_length)
, step_height_(step_height)
, con_bias_(con_bias)
, sampling_rate_(sampling_rate)
, max_adjust_steps_(max_adjust_steps)
, BL_(BL)
, BW_(BW)
, probe_speed_(probe_speed)
, max_probe_steps_(max_probe_steps)
, contact_swing_accept_ratio_(contact_swing_accept_ratio)
, contact_on_count_threshold_(contact_on_count)
, contact_off_count_threshold_(contact_off_count)
, pre_swing_advance_scale_(pre_swing_advance_scale)
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

    contact_swing_accept_ratio_ =
        std::max(0.0, std::min(1.0, contact_swing_accept_ratio_));
    contact_on_count_threshold_  = std::max(1, contact_on_count_threshold_);
    contact_off_count_threshold_ = std::max(1, contact_off_count_threshold_);
    max_probe_steps_             = std::max(0, max_probe_steps_);
    probe_speed_                 = std::max(0.0, probe_speed_);
    pre_swing_advance_scale_     = std::max(0.0, pre_swing_advance_scale_);
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

    case Phase::PROBING:
        do_probing(input, out);
        break;

    case Phase::ADJUSTING:
        do_adjusting(input, out);
        break;

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
    const double remaining = pre_swing_target_dist_ - pre_swing_advanced_dist_;
    if (remaining > 1e-6) {
        const double request_dx = std::min(dS_ + stance_move_carry_, remaining);
        const double actual_dx = advance_stance_body(
            input, pre_swing_tick_ == 0 && needs_motor_sync_, request_dx);
        pre_swing_advanced_dist_ += actual_dx;
        stance_move_carry_ = std::max(0.0, request_dx - actual_dx);
    }

    fill_output(out);
    out.phase_int = 0;
    out.in_walk   = false;
    pre_swing_tick_++;

    const int max_pre_swing_ticks = std::max(pre_swing_steps_ * 3, pre_swing_steps_ + sampling_rate_ / 2);
    if (pre_swing_advanced_dist_ + 1e-6 >= pre_swing_target_dist_ ||
        pre_swing_tick_ >= max_pre_swing_ticks) {
        if (pre_swing_tick_ >= max_pre_swing_ticks &&
            pre_swing_advanced_dist_ + 1e-6 < pre_swing_target_dist_) {
            std::printf("[EventWalkGait] WARN: PRE_SWING advance incomplete target=%.4f actual=%.4f\n",
                pre_swing_target_dist_, pre_swing_advanced_dist_);
        }
        start_swing(swing_leg_, input);  // needs_motor_sync_==false → skip re-sync
        phase_ = Phase::SWING;
    }
}

void EventWalkGait::start_swing(int leg, const ExternalInput & input)
{
    swing_leg_  = leg;
    swing_tick_ = 0;
    late_probing_ = false;
    probe_tick_ = 0;
    support_advance_ticks_ = 0;
    stance_move_carry_ = 0.0;
    swing_mask_.fill(0);
    swing_mask_[leg] = 1;
    attitude_stable_latch_ = false;  // reset latch for new adjustment cycle
    filtered_contact_[leg] = false;
    contact_on_count_[leg] = 0;
    contact_off_count_[leg] = 0;

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
    double td_x = hip_[leg][0] + foot_offset + con_bias_;
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
            if      (found_rim == 3) { td_x = hip_[leg][0] + foot_offset + con_bias_; td_z = leg_model_->r; }
            else if (found_rim == 2) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->L_l[0] + con_bias_; td_z = leg_model_->G[1] - leg_model_->L_l[1] + leg_model_->radius; }
            else if (found_rim == 4) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->L_r[0] + con_bias_; td_z = leg_model_->G[1] - leg_model_->L_r[1] + leg_model_->radius; }
            else if (found_rim == 1) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->U_l[0] + con_bias_; td_z = leg_model_->G[1] - leg_model_->U_l[1] + leg_model_->radius; }
            else if (found_rim == 5) { td_x = hip_[leg][0] + foot_offset + leg_model_->G[0] - leg_model_->U_r[0] + con_bias_; td_z = leg_model_->G[1] - leg_model_->U_r[1] + leg_model_->radius; }
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
    const double actual_dx = advance_stance_legs(input, false, dS_, swing_leg_);
    hip_[swing_leg_][0] += actual_dx;

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

    fill_output(out);
    out.phase_int = 0;
    out.in_walk   = true;
    swing_tick_++;

    bool duty_done   = (swing_tick_ >= swing_steps_);
    bool contact_hit = update_contact_filter(swing_leg_, input, progress);

    if (contact_hit) {
        finish_touchdown(true, "CONTACT");
        fill_output(out);
        out.phase_int = 2;
        out.in_walk   = true;
    } else if (duty_done) {
        if (input.contact_enabled && max_probe_steps_ > 0 && probe_speed_ > 0.0) {
            enter_probing();
            fill_output(out);
            out.phase_int = 0;
            out.in_walk   = true;
        } else {
            finish_touchdown(false, "DUTY");
            fill_output(out);
            out.phase_int = 2;
            out.in_walk   = true;
        }
    }
}

void EventWalkGait::do_probing(const ExternalInput & input, GaitOutput & out)
{
    const bool contact_hit = update_contact_filter(swing_leg_, input, 1.0);

    if (contact_hit || probe_tick_ >= max_probe_steps_) {
        finish_touchdown(contact_hit, contact_hit ? "LATE_CONTACT" : "PROBE_TIMEOUT");
        fill_output(out);
        out.phase_int = 2;
        out.in_walk   = true;
        return;
    }

    probe_point_[1] -= probe_speed_ / static_cast<double>(sampling_rate_);
    std::array<double,2> foot_hip = {
        probe_point_[0] - hip_[swing_leg_][0],
        probe_point_[1] - hip_[swing_leg_][1]};
    auto eta = leg_model_->inverse(foot_hip, "G");
    theta_[swing_leg_] = eta[0];
    beta_[swing_leg_]  = eta[1];
    probe_tick_++;

    fill_output(out);
    out.phase_int = 0;
    out.in_walk   = true;
}

void EventWalkGait::do_adjusting(const ExternalInput & input, GaitOutput & out)
{
    if (input.attitude_stable) attitude_stable_latch_ = true;

    adjust_tick_++;
    fill_output(out);
    out.phase_int = 2;
    out.in_walk   = true;

    const bool attitude_done =
        attitude_stable_latch_ || adjust_tick_ >= max_adjust_steps_;

    if (attitude_done) {
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
                return;
            }
        }

        needs_motor_sync_ = true;
        if (swing_leg_ == 2 || swing_leg_ == 3) {
            const double eff = leg_step_length_[(swing_leg_ + 2) % 4];
            const int base_pre_swing_steps = static_cast<int>(
                (1.0 - 4.0 * SWING_TIME) / 2.0 * eff / dS_);
            pre_swing_steps_ = static_cast<int>(
                pre_swing_advance_scale_ * static_cast<double>(base_pre_swing_steps));
            pre_swing_target_dist_ = static_cast<double>(pre_swing_steps_) * dS_;
            pre_swing_advanced_dist_ = 0.0;
            stance_move_carry_ = 0.0;
            pre_swing_tick_  = 0;
            phase_ = Phase::PRE_SWING;
        } else {
            pre_swing_steps_ = 0;
            pre_swing_target_dist_ = 0.0;
            pre_swing_advanced_dist_ = 0.0;
            stance_move_carry_ = 0.0;
            start_swing(swing_leg_, input);
            phase_ = Phase::SWING;
        }
    }
}

double EventWalkGait::advance_stance_body(const ExternalInput & input, bool sync_from_feedback, double dx)
{
    return advance_stance_legs(input, sync_from_feedback, dx, -1);
}

double EventWalkGait::advance_stance_legs(const ExternalInput & input, bool sync_from_feedback,
                                          double dx, int skip_leg)
{
    if (dx <= 0.0) return 0.0;

    if (sync_from_feedback && input.motor_state_valid) {
        sync_from_motor(input);
        needs_motor_sync_ = false;
    }

    const auto theta_start = theta_;
    const auto beta_start = beta_;
    std::array<double, 4> cand_theta = theta_;
    std::array<double, 4> cand_beta = beta_;

    double trial_dx = dx;
    while (trial_dx >= stance_move_min_dx_) {
        bool ok = true;
        int bad_leg = -1;
        double bad_res_x = 0.0;
        double bad_res_y = 0.0;
        double bad_cost = 0.0;

        cand_theta = theta_start;
        cand_beta = beta_start;

        for (int i = 0; i < 4; ++i) {
            if (i == skip_leg) continue;

            auto eta = leg_model_->move(theta_start[i], beta_start[i], {trial_dx, 0.0});
            const double res_x = leg_model_->last_move_residual[0];
            const double res_y = leg_model_->last_move_residual[1];
            const double cost = leg_model_->last_move_cost;

            if (!leg_model_->last_move_converged ||
                cost > stance_move_cost_tol_ ||
                std::abs(res_y) > stance_move_vertical_tol_) {
                ok = false;
                bad_leg = i;
                bad_res_x = res_x;
                bad_res_y = res_y;
                bad_cost = cost;
                break;
            }

            cand_theta[i] = eta[0];
            cand_beta[i] = eta[1];
        }

        if (ok) {
            theta_ = cand_theta;
            beta_ = cand_beta;
            for (int i = 0; i < 4; ++i) {
                if (i == skip_leg) continue;
                hip_[i][0] += trial_dx;
            }
            return trial_dx;
        }

        if (trial_dx <= stance_move_min_dx_ * 2.0) {
            std::printf(
                "[EventWalkGait] WARN: reject stance move leg %d residual=(%.5f, %.5f) cost=%.5f requested=%.5f\n",
                bad_leg, bad_res_x, bad_res_y, bad_cost, trial_dx);
        }
        trial_dx *= 0.5;
    }

    return 0.0;
}

bool EventWalkGait::update_contact_filter(int leg, const ExternalInput & input, double swing_progress)
{
    if (!input.contact_enabled) return false;

    if (phase_ == Phase::SWING && swing_progress < contact_swing_accept_ratio_) {
        filtered_contact_[leg] = false;
        contact_on_count_[leg] = 0;
        contact_off_count_[leg] = 0;
        return false;
    }

    if (input.contact[leg]) {
        contact_on_count_[leg]++;
        contact_off_count_[leg] = 0;
    } else {
        contact_on_count_[leg] = 0;
        contact_off_count_[leg]++;
    }

    if (filtered_contact_[leg]) {
        if (contact_off_count_[leg] >= contact_off_count_threshold_) {
            filtered_contact_[leg] = false;
        }
    } else if (contact_on_count_[leg] >= contact_on_count_threshold_) {
        filtered_contact_[leg] = true;
    }

    return filtered_contact_[leg];
}

void EventWalkGait::enter_probing()
{
    late_probing_ = true;
    probe_tick_ = 0;
    probe_point_ = p_td_;
    std::printf("[EventWalkGait] Leg %d planned touchdown missed — probing downward\n",
        swing_leg_);
    phase_ = Phase::PROBING;
}

void EventWalkGait::finish_touchdown(bool contact_hit, const char * reason)
{
    leg_step_length_[swing_leg_] =
        static_cast<double>(std::max(1, swing_tick_)) * dS_ / SWING_TIME;

    if (contact_hit || late_probing_) {
        leg_model_->forward(theta_[swing_leg_], beta_[swing_leg_]);
        p_td_ = {hip_[swing_leg_][0] + leg_model_->G[0],
                 hip_[swing_leg_][1] + leg_model_->G[1]};
    } else {
        auto eta_td = leg_model_->inverse(
            {p_td_[0] - hip_[swing_leg_][0], p_td_[1] - hip_[swing_leg_][1]}, "G");
        theta_[swing_leg_] = eta_td[0];
        beta_[swing_leg_]  = eta_td[1];
    }

    foothold_[swing_leg_] = {p_td_[0], 0.0};
    swing_mask_.fill(0);
    late_probing_ = false;

    std::printf("[EventWalkGait] Leg %d touchdown via %s (tick=%d, eff=%.4f)\n",
        swing_leg_, reason, swing_tick_, leg_step_length_[swing_leg_]);
    enter_adjusting();
}

void EventWalkGait::enter_adjusting()
{
    adjust_tick_ = 0;
    support_advance_ticks_ = 0;
    support_advance_target_steps_ = 0;
    phase_ = Phase::ADJUSTING;
    std::printf("[EventWalkGait] Leg %d landed — entering ADJUSTING phase\n", swing_leg_);
}

void EventWalkGait::fill_output(GaitOutput & out) const
{
    out.theta      = theta_;
    out.beta       = beta_;
    out.swing_mask = swing_mask_;
}
