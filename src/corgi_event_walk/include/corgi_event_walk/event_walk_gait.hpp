#ifndef EVENT_WALK_GAIT_HPP
#define EVENT_WALK_GAIT_HPP

/**
 * event_walk_gait.hpp
 *
 * ROS-free event-driven sequential walk gait engine.
 *
 * Design mirrors WalkGait (corgi_walk): instantiate, call step() each control
 * tick, act on the returned GaitOutput.  No ROS headers are included.
 *
 * State machine:
 *   INIT → TRANSFORM → READY → SWING → ADJUSTING → (next leg or END)
 *   Hind legs (RR=2, RL=3) have a brief PRE_SWING body-advance phase first.
 *
 * Leg numbering:
 *   0 = FL (+BL/2, +BW/2)
 *   1 = FR (+BL/2, -BW/2)
 *   2 = RR (-BL/2, -BW/2)
 *   3 = RL (-BL/2, +BW/2)
 * Walk sequence: FL(0) → RR(2) → FR(1) → RL(3)
 */

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

class EventWalkGait
{
public:
    enum class LegState : int {
        INACTIVE  = -1,
        STANCE    = 0,
        SWING     = 1,
        PROBING   = 2,
        ADJUSTING = 3,
        PRE_SWING = 4
    };

    // ── I/O types ─────────────────────────────────────────────────────────────

    struct ExternalInput {
        bool trigger{false};
        bool attitude_stable{false};
        std::array<bool, 4>   contact{false, false, false, false};
        bool contact_enabled{false};
        /// Motor angles in the motor-output convention:
        ///   legs 0 & 3 carry beta with opposite sign on the motor bus.
        std::array<double, 4> motor_theta{};
        std::array<double, 4> motor_beta{};   // motor-convention beta
        bool motor_state_valid{false};
    };

    struct GaitOutput {
        std::array<double, 4> theta{};
        /// Beta in EventWalkGait internal convention (same as WalkGait):
        ///   legs 0 & 3 are stored negated; legs 1 & 2 are stored positive.
        /// To publish as impedance command (walk_h20 convention):
        ///   imp_cmd.beta[i] = (i==1||i==2) ? out.beta[i] : -out.beta[i]
        std::array<double, 4> beta{};
        std::array<int, 4>    swing_mask{0, 0, 0, 0};  // 1=swinging, 0=stance
        /// Per-leg state, ordered FL, FR, RR, RL. Values follow LegState.
        std::array<int, 4>    leg_state{-1, -1, -1, -1};
        int  phase_int{0};    // 0 = normal/passthrough, 2 = ADJUSTING
        bool in_walk{false};  // true iff phase is SWING or ADJUSTING
    };

    // ── constructor ───────────────────────────────────────────────────────────

    /**
     * @param sim              Passed to LegModel
     * @param velocity         Initial forward velocity (m/s)
     * @param stand_height     Hip height (m)  — must match INIT_ETA
     * @param step_length      Nominal step length (m)
     * @param step_height      Swing clearance (m)
     * @param sampling_rate    Control loop frequency (Hz)
     * @param max_adjust_steps Max ticks in ADJUSTING before forcing next leg
     * @param BL               Body length (m)
     * @param BW               Body width  (m)
     */
    EventWalkGait(bool   sim,
                  double velocity,
                  double stand_height,
                  double step_length,
                  double step_height,
                  double con_bias,
                  int    sampling_rate,
                  int    max_adjust_steps = 3000,
                  double BL = 0.444,
                  double BW = 0.4,
                  double probe_speed = 0.15,
                  int    max_probe_steps = 500,
                  double contact_swing_accept_ratio = 0.5,
                  int    contact_on_count = 5,
                  int    contact_off_count = 2,
                  double pre_swing_advance_scale = 1.5);

    // ── main API ──────────────────────────────────────────────────────────────

    GaitOutput step(const ExternalInput & input);

    void set_velocity(double v);
    void set_stand_height(double h);

    std::array<int, 4> get_swing_phase() const;
    int completed_cycles() const;
    bool is_adjusting() const;
    bool is_ready()     const;
    bool is_ended()     const;

private:
    // ── state machine ─────────────────────────────────────────────────────────
    enum class Phase { INIT, TRANSFORM, READY, PRE_SWING, SWING, PROBING, ADJUSTING, END };
    Phase phase_{Phase::INIT};

    static constexpr int    LEG_SEQ[4]  = {0, 2, 1, 3};
    static constexpr double SWING_TIME  = 0.2;
    static constexpr double INIT_THETA  = M_PI * 17.0 / 180.0;
    static constexpr double INIT_BETA   = 0.0;

    // Stand configuration: stand_height=0.20
    static constexpr double INIT_ETA[8] = {
        1.3313651941315507,  0.4032814817188362,
        1.1847611807810603,  0.10626486289107877,
        1.1847611807810603, -0.10626486289107877,
        1.3313651941315507, -0.4032814817188362
    };

    // ── parameters ────────────────────────────────────────────────────────────
    bool   sim_;
    double velocity_;
    double stand_height_;
    double step_length_;
    double step_height_;
    double con_bias_;
    int    sampling_rate_;
    int    max_adjust_steps_;
    double BL_, BW_;
    double probe_speed_;
    int    max_probe_steps_;
    double contact_swing_accept_ratio_;
    int    contact_on_count_threshold_;
    int    contact_off_count_threshold_;
    double pre_swing_advance_scale_;
    double stance_move_vertical_tol_{5e-4};
    double stance_move_cost_tol_{1e-4};
    double stance_move_min_dx_{1e-5};

    // ── walk state ────────────────────────────────────────────────────────────
    std::array<double, 4>              theta_{};
    std::array<double, 4>              beta_{};
    std::array<std::array<double,2>,4> hip_{};       // {x, z}
    std::array<std::array<double,2>,4> foothold_{};
    std::array<double, 4>              leg_step_length_{};
    std::array<int, 4>                 swing_mask_{0, 0, 0, 0};

    int  seq_idx_{0};
    int  swing_leg_{0};
    int  swing_tick_{0};
    int  pre_swing_steps_{0};
    int  pre_swing_tick_{0};
    double pre_swing_target_dist_{0.0};
    double pre_swing_advanced_dist_{0.0};
    double stance_move_carry_{0.0};
    bool needs_motor_sync_{true};
    bool late_probing_{false};
    int  probe_tick_{0};
    std::array<double,2> probe_point_{};
    int  support_advance_ticks_{0};
    int  support_advance_target_steps_{0};

    std::array<bool, 4> filtered_contact_{false, false, false, false};
    std::array<int, 4>  contact_on_count_{0, 0, 0, 0};
    std::array<int, 4>  contact_off_count_{0, 0, 0, 0};

    // attitude_stable latch — reset in start_swing(), set on input.attitude_stable
    bool attitude_stable_latch_{false};
    int  adjust_tick_{0};
    int  completed_cycles_{0};

    int  transform_count_;
    int  transform_tick_{0};

    double dS_;
    int    swing_steps_;

    SwingProfile          swing_profile_;
    std::array<double,2>  p_lo_{};
    std::array<double,2>  p_td_{};

    std::unique_ptr<LegModel> leg_model_;

    // ── private helpers ───────────────────────────────────────────────────────
    void do_transform(const ExternalInput & input, GaitOutput & out);
    void do_pre_swing(const ExternalInput & input, GaitOutput & out);
    void start_swing (int leg, const ExternalInput & input);
    void do_swing    (const ExternalInput & input, GaitOutput & out);
    void do_probing  (const ExternalInput & input, GaitOutput & out);
    void do_adjusting(const ExternalInput & input, GaitOutput & out);
    double advance_stance_body(const ExternalInput & input, bool sync_from_feedback, double dx);
    double advance_stance_legs(const ExternalInput & input, bool sync_from_feedback,
                               double dx, int skip_leg);
    bool update_contact_filter(int leg, const ExternalInput & input, double swing_progress);
    void enter_probing();
    void finish_touchdown(bool contact_hit, const char * reason);
    void enter_adjusting();
    void fill_output (GaitOutput & out) const;
    void sync_from_motor(const ExternalInput & input);
};

#endif // EVENT_WALK_GAIT_HPP
