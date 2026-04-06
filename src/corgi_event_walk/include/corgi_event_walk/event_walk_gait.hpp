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
    // ── I/O types ─────────────────────────────────────────────────────────────

    struct ExternalInput {
        bool trigger{false};
        bool attitude_stable{false};
        std::array<bool, 4>   contact{false, false, false, false};
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
                  int    sampling_rate,
                  int    max_adjust_steps = 3000,
                  double BL = 0.444,
                  double BW = 0.4);

    // ── main API ──────────────────────────────────────────────────────────────

    GaitOutput step(const ExternalInput & input);

    void set_velocity(double v);
    void set_stand_height(double h);

    std::array<int, 4> get_swing_phase() const;
    bool is_adjusting() const;
    bool is_ready()     const;
    bool is_ended()     const;

private:
    // ── state machine ─────────────────────────────────────────────────────────
    enum class Phase { INIT, TRANSFORM, READY, PRE_SWING, SWING, ADJUSTING, END };
    Phase phase_{Phase::INIT};

    static constexpr int    LEG_SEQ[4]  = {0, 2, 1, 3};
    static constexpr double SWING_TIME  = 0.2;
    static constexpr double INIT_THETA  = M_PI * 17.0 / 180.0;
    static constexpr double INIT_BETA   = 0.0;

    // Stand configuration: stand_height=0.25, step_length=0.3, swing_time=0.2
    static constexpr double INIT_ETA[8] = {
        1.857467698281913,   0.4791102940603915,   // FL (0)
        1.6046663223045279,  0.12914729012802004,  // FR (1)
        1.6046663223045279, -0.12914729012802004,  // RR (2)
        1.857467698281913,  -0.4791102940603915    // RL (3)
    };

    // ── parameters ────────────────────────────────────────────────────────────
    bool   sim_;
    double velocity_;
    double stand_height_;
    double step_length_;
    double step_height_;
    int    sampling_rate_;
    int    max_adjust_steps_;
    double BL_, BW_;

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
    bool needs_motor_sync_{true};

    // attitude_stable latch — reset in start_swing(), set on input.attitude_stable
    bool attitude_stable_latch_{false};
    int  adjust_tick_{0};

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
    void enter_adjusting();
    void fill_output (GaitOutput & out) const;
    void sync_from_motor(const ExternalInput & input);
};

#endif // EVENT_WALK_GAIT_HPP
