#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include <cstddef>
#include <cstdint>

namespace corgi {

/**
 * @brief Tunable parameters for the estimation pipeline.
 *
 * Physical constants (mass, geometry, polynomial coefficients) remain
 * in Config.hpp as constexpr.  Everything here is meant to be
 * adjustable at runtime via ROS 2 parameters / YAML / CLI.
 */
struct Params {
    // ── ESEKF noise ─────────────────────────────────────────────
    Eigen::Vector3f sigma_a       = {5.0f, 1.0f, 1.0f};
    Eigen::Vector3f sigma_w       = {0.001f, 0.01f, 0.001f};
    Eigen::Vector3f sigma_ba      = {1e-5f, 1e-5f, 1e-5f};
    Eigen::Vector3f sigma_bw      = {1e-8f, 1e-8f, 1e-8f};
    Eigen::Vector3f sigma_leg_vec = {0.05f, 1.5f, 1.2f};
    float mahalanobis_threshold   = 16.27f;

    // ── Observer / filter ───────────────────────────────────────
    double observer_cutoff_freq = 15.0;   // Disturbance observer LPF [Hz]
    double encoder_cutoff_freq  = 30.0;   // Encoder velocity LPF [Hz]

    // ── Contact Schmitt trigger ────────────────────────────────
    double contact_rm_threshold_high   = 25.0;
    double contact_rm_threshold_low    = 15.0;
    double contact_beta_threshold_high = 10.0;
    double contact_beta_threshold_low  =  1.0;

    // ── ZUPT (Zero Velocity Update) ────────────────────────────
    // Applied when all 4 legs are off the ground and gyro norm < zupt_gyro_thresh.
    bool            zupt_enabled    = true;
    Eigen::Vector3f zupt_sigma_vec  = {0.01f, 0.01f, 0.01f};  // velocity noise std [m/s]
    float           zupt_gyro_thresh = 0.3f;   // skip ZUPT if |w_corr| > this [rad/s]

    // ── Static IMU initialization ──────────────────────────────
    // Window length (ms) to average IMU before first ESEKF tick.
    // Accumulates in cb_imu(); used to estimate ba, bw, and gravity direction.
    // If fewer samples are available, degrades gracefully to ba=bw=0.
    int   static_init_window_ms      = 200;
    // Gyro norm threshold for motion detection during init window [rad/s].
    // If |w_mean| > this, log a warning (but still proceed).
    float static_motion_gyro_thresh  = 0.02f;
    // Initial z position for ESEKF [m].
    // Set to nominal hip height (~0.2 m) for consistency with legacy estimator.
    float initial_z                  = 0.2f;

    // ── Logic switches ──────────────────────────────────────────
    // simulate_imu_noise: offline_test only — adds synthetic noise to IMU.
    // use_esekf_state: sim / offline nodes only — feeds ESEKF-estimated
    //   position/velocity/orientation back into the GMO pipeline instead of
    //   external ground-truth topics.  The online real-robot node (corgi_leg_odom)
    //   hard-codes this to true and does NOT read it from config_online.yaml.
    bool simulate_imu_noise = false;
    bool use_esekf_state    = false;
    bool use_bv_feedback    = false;   // Phase 3: feed outer-EKF bv back to inner ESEKF
    bool quiet              = false;

    // ── Offline ─────────────────────────────────────────────────
    std::string csv_filename = "walk_2m_01";
    int    start_index       = 0;
    size_t max_processed     = 12000;
    size_t rmse_skip         = 0;
    bool   enable_logging    = true;
    bool   log_details       = false;
    uint64_t imu_noise_seed  = 42;
    bool   use_dynamic_dt    = true;   // use IMU timestamps for ESEKF dt (offline)

    // ── Kinematics (Stage 1.5 camber correction) ───────────────
    // Defaults preserve legacy behaviour exactly: camber off, legacy
    // radius, sagittal-only contact model.  Old YAMLs (no `kinematics:`
    // block) load unchanged.
    //
    // camber_enabled: rotate the contact point/velocity about the ABAD
    //   axis by the measured per-leg gamma (ABAD-axis frame: leg offset
    //   y becomes ±hip_y_abad_axis and the wheel-plane arm moves into
    //   d_wheel = abad_axis_to_wheel_plane ± wheel_half_width).
    //   Never sum the legacy 0.193 with abad_axis_to_wheel_plane.
    bool camber_enabled = false;

    // radius_mode: "legacy" | "design" | "calibrated"
    //   legacy     → r_skin = 0.019  (Config::TIRE_SKIN_RADIUS, rim 0.119)
    //   design     → r_skin = 0.045  (LegWheel design outer 0.145)
    //   calibrated → r_skin = rolling_radius_calibrated − 0.1
    // R = 0.1 (Config::WHEEL_RADIUS) is the linkage joint circle and
    // never changes.
    std::string radius_mode = "legacy";
    // Effective rolling radius from offline calibration [m].
    // 0.14482 = stage15 fit (sim, roll state, kp 500) — set via YAML;
    // struct default 0 forces an explicit value when mode=calibrated.
    float rolling_radius_calibrated = 0.0f;

    // ABAD frame geometry [m] (sim, CorgiRobotABAD.proto)
    float hip_y_abad_axis         = 0.12f;      // |y| of ABAD hinge axis
    float abad_axis_to_wheel_plane = 0.091675f; // axis → wheel mid-plane arm
    float wheel_half_width        = 0.02f;      // half wheel width (edge offset)

    // Per-leg gamma sign map, order A/B/C/D = LF/RF/RH/LH.
    // Adjudicated 2026-08-19 (sim wheel mode): raw motor gamma is the
    // local tilt for every leg → all +1.
    std::array<float, 4> gamma_signs = {1.0f, 1.0f, 1.0f, 1.0f};

    // Wheel mode (closed wheel, stage15): when true AND a leg's measured
    // theta < wheel_theta_max_deg, that leg is treated as a closed wheel:
    // ContactMap is skipped (its domain starts at theta = 17 deg, below
    // the achieved wheel closure ~16.2 deg), contact comes from the GMO
    // Schmitt state alone, and the contact point/velocity use the rim
    // model (offset + R_x(gamma)·(0, sy·d_wheel, −r_eff), rolling term
    // −spin·r_eff·x̂ with spin = UNFLIPPED beta_dot).
    bool  wheel_mode = false;
    float wheel_theta_max_deg = 20.0f;

    // k=1 rolling-radius modulation r(β) = r0 + e·cos(β + φ), per leg.
    // Only active when ecc_enabled AND camber_enabled.
    bool ecc_enabled = false;
    std::array<float, 4> ecc_e       = {0.0f, 0.0f, 0.0f, 0.0f};  // [m]
    std::array<float, 4> ecc_phi_deg = {0.0f, 0.0f, 0.0f, 0.0f};  // [deg]

    /// Little-r (tire skin radius) for Leg construction per radius_mode.
    /// Rolling rim radius = r_skin + R (R = 0.1 linkage joint circle).
    float r_skin() const {
        if (radius_mode == "design")     return 0.045f;   // → rim 0.145
        if (radius_mode == "calibrated") return rolling_radius_calibrated - 0.1f;
        return 0.019f;                                    // legacy → rim 0.119
    }

    // ── GT velocity filter ──────────────────────────────────────
    double gt_velocity_lpf_cutoff = 10.0;  // [Hz]

    // ── Fake LiDAR (simulation/offline) ────────────────────────
    float  fake_lidar_sigma_p    = 0.02f;   // position noise std dev [m]
    float  fake_lidar_sigma_q    = 0.005f;  // orientation noise std dev [rad]
    double fake_lidar_latency_ms = 80.0;    // publish latency [ms]
    double fake_lidar_rate_hz    = 10.0;    // publish rate [Hz]
};

}  // namespace corgi
