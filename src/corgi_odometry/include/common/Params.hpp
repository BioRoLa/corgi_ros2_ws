#pragma once

#include <Eigen/Dense>

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
    bool use_bv_feedback    = false;   // Phase 3: feed outer-EKF bv back to inner ESEKF
    bool use_dynamic_dt     = true;    // use IMU timestamps for ESEKF propagation
};

}  // namespace corgi
