#pragma once

#include <Eigen/Dense>
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

    // ── GT velocity filter ──────────────────────────────────────
    double gt_velocity_lpf_cutoff = 10.0;  // [Hz]

    // ── Fake LiDAR (simulation/offline) ────────────────────────
    float  fake_lidar_sigma_p    = 0.02f;   // position noise std dev [m]
    float  fake_lidar_sigma_q    = 0.005f;  // orientation noise std dev [rad]
    double fake_lidar_latency_ms = 80.0;    // publish latency [ms]
    double fake_lidar_rate_hz    = 10.0;    // publish rate [Hz]
};

}  // namespace corgi
