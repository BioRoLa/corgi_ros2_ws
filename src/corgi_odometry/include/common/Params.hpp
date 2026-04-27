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

    // ── Logic switches ──────────────────────────────────────────
    bool simulate_imu_noise = false;
    bool use_esekf_state    = false;
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
};

}  // namespace corgi
