#pragma once

#include "common/Params.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

namespace corgi {

namespace detail {

/// Read a 3-element YAML sequence into an Eigen::Vector3f.
inline Eigen::Vector3f read_vec3f(const YAML::Node& node,
                                  const Eigen::Vector3f& fallback) {
    if (!node || !node.IsSequence() || node.size() != 3)
        return fallback;
    return {node[0].as<float>(), node[1].as<float>(), node[2].as<float>()};
}

/// Convenience: read scalar with default.
template <typename T>
inline T val(const YAML::Node& node, const std::string& key, T def) {
    if (node[key]) return node[key].as<T>();
    return def;
}

/// Read a 4-element YAML sequence into a std::array<float, 4>.
inline std::array<float, 4> read_arr4f(const YAML::Node& node,
                                       const std::array<float, 4>& fallback) {
    if (!node || !node.IsSequence() || node.size() != 4)
        return fallback;
    return {node[0].as<float>(), node[1].as<float>(),
            node[2].as<float>(), node[3].as<float>()};
}

}  // namespace detail

/**
 * @brief Load a Params struct from a YAML file.
 *
 * Missing keys fall back to the compiled-in defaults in Params.hpp.
 */
inline Params load_params(const std::string& yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load YAML config: " + yaml_path +
                                 "\n  " + e.what());
    }

    Params p;  // starts at compiled defaults

    // ── ESEKF noise ─────────────────────────────────────────────
    if (auto n = root["esekf"]) {
        p.sigma_a       = detail::read_vec3f(n["sigma_a"],       p.sigma_a);
        p.sigma_w       = detail::read_vec3f(n["sigma_w"],       p.sigma_w);
        p.sigma_ba      = detail::read_vec3f(n["sigma_ba"],      p.sigma_ba);
        p.sigma_bw      = detail::read_vec3f(n["sigma_bw"],      p.sigma_bw);
        p.sigma_leg_vec = detail::read_vec3f(n["sigma_leg_vec"], p.sigma_leg_vec);
        p.mahalanobis_threshold = detail::val(n, "mahalanobis_threshold",
                                              p.mahalanobis_threshold);
    }

    // ── Observer / filter ───────────────────────────────────────
    if (auto n = root["observer"]) {
        p.observer_cutoff_freq = detail::val(n, "cutoff_freq",        p.observer_cutoff_freq);
        p.encoder_cutoff_freq  = detail::val(n, "encoder_cutoff_freq", p.encoder_cutoff_freq);
    }

    // ── Contact Schmitt trigger ────────────────────────────────
    if (auto n = root["contact"]) {
        p.contact_rm_threshold_high   = detail::val(n, "rm_threshold_high",   p.contact_rm_threshold_high);
        p.contact_rm_threshold_low    = detail::val(n, "rm_threshold_low",    p.contact_rm_threshold_low);
        p.contact_beta_threshold_high = detail::val(n, "beta_threshold_high", p.contact_beta_threshold_high);
        p.contact_beta_threshold_low  = detail::val(n, "beta_threshold_low",  p.contact_beta_threshold_low);
    }

    // ── ZUPT ────────────────────────────────────────────────────
    if (auto n = root["zupt"]) {
        p.zupt_enabled     = detail::val(n, "enabled",      p.zupt_enabled);
        p.zupt_sigma_vec   = detail::read_vec3f(n["sigma_vec"], p.zupt_sigma_vec);
        p.zupt_gyro_thresh = detail::val(n, "gyro_thresh",  p.zupt_gyro_thresh);
    }

    // ── Static IMU initialization ──────────────────────────────
    if (auto n = root["static_init"]) {
        p.static_init_window_ms     = detail::val(n, "window_ms",          p.static_init_window_ms);
        p.static_motion_gyro_thresh = detail::val(n, "motion_gyro_thresh", p.static_motion_gyro_thresh);
        p.initial_z                 = detail::val(n, "initial_z",          p.initial_z);
    }

    // ── Logic switches ──────────────────────────────────────────
    p.simulate_imu_noise = detail::val(root, "simulate_imu_noise", p.simulate_imu_noise);
    p.use_esekf_state    = detail::val(root, "use_esekf_state",    p.use_esekf_state);
    p.use_bv_feedback    = detail::val(root, "use_bv_feedback",    p.use_bv_feedback);
    p.quiet              = detail::val(root, "quiet",              p.quiet);

    // ── Offline ─────────────────────────────────────────────────
    if (auto n = root["offline"]) {
        p.csv_filename   = detail::val<std::string>(n, "csv_filename", p.csv_filename);
        p.start_index    = detail::val(n, "start_index",    p.start_index);
        p.max_processed  = detail::val(n, "max_processed",  p.max_processed);
        p.rmse_skip      = detail::val(n, "rmse_skip",      p.rmse_skip);
        p.enable_logging = detail::val(n, "enable_logging", p.enable_logging);
        p.log_details    = detail::val(n, "log_details",    p.log_details);
        p.imu_noise_seed = detail::val(n, "imu_noise_seed", p.imu_noise_seed);
        p.use_dynamic_dt = detail::val(n, "use_dynamic_dt", p.use_dynamic_dt);
    }

    // ── Kinematics (Stage 1.5 camber correction) ───────────────
    // Absent block → compiled defaults (legacy behaviour, everything off).
    if (auto n = root["kinematics"]) {
        p.camber_enabled = detail::val(n, "camber_enabled", p.camber_enabled);
        p.radius_mode    = detail::val<std::string>(n, "radius_mode", p.radius_mode);
        if (p.radius_mode != "legacy" && p.radius_mode != "design" &&
            p.radius_mode != "calibrated") {
            throw std::runtime_error(
                "kinematics.radius_mode must be legacy|design|calibrated, got: " +
                p.radius_mode);
        }
        p.rolling_radius_calibrated = detail::val(n, "rolling_radius_calibrated",
                                                  p.rolling_radius_calibrated);
        if (p.radius_mode == "calibrated" && p.rolling_radius_calibrated <= 0.1f) {
            throw std::runtime_error(
                "kinematics.radius_mode=calibrated requires "
                "rolling_radius_calibrated > 0.1 (R of linkage joint circle)");
        }
        p.hip_y_abad_axis          = detail::val(n, "hip_y_abad_axis",
                                                 p.hip_y_abad_axis);
        p.abad_axis_to_wheel_plane = detail::val(n, "abad_axis_to_wheel_plane",
                                                 p.abad_axis_to_wheel_plane);
        p.wheel_half_width         = detail::val(n, "wheel_half_width",
                                                 p.wheel_half_width);
        p.gamma_signs = detail::read_arr4f(n["gamma_signs"], p.gamma_signs);

        if (auto e = n["eccentricity"]) {
            p.ecc_enabled = detail::val(e, "enabled", p.ecc_enabled);
            p.ecc_e       = detail::read_arr4f(e["e"],       p.ecc_e);
            p.ecc_phi_deg = detail::read_arr4f(e["phi_deg"], p.ecc_phi_deg);
        }
    }

    // ── GT velocity filter ──────────────────────────────────────
    p.gt_velocity_lpf_cutoff = detail::val(root, "gt_velocity_lpf_cutoff",
                                           p.gt_velocity_lpf_cutoff);

    return p;
}

}  // namespace corgi
