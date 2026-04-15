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
        p.sigma_bv      = detail::read_vec3f(n["sigma_bv"],      p.sigma_bv);
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

    // ── Logic switches ──────────────────────────────────────────
    p.simulate_imu_noise = detail::val(root, "simulate_imu_noise", p.simulate_imu_noise);
    p.use_esekf_state    = detail::val(root, "use_esekf_state",    p.use_esekf_state);
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

    // ── GT velocity filter ──────────────────────────────────────
    p.gt_velocity_lpf_cutoff = detail::val(root, "gt_velocity_lpf_cutoff",
                                           p.gt_velocity_lpf_cutoff);

    return p;
}

}  // namespace corgi
