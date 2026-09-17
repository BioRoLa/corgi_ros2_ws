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
        p.sigma_leg_vec = detail::read_vec3f(n["sigma_leg_vec"], p.sigma_leg_vec);
        p.mahalanobis_threshold = detail::val(n, "mahalanobis_threshold",
                                              p.mahalanobis_threshold);
    }

    // ── Observer / filter ───────────────────────────────────────
    if (auto n = root["observer"]) {
        p.observer_cutoff_freq = detail::val(n, "cutoff_freq",        p.observer_cutoff_freq);
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
    p.use_bv_feedback    = detail::val(root, "use_bv_feedback",    p.use_bv_feedback);
    p.use_dynamic_dt     = detail::val(root, "use_dynamic_dt",     p.use_dynamic_dt);

    return p;
}

}  // namespace corgi
