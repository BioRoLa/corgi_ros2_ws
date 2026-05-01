#pragma once
/**
 * @file FusionParamsIO.hpp
 * @brief yaml-cpp loader for FusionConfig (Outer EKF noise + frame params).
 *
 * Missing YAML keys fall back to the compiled-in defaults in OuterEKF::NoiseParams.
 * Pattern mirrors common/ParamsIO.hpp.
 */

#include "fusion/OuterEKF.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>

namespace fusion {

struct FusionConfig {
    OuterEKF::NoiseParams noise;           ///< EKF process / measurement noise
    std::string           map_frame   = "map";
    std::string           odom_frame  = "odom";
};

/**
 * @brief Load FusionConfig from a YAML file.
 *
 * Expected layout (all keys optional, fall back to defaults):
 * @code
 * fusion:
 *   q_p:   1.0e-4
 *   q_th:  1.0e-5
 *   q_bv:  1.0e-5
 *   r_p:   4.0e-4
 *   r_th:  2.5e-5
 *   map_frame:  "map"
 *   odom_frame: "odom"
 * @endcode
 */
inline FusionConfig load_fusion_config(const std::string& yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            "Failed to load fusion YAML: " + yaml_path + "\n  " + e.what());
    }

    FusionConfig cfg;   // starts at header defaults

    if (auto n = root["fusion"]) {
        if (n["q_p"])        cfg.noise.q_p        = n["q_p"].as<float>();
        if (n["q_th"])       cfg.noise.q_th       = n["q_th"].as<float>();
        if (n["q_bv"])       cfg.noise.q_bv       = n["q_bv"].as<float>();
        if (n["r_p"])        cfg.noise.r_p        = n["r_p"].as<float>();
        if (n["r_th"])       cfg.noise.r_th       = n["r_th"].as<float>();
        if (n["map_frame"])  cfg.map_frame        = n["map_frame"].as<std::string>();
        if (n["odom_frame"]) cfg.odom_frame       = n["odom_frame"].as<std::string>();
    }

    return cfg;
}

}  // namespace fusion
