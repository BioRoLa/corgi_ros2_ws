// Reading the per-leg direction table, corgi_utils/config/module_directions.yaml
// (CONVENTIONS.md section 5). The robot bridge reads it through this header;
// the sim driver reads the same file in Python (corgi_sim
// corgi_driver_pkg/module_directions.py) with the same rules.
//
// Header-only; an includer links yaml-cpp.
#pragma once

#include <array>
#include <set>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "corgi_utils/module_frame.hpp"

namespace corgi {

inline const std::array<std::string, 4>& module_leg_names()
{
    static const std::array<std::string, 4> names = {"A", "B", "C", "D"};
    return names;
}

// Every leg and every key (theta, beta, gamma) is required, and nothing else
// is allowed: a boundary running with the wrong directions drives mirrored legs
// the wrong way, so a missing value is an error, never a default of +1. The
// unknown-key check also stops an old-format table (joint_dir / motor_dir) from
// being half-read.
inline std::array<ModuleFrame, 4> load_module_directions(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("cannot read " + path + ": " + e.what());
    }
    static const std::set<std::string> keys = {"theta", "beta", "gamma"};
    std::array<ModuleFrame, 4> frames;
    for (size_t i = 0; i < frames.size(); ++i) {
        const std::string& leg = module_leg_names()[i];
        const YAML::Node node = root[leg];
        if (!node || !node.IsMap()) {
            throw std::runtime_error("leg " + leg + " is missing from " + path);
        }
        for (const auto& kv : node) {
            const auto key = kv.first.as<std::string>();
            if (keys.count(key) == 0) {
                throw std::runtime_error("leg " + leg + ": unknown key '" + key + "' in " + path +
                                         " (expected theta, beta, gamma; CONVENTIONS.md section 5)");
            }
        }
        ModuleDirections d;
        for (const auto& key : keys) {
            if (!node[key]) {
                throw std::runtime_error("leg " + leg + ": " + key + " is missing from " + path);
            }
        }
        d.theta = node["theta"].as<double>();
        d.beta = node["beta"].as<double>();
        d.gamma = node["gamma"].as<double>();
        frames[i] = ModuleFrame(d, leg);  // throws on non-unit or theta = -1
    }
    return frames;
}

}  // namespace corgi
