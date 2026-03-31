#ifndef WALK_UTILS_HPP
#define WALK_UTILS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "corgi_utils/leg_model.hpp"

// Global flag for simulation mode
extern bool sim;

// Global leg model instance
extern LegModel legmodel;

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param q Input quaternion
 * @param roll Output roll angle (rad)
 * @param pitch Output pitch angle (rad)
 * @param yaw Output yaw angle (rad)
 */
inline void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

#endif // WALK_UTILS_HPP
