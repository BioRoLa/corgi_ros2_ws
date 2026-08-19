#pragma once
/**
 * @file FakeLidarSimulator.hpp
 * @brief Shared fake-LiDAR noise model for simulation.
 *
 * Applies Gaussian position noise and small-angle orientation noise to a pose.
 * Used by both the online ROS2 node (fake_lidar_odom) and the offline test
 * runner (OfflineTestNode), keeping their noise behaviour identical.
 *
 * No ROS 2 dependencies — mirrors the pattern of ImuNoiseSimulator.hpp.
 *
 * Usage:
 *   FakeLidarSimulator sim;                                      // defaults
 *   FakeLidarSimulator sim(FakeLidarParams{0.02f, 0.005f, 42}); // explicit
 *   Eigen::Vector3f p_noisy; Eigen::Quaternionf q_noisy;
 *   sim.apply_noise(p_clean, q_clean, p_noisy, q_noisy);
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <cstdint>

namespace sim {

/// Parameters for FakeLidarSimulator. Defined outside the class to avoid
/// GCC's restriction on using nested-struct default member initialisers
/// in the enclosing class constructor default argument.
struct FakeLidarParams {
    float    sigma_p = 0.02f;   ///< Position noise std dev [m]
    float    sigma_q = 0.005f;  ///< Orientation noise std dev [rad] (axis-angle)
    uint64_t seed    = 12345;   ///< RNG seed (0 = non-deterministic)
};

class FakeLidarSimulator {
public:
    // Keep Params as an alias so call sites can still write
    // FakeLidarSimulator::Params{...}
    using Params = FakeLidarParams;

    FakeLidarSimulator() : FakeLidarSimulator(Params{}) {}

    explicit FakeLidarSimulator(const Params& p)
        : params_(p), dist_(0.0, 1.0)
    {
        if (p.seed == 0) {
            std::random_device rd;
            rng_.seed(rd());
        } else {
            rng_.seed(p.seed);
        }
    }

    /**
     * @brief Apply Gaussian position noise and small-angle orientation noise.
     *
     * @param p_in    Clean position
     * @param q_in    Clean orientation
     * @param p_out   Noisy position output
     * @param q_out   Noisy orientation output
     */
    void apply_noise(const Eigen::Vector3f&    p_in,
                     const Eigen::Quaternionf& q_in,
                     Eigen::Vector3f&          p_out,
                     Eigen::Quaternionf&       q_out)
    {
        p_out = p_in + params_.sigma_p * Eigen::Vector3f(
            sample(), sample(), sample());

        Eigen::Vector3f ax(sample(), sample(), sample());
        float ax_norm = ax.norm();
        if (ax_norm < 1e-6f) ax = Eigen::Vector3f(1.f, 0.f, 0.f); else ax /= ax_norm;
        float noise_ang = params_.sigma_q * sample();
        Eigen::Quaternionf dq(Eigen::AngleAxisf(noise_ang, ax));
        q_out = (q_in * dq).normalized();
    }

    float sigma_p() const { return params_.sigma_p; }
    float sigma_q() const { return params_.sigma_q; }

private:
    float sample() { return static_cast<float>(dist_(rng_)); }

    Params                           params_;
    std::mt19937                     rng_;
    std::normal_distribution<double> dist_;
};

}  // namespace sim
