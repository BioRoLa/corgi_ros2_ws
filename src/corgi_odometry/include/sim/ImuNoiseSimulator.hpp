#pragma once
/**
 * @file ImuNoiseSimulator.hpp
 * @brief Realistic IMU noise model based on 3DM-CX5-AHRS datasheet (25°C).
 *
 * Noise components:
 *   1. Additive white Gaussian noise   (from noise density × √fs)
 *   2. Initial bias error              (drawn once at construction)
 *   3. In-run bias instability          (random-walk / 1st-order Gauss–Markov)
 *
 * Usage:
 *   ImuNoiseSimulator sim(dt);          // dt = 1/fs
 *   sim.apply(acc, gyro);               // modifies vectors in-place
 *
 * All units SI: m/s², rad/s, seconds.
 */

#include <random>
#include <cmath>
#include <cstdint>
#include <Eigen/Dense>

class ImuNoiseSimulator {
public:
    /**
     * @param dt       Sampling period [s] (1/fs). Default fs = 1000 Hz → dt = 0.001.
     * @param seed     RNG seed. 0 = use std::random_device (non-deterministic).
     */
    explicit ImuNoiseSimulator(double dt, uint64_t seed = 0)
        : dt_(dt), fs_(1.0 / dt)
    {
        // Seed the RNG
        if (seed == 0) {
            std::random_device rd;
            rng_.seed(rd());
        } else {
            rng_.seed(seed);
        }

        // ── Precompute per-sample noise standard deviations ─────────
        // Accelerometer white noise
        //   PSD = 20 µg/√Hz = 20e-6 × 9.81 m/s²/√Hz
        //   Per-sample σ = PSD × √fs
        sigma_acc_white_ = ACCEL_NOISE_DENSITY * std::sqrt(fs_);

        // Gyroscope white noise (ARW)
        //   PSD = 0.005 °/s/√Hz = 0.005 × π/180 rad/s/√Hz
        //   Per-sample σ = PSD × √fs
        sigma_gyro_white_ = GYRO_NOISE_DENSITY * std::sqrt(fs_);

        // Accelerometer bias random-walk step σ
        //   Bias instability = 0.04 mg → 0.04e-3 × 9.81 m/s²
        //   Random-walk increment σ = bias_instability × √dt
        sigma_acc_rw_ = ACCEL_BIAS_INSTABILITY * std::sqrt(dt_);

        // Gyroscope bias random-walk step σ
        //   Bias instability = 8 °/hr → 8 × π/180 / 3600 rad/s
        //   Random-walk increment σ = bias_instability × √dt
        sigma_gyro_rw_ = GYRO_BIAS_INSTABILITY * std::sqrt(dt_);

        // ── Draw initial bias errors (uniform) ─────────────────────
        // Accel: ±0.002 g = ±0.01962 m/s²
        std::uniform_real_distribution<double> acc_bias_dist(-ACCEL_INIT_BIAS, ACCEL_INIT_BIAS);
        bias_acc_ = Eigen::Vector3d(acc_bias_dist(rng_), acc_bias_dist(rng_), acc_bias_dist(rng_));

        // Gyro: ±0.04 °/s = ±6.981e-4 rad/s
        std::uniform_real_distribution<double> gyro_bias_dist(-GYRO_INIT_BIAS, GYRO_INIT_BIAS);
        bias_gyro_ = Eigen::Vector3d(gyro_bias_dist(rng_), gyro_bias_dist(rng_), gyro_bias_dist(rng_));
    }

    /**
     * @brief Apply noise in-place to accelerometer and gyroscope readings.
     * @param acc   3-axis accelerometer (modified in-place) [m/s²]
     * @param gyro  3-axis gyroscope     (modified in-place) [rad/s]
     */
    void apply(Eigen::Vector3f& acc, Eigen::Vector3f& gyro) {
        std::normal_distribution<double> N(0.0, 1.0);

        // ── Random-walk update of biases ────────────────────────────
        for (int i = 0; i < 3; ++i) {
            bias_acc_(i)  += sigma_acc_rw_  * N(rng_);
            bias_gyro_(i) += sigma_gyro_rw_ * N(rng_);
        }

        // ── Add bias + white noise ──────────────────────────────────
        for (int i = 0; i < 3; ++i) {
            acc(i)  += static_cast<float>(bias_acc_(i)  + sigma_acc_white_  * N(rng_));
            gyro(i) += static_cast<float>(bias_gyro_(i) + sigma_gyro_white_ * N(rng_));
        }
    }

    /** @brief Reset biases and re-seed RNG. */
    void reset(uint64_t seed = 0) {
        if (seed == 0) {
            std::random_device rd;
            rng_.seed(rd());
        } else {
            rng_.seed(seed);
        }

        std::uniform_real_distribution<double> acc_bias_dist(-ACCEL_INIT_BIAS, ACCEL_INIT_BIAS);
        bias_acc_ = Eigen::Vector3d(acc_bias_dist(rng_), acc_bias_dist(rng_), acc_bias_dist(rng_));

        std::uniform_real_distribution<double> gyro_bias_dist(-GYRO_INIT_BIAS, GYRO_INIT_BIAS);
        bias_gyro_ = Eigen::Vector3d(gyro_bias_dist(rng_), gyro_bias_dist(rng_), gyro_bias_dist(rng_));
    }

    // ── Accessors (for logging / diagnostics) ───────────────────────
    const Eigen::Vector3d& accel_bias()  const { return bias_acc_; }
    const Eigen::Vector3d& gyro_bias()   const { return bias_gyro_; }
    double sample_rate()                 const { return fs_; }

private:
    // ── 3DM-CX5-AHRS datasheet constants (25 °C) ───────────────────
    // Accelerometer
    static constexpr double ACCEL_NOISE_DENSITY   = 20e-6 * 9.81;          // 20 µg/√Hz → m/s²/√Hz
    static constexpr double ACCEL_INIT_BIAS        = 0.002 * 9.81;          // ±0.002 g  → m/s²
    static constexpr double ACCEL_BIAS_INSTABILITY = 0.04e-3 * 9.81;       // 0.04 mg   → m/s²

    // Gyroscope
    static constexpr double GYRO_NOISE_DENSITY    = 0.005 * M_PI / 180.0;  // 0.005 °/s/√Hz → rad/s/√Hz
    static constexpr double GYRO_INIT_BIAS         = 0.04 * M_PI / 180.0;   // ±0.04 °/s    → rad/s
    static constexpr double GYRO_BIAS_INSTABILITY  = 8.0 * M_PI / 180.0 / 3600.0; // 8 °/hr → rad/s

    // ── Runtime state ───────────────────────────────────────────────
    double dt_;                     // Sampling period [s]
    double fs_;                     // Sampling rate [Hz]
    std::mt19937_64 rng_;           // Mersenne-Twister RNG

    double sigma_acc_white_;        // Per-sample accel white noise σ
    double sigma_gyro_white_;       // Per-sample gyro  white noise σ
    double sigma_acc_rw_;           // Per-step accel bias random-walk σ
    double sigma_gyro_rw_;          // Per-step gyro  bias random-walk σ

    Eigen::Vector3d bias_acc_;      // Current accel bias [m/s²]
    Eigen::Vector3d bias_gyro_;     // Current gyro  bias [rad/s]
};
