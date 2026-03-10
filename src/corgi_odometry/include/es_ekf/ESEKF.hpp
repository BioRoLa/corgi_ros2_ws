#ifndef ESEKF_HPP
#define ESEKF_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "kinematic/Leg.hpp"
#include "kinematic/ContactMap.hpp"
#include <vector>
#include <array>

namespace estimation_model {

// ============================================================
// Error state index definitions (18-dim error state)
// Attitude uses 3-dim rotation vector (not 4-dim quaternion)
// ============================================================
enum StateIdx {
    P_IDX  = 0,   // position error         (3)
    V_IDX  = 3,   // velocity error          (3)
    TH_IDX = 6,   // attitude error (rot vec)(3)
    BA_IDX = 9,   // accelerometer bias error(3)
    BW_IDX = 12,  // gyroscope bias error    (3)
    BV_IDX = 15,  // velocity bias error     (3)
    ERR_STATE_DIM = 18
};

// ============================================================
// Nominal state (19-dim, quaternion for attitude)
// ============================================================
struct NominalState {
    Eigen::Vector3f p  = Eigen::Vector3f::Zero();              // position in world frame
    Eigen::Vector3f v  = Eigen::Vector3f::Zero();              // velocity in body frame
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();     // attitude (body->world)
    Eigen::Vector3f ba = Eigen::Vector3f::Zero();              // accelerometer bias
    Eigen::Vector3f bw = Eigen::Vector3f::Zero();              // gyroscope bias
    Eigen::Vector3f bv = Eigen::Vector3f::Zero();              // velocity bias (extended, init 0)
};

// ============================================================
// Process & measurement noise parameters
// ============================================================
struct NoiseParams {
    // IMU noise (from original system: da, dw)
    Eigen::Vector3f sigma_a  = {1.0f,  1.0f,  1.0f};          // accel noise std
    Eigen::Vector3f sigma_w  = {0.001f, 0.01f, 0.001f};             // gyro noise std

    // Bias random walk (from original system: dba, new: dbw, dbv)
    Eigen::Vector3f sigma_ba = {1e-5f, 1e-5f, 1e-5f}; // accel bias
    Eigen::Vector3f sigma_bw = {1e-8f, 1e-8f, 1e-8f};             // gyro bias
    Eigen::Vector3f sigma_bv = {1e-6f, 1e-6f, 1e-6f};             // velocity bias

    // Leg measurement noise per axis: sigma_leg_vec = [std_x, std_y, std_z] [m/s].
    // R_leg = diag(sigma_leg_vec.sq())  (per-axis variance, NOT std).
    //
    // Motivation for anisotropic R:
    //   X/Z: Leg FK RMSE ≈ 10.5 mm/s → sigma ≈ 0.011 (tight, trustworthy)
    //   Y:   Leg FK has large lateral error due to y-offset coupling
    //         → sigma_y = 0.1 (10x looser, prevents Y noise contaminating X)
    //
    // Previous bug: scalar sigma_leg=1e-3 used directly as variance (std≈31.6mm/s).
    Eigen::Vector3f sigma_leg_vec = {8.0f,  0.1f,  1.2f};
};

// ============================================================
// Per-leg observation data for the update step
// ============================================================
struct LegObservation {
    Leg* leg;             // pointer to the Leg kinematic model
    float theta;          // motor opening angle
    float theta_d;        // motor opening angle velocity
    float beta;           // motor rotation angle
    float beta_d;         // motor rotation angle velocity
    RIM rim;              // which rim section is in contact
    float alpha;          // contact angle relative to body frame (contact_beta - beta)
    bool in_contact;      // whether this leg is in ground contact
};

// ============================================================
// ES-EKF (Error-State Extended Kalman Filter)
//
// Predict: IMU propagates nominal state + error covariance
// Update:  Instantaneous no-slip velocity constraint per leg
// Inject:  Error state correction into nominal + reset
// ============================================================
class ESEKF {
public:
    /// @brief Constructor
    /// @param dt  time step [s]
    ESEKF(float dt);

    /// @brief Initialize nominal state (call once at startup)
    void init(const NominalState& x0);

    /// @brief IMU prediction step:
    ///   - Propagate nominal state (position, velocity, attitude, biases)
    ///   - Propagate error-state covariance P via linearized Jacobian Fx
    /// @param a_m  raw accelerometer measurement in body frame [m/s^2]
    /// @param w_m  raw gyroscope measurement in body frame [rad/s]
    void predict(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m);

    /// @brief Single-leg measurement update (sequential EKF update)
    ///   Observation model:  z_leg = v + bv + noise
    ///   where z_leg = -PointVelocity(v=0, w_imu) from encoder kinematics
    /// @param obs  leg observation data (encoder state + contact info)
    /// @param w_m  raw gyroscope measurement (needed for ω×r term)
    void update_leg(LegObservation& obs, const Eigen::Vector3f& w_m);

    /// @brief Update all legs sequentially
    /// @param obs       vector of 4 leg observations [LF, RF, RH, LH]
    /// @param w_m       raw gyroscope measurement
    /// @param exclude   per-leg exclude flags (true = skip this leg)
    void update_all_legs(std::vector<LegObservation>& obs,
                         const Eigen::Vector3f& w_m,
                         const std::array<bool, 4>& exclude);

    /// @brief Inject error state into nominal state, then reset δx = 0
    void inject_and_reset();

    /// @brief Get current nominal state (read-only)
    const NominalState& nominal() const { return x_nom_; }

    /// @brief Get current error-state covariance (read-only)
    const Eigen::MatrixXf& covariance() const { return P_; }

    /// @brief Set noise parameters (optional, has defaults)
    void set_noise_params(const NoiseParams& params) { noise_ = params; }

private:
    // --- Helper ---
    /// @brief Compute skew-symmetric matrix [v]×
    static Eigen::Matrix3f skew(const Eigen::Vector3f& v);

    // --- State ---
    NominalState x_nom_;                 // nominal state (19-dim)
    Eigen::VectorXf dx_;                 // error state   (18-dim)
    Eigen::MatrixXf P_;                  // error covariance (18×18)

    // --- Parameters ---
    float dt_;
    NoiseParams noise_;
};

} // namespace estimation_model

#endif // ESEKF_HPP
