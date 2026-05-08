#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace fusion {

// ================================================================
// Outer Fusion EKF  (9-dim error state)
//
// Purpose: loosely fuse Inner-ESEKF odometry with LiDAR (or any
//          6-DOF absolute pose sensor) to maintain a map→odom
//          transform and estimate systematic velocity bias bv.
//
// Nominal state (9-dim concept, stored separately):
//   p_mo  [3]  – origin of odom frame expressed in map frame
//   q_mo  [4]  – orientation odom→map (unit quaternion)
//   bv    [3]  – systematic velocity bias in world/map frame [m/s]
//
// Error state (9-dim):
//   δp_mo [0-2]
//   δθ_mo [3-5]  – rotation vector (body-frame perturbation)
//   δbv   [6-8]
// ================================================================

class OuterEKF {
public:
    // ── Index constants ────────────────────────────────────────
    static constexpr int P_IDX  = 0;
    static constexpr int TH_IDX = 3;
    static constexpr int BV_IDX = 6;
    static constexpr int DIM    = 9;

    // ── Tunable noise ──────────────────────────────────────────
    struct NoiseParams {
        // Process noise (per second, added as Q*dt each predict)
        float q_p   = 1e-4f;   // position drift variance/s   [m²/s]
        float q_th  = 1e-5f;   // rotation drift variance/s   [rad²/s]
        float q_bv  = 1e-5f;   // bv random walk variance/s   [(m/s)²/s]

        // LiDAR measurement noise (fixed, isotropic)
        float r_p   = 4e-4f;   // position noise variance      [m²]  (σ=0.02m)
        float r_th  = 2.5e-5f; // rotation noise variance      [rad²](σ=0.005rad)
    };

    // ── Initial error covariance constants ────────────────────
    struct InitCovariance {
        static constexpr float p_mo_m2  = 1.0f;    ///< p_mo initial variance  [m²]   (±1 m)
        static constexpr float th_mo_r2 = 0.01f;   ///< θ_mo initial variance  [rad²] (±0.1 rad)
        static constexpr float bv_ms2   = 1e-4f;   ///< bv  initial variance   [(m/s)²] (±0.01 m/s)
    };

    // ── Per-update diagnostics ─────────────────────────────────
    struct UpdateDiag {
        Eigen::Vector3f innov_p  = Eigen::Vector3f::Zero();
        Eigen::Vector3f innov_th = Eigen::Vector3f::Zero();
        float mahal_p  = 0.f;
        float mahal_th = 0.f;
        bool  valid    = false;
    };

    // ── Constructor / init ─────────────────────────────────────
    OuterEKF();

    void set_noise(const NoiseParams& np) { noise_ = np; }

    /// Initialise with known map→odom transform.
    /// Call once when the first LiDAR measurement arrives.
    void init(const Eigen::Vector3f& p_mo,
              const Eigen::Quaternionf& q_mo);

    bool initialized() const { return initialized_; }

    // ── EKF steps ──────────────────────────────────────────────

    /// Predict step – call at inner-ESEKF rate (or decimated).
    /// dt = elapsed time since last predict [s].
    void predict(float dt);

    /// LiDAR update step.
    ///
    /// @param p_lidar   absolute position from LiDAR in map frame
    /// @param q_lidar   absolute orientation from LiDAR (map←body)
    /// @param p_odom    inner-ESEKF position at LiDAR stamp (odom frame)
    /// @param q_odom    inner-ESEKF orientation at LiDAR stamp (odom←body)
    /// @param dt_since_last_lidar  elapsed time since previous lidar [s];
    ///        used to build the bv-observability coupling in H.
    void update_lidar(const Eigen::Vector3f&    p_lidar,
                      const Eigen::Quaternionf& q_lidar,
                      const Eigen::Vector3f&    p_odom,
                      const Eigen::Quaternionf& q_odom,
                      float                     dt_since_last_lidar,
                      UpdateDiag*               diag = nullptr);

    // ── Accessors ──────────────────────────────────────────────
    const Eigen::Vector3f&    p_mo()     const { return p_mo_; }
    const Eigen::Quaternionf& q_mo()     const { return q_mo_; }
    const Eigen::Vector3f&    bv()       const { return bv_; }
    const Eigen::MatrixXf&    covariance() const { return P_; }

    /// Compute the fused position in map frame:
    ///   p_map = p_mo + R_mo * p_odom
    Eigen::Vector3f map_position(const Eigen::Vector3f& p_odom) const;

    /// Compute the fused orientation in map frame:
    ///   q_map = q_mo * q_odom
    Eigen::Quaternionf map_orientation(const Eigen::Quaternionf& q_odom) const;

private:
    static Eigen::Matrix3f skew(const Eigen::Vector3f& v);

    // Nominal state
    Eigen::Vector3f    p_mo_  = Eigen::Vector3f::Zero();
    Eigen::Quaternionf q_mo_  = Eigen::Quaternionf::Identity();
    Eigen::Vector3f    bv_    = Eigen::Vector3f::Zero();

    // Error covariance (9×9)
    Eigen::MatrixXf P_;

    NoiseParams noise_;
    bool initialized_ = false;
};

}  // namespace fusion
