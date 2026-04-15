#include "es_ekf/ESEKF.hpp"
#include <cmath>

namespace estimation_model {

// ============================================================
// Static helper
// ============================================================

Eigen::Matrix3f ESEKF::skew(const Eigen::Vector3f& v) {
    Eigen::Matrix3f S;
    S <<    0,  -v(2),  v(1),
         v(2),     0,  -v(0),
        -v(1),  v(0),     0;
    return S;
}

// ============================================================
// Constructor & Init
// ============================================================

ESEKF::ESEKF() {
    dx_ = Eigen::VectorXf::Zero(ERR_STATE_DIM);

    // Initial error covariance: small for biases, larger for position/velocity
    P_ = Eigen::MatrixXf::Identity(ERR_STATE_DIM, ERR_STATE_DIM) * 1e-4f;
    P_.block<3,3>(P_IDX, P_IDX)  *= 100.0f;   // position uncertainty
    P_.block<3,3>(V_IDX, V_IDX)  *= 100.0f;   // velocity uncertainty
    // Roll/yaw attitude: very small since unobservable from legs
    P_(TH_IDX + 0, TH_IDX + 0) = 1e-8f;  // roll:  trust IMU integration
    P_(TH_IDX + 2, TH_IDX + 2) = 1e-8f;  // yaw:   trust IMU integration
    // bw: all tiny.  bw_y (pitch gyro bias) is partially observable via the
    // H(bw_y) Jacobian, but in simulation the real bias is 0.  Starting with
    // large P_bw_y (1e-4) allows early spurious drift that corrupts attitude.
    // Setting all to 1e-10 locks bw_y, consistent with sigma_bw=1e-8.
    P_(BW_IDX + 0, BW_IDX + 0) = 1e-10f;
    P_(BW_IDX + 1, BW_IDX + 1) = 1e-10f;  // was 1e-4: caused bw_y→0.017 rad/s divergence
    P_(BW_IDX + 2, BW_IDX + 2) = 1e-10f;
}

void ESEKF::init(const NominalState& x0) {
    x_nom_ = x0;
    dx_.setZero();
}

// ============================================================
// Predict (IMU propagation)
// ============================================================

void ESEKF::predict(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m, float dt) {
    // ----------------------------------------------------------
    // 1. Bias-corrected IMU measurements
    // ----------------------------------------------------------
    // Gravity in world frame
    static const Eigen::Vector3f g_world(0.0f, 0.0f, -9.81f);
    
    Eigen::Vector3f a_hat = a_m - x_nom_.ba;   // corrected acceleration
    Eigen::Vector3f w_hat = w_m - x_nom_.bw;   // corrected angular velocity

    // Current body-to-world rotation
    Eigen::Matrix3f R_body = x_nom_.q.toRotationMatrix();

    Eigen::Vector3f g_body = R_body.transpose() * g_world;  // gravity in body frame


    // ----------------------------------------------------------
    // 2. Rotation increment R_delta = Exp(w_hat * dt)
    // ----------------------------------------------------------
    Eigen::Vector3f dw = w_hat * dt;
    float angle = dw.norm();
    Eigen::Matrix3f R_delta;
    if (angle > 1e-8f) {
        R_delta = Eigen::AngleAxisf(angle, dw / angle).toRotationMatrix();
    } else {
        R_delta = Eigen::Matrix3f::Identity();
    }

    // ----------------------------------------------------------
    // 3. Nominal state propagation (no uncertainty)
    //    System estimates velocity in body frame
    //
    //    p_{k+1} = p_k + R(q_k) * v_k * dt
    //    v_{k+1} = R_delta * v_k + R_delta * a_hat * dt
    //    q_{k+1} = q_k ⊗ q(R_delta)
    //    biases:  unchanged
    // ----------------------------------------------------------
    x_nom_.p += R_body * x_nom_.v * dt;
    x_nom_.v  = R_delta.transpose() * x_nom_.v + (a_hat + g_body) * dt;
    x_nom_.q  = (x_nom_.q * Eigen::Quaternionf(R_delta)).normalized();
    // ba, bw, bv: constant (random walk, corrected in update)

    // ----------------------------------------------------------
    // 4. Error-state Jacobian Fx (18×18)
    //
    //    δx_{k+1} = Fx * δx_k + noise
    //
    //    Fx structure (block rows: δp, δv, δθ, δba, δbw, δbv):
    //      δp  row: I | R*dt | -R*[v]× dt |  0   |  0     | 0
    //      δv  row: 0 | R_Δ^T| -R_Δ^T[â]× dt| -R_Δ^T dt| 0  | 0
    //      δθ  row: 0 |  0   |  R_Δ^T     |  0   | -I*dt  | 0
    //      δba row: 0 |  0   |   0        |  I   |  0     | 0
    //      δbw row: 0 |  0   |   0        |  0   |  I     | 0
    //      δbv row: 0 |  0   |   0        |  0   |  0     | I
    // ----------------------------------------------------------
    Eigen::MatrixXf Fx = Eigen::MatrixXf::Identity(ERR_STATE_DIM, ERR_STATE_DIM);

    // δp row
    Fx.block<3,3>(P_IDX, V_IDX)   =  R_body * dt;
    Fx.block<3,3>(P_IDX, TH_IDX)  = -R_body * skew(x_nom_.v) * dt;

    // δv row  (v_{k+1} = R_Δ^T v_k + ...)
    Fx.block<3,3>(V_IDX, V_IDX)   =  R_delta.transpose();
    // ∂v/∂δθ: only g_body depends on attitude (a_hat = a_m - ba is measurement, no θ)
    //   δ(g_body) = [g_body]× · δθ   ⇒   Fx(V,TH) = R_Δ^T · [g_body]× · dt
    // NOTE: The previous formula used -[a_hat + g_body]× which ≈ 0 (gravity cancels
    //       specific force), destroying attitude–velocity coupling and observability.
    Fx.block<3,3>(V_IDX, TH_IDX)  =  R_delta.transpose() * skew(g_body) * dt;
    Fx.block<3,3>(V_IDX, BA_IDX)  = -Eigen::Matrix3f::Identity() * dt;

    // δθ row
    Fx.block<3,3>(TH_IDX, TH_IDX) =  R_delta.transpose();
    Fx.block<3,3>(TH_IDX, BW_IDX) = -Eigen::Matrix3f::Identity() * dt;

    // δba, δbw, δbv rows: already Identity on diagonal

    // ----------------------------------------------------------
    // 5. Process noise covariance Qc (18×18)
    //
    //    Qc = diag(0, σ_a² dt² I, σ_ω² dt² I, σ_ba² I, σ_bω² I, σ_bv² I)
    // ----------------------------------------------------------
    Eigen::MatrixXf Qc = Eigen::MatrixXf::Zero(ERR_STATE_DIM, ERR_STATE_DIM);

    // Discrete-time noise: Q = σ² * dt².
    // Derivation: velocity error from accel noise is δv = n_a * dt, so
    // Var(δv) = σ_a² * dt².  This is correct for per-sample discrete noise.
    // NOTE: σ_a here is the per-sample std (m/s²), not spectral density.
    // With σ_a=0.1, dt=0.001 → Q_v = 1e-8.  Steady-state Kalman gain ≈ 0.9%.
    // To get higher gain, increase σ_a (e.g. 1.0 → Q_v=1e-6, gain≈9%).
    Qc.block<3,3>(V_IDX, V_IDX)   = noise_.sigma_a.array().square().matrix().asDiagonal() * (dt * dt);
    Qc.block<3,3>(TH_IDX, TH_IDX) = noise_.sigma_w.array().square().matrix().asDiagonal() * (dt * dt);
    Qc.block<3,3>(BA_IDX, BA_IDX) = noise_.sigma_ba.array().square().matrix().asDiagonal();
    Qc.block<3,3>(BW_IDX, BW_IDX) = noise_.sigma_bw.array().square().matrix().asDiagonal();
    Qc.block<3,3>(BV_IDX, BV_IDX) = noise_.sigma_bv.array().square().matrix().asDiagonal();

    // ----------------------------------------------------------
    // 6. Covariance propagation: P = Fx * P * Fx^T + Qc
    // ----------------------------------------------------------
    P_ = Fx * P_ * Fx.transpose() + Qc;
}

// ============================================================
// Update — single leg (sequential EKF)
// ============================================================

void ESEKF::update_leg(LegObservation& obs, const Eigen::Vector3f& w_m,
                       LegUpdateDiag* diag) {
    if (!obs.in_contact) {
        if (diag) { *diag = LegUpdateDiag{}; diag->skipped = true; }
        return;
    }

    // ----------------------------------------------------------
    // 1. Compute observation z_leg from encoder kinematics
    //
    //    No-slip constraint: contact point velocity = 0 in world
    //      0 = v_body + ω×r_contact + ṙ_leg + ω_rim×r_rim
    //
    //    Set v=0, compute the remaining terms via PointVelocity,
    //    then z_leg = -contact_velocity(v=0) gives us the
    //    "observed body velocity" from encoder data alone.
    // ----------------------------------------------------------

    // Forward kinematics + contact point
    obs.leg->Calculate(obs.theta, obs.theta_d, 0, obs.beta, obs.beta_d, 0);
    obs.leg->PointContact(obs.rim, obs.alpha);

    // Use bias-corrected gyro for the ω×r term (pitch-only)
    // Only w_y (pitch rate) is used: the leg model is planar (XZ), so
    // roll/yaw gyro × large y-offset produces spurious velocity that
    // destabilises the filter.  Yaw observability requires external
    // heading reference (magnetometer, vision).
    Eigen::Vector3f w_corrected = w_m - x_nom_.bw;
    w_corrected.x() = 0.0f;
    w_corrected.z() = 0.0f;
    Eigen::Vector3f v_zero = Eigen::Vector3f::Zero();
    obs.leg->PointVelocity(v_zero, w_corrected, obs.rim, obs.alpha, true);

    // z_leg = -contact_velocity(v=0) = "observed" body velocity
    Eigen::Vector3f z_leg = -obs.leg->contact_velocity;

    // ----------------------------------------------------------
    // 2. Innovation (residual)
    //
    //    Observation model:  z_leg = v + noise
    //    NOTE: bv removed from observation to preserve observability.
    //    Include accumulated δx for correct sequential multi-leg update.
    // ----------------------------------------------------------
    Eigen::Vector3f v_pred = x_nom_.v + dx_.segment<3>(V_IDX);
    Eigen::Vector3f innovation = z_leg - v_pred;

    // ----------------------------------------------------------
    // 3. Observation Jacobian H (3×18)
    //
    //    Only w_y is used ⇒ only bw_y column of H_bw is non-zero.
    //    z_leg = -(w×r_c + ...), ∂z/∂δbw_y:
    //      δw_y = -δbw_y
    //      (δw × r_c)_x = δw_y · r_cz  →  δz_x = -(δw_y · r_cz) = +δbw_y · r_cz
    //      (δw × r_c)_z = -δw_y · r_cx →  δz_z = -(-δw_y · r_cx) = -δbw_y · r_cx
    //    ⇒  ∂z_x/∂δbw_y = +r_c.z()
    //    ⇒  ∂z_z/∂δbw_y = -r_c.x()
    // ----------------------------------------------------------
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, ERR_STATE_DIM);
    H.block<3,3>(0, V_IDX) = Eigen::Matrix3f::Identity();  // ∂h/∂δv
    // NOTE: bv deliberately excluded from H to preserve observability
    const Eigen::Vector3f& r_c = obs.leg->contact_point;
    H(0, BW_IDX + 1) =  r_c.z();   // ∂z_leg_x / ∂δbw_y = +r_c.z()
    H(2, BW_IDX + 1) = -r_c.x();   // ∂z_leg_z / ∂δbw_y = -r_c.x()

    // ----------------------------------------------------------
    // 4. Kalman gain & state/covariance update
    //
    //    S = H P H^T + R_leg        (3×3)
    //    K = P H^T S^{-1}           (18×3)
    //    δx += K * innovation
    //    P  = (I - KH) P (I - KH)^T + K R K^T   (Joseph form)
    // ----------------------------------------------------------
    // R_leg is per-axis measurement noise COVARIANCE (m²/s²).
    // sigma_leg_vec holds std devs [m/s]; R_leg = diag(sigma²).
    // Anisotropic: trust X/Z more (small σ), Y less (large σ).
    Eigen::Matrix3f R_leg = noise_.sigma_leg_vec.array().square().matrix().asDiagonal();

    Eigen::Matrix3f S = H * P_ * H.transpose() + R_leg;
    Eigen::Matrix3f S_inv = S.inverse();

    // Mahalanobis distance outlier rejection (Bloesch et al. 2013)
    // D² = yᵀ S⁻¹ y; reject if D² > threshold (χ²(3) @ 99.9% = 16.27)
    float d_squared = (innovation.transpose() * S_inv * innovation).value();

    // Populate diagnostics before potential early return
    if (diag) {
        diag->d_squared  = d_squared;
        diag->innovation = innovation;
        diag->S_diag     = S.diagonal();
        diag->rejected   = (d_squared > noise_.mahalanobis_threshold);
        diag->skipped    = false;
    }

    if (d_squared > noise_.mahalanobis_threshold) return;

    Eigen::MatrixXf K = P_ * H.transpose() * S_inv;  // 18×3

    dx_ += K * innovation;

    // Joseph-form covariance update (numerically stable)
    Eigen::MatrixXf I_KH = Eigen::MatrixXf::Identity(ERR_STATE_DIM, ERR_STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_leg * K.transpose();
}

// ============================================================
// Update — all legs
// ============================================================

void ESEKF::update_all_legs(std::vector<LegObservation>& obs,
                            const Eigen::Vector3f& w_m,
                            const std::array<bool, 4>& exclude) {
    for (int i = 0; i < 4; i++) {
        if (!exclude[i]) {
            update_leg(obs[i], w_m, &leg_diag_[i]);
        } else {
            leg_diag_[i] = LegUpdateDiag{};
            leg_diag_[i].skipped = true;
        }
    }
}

// ============================================================
// Inject error state into nominal + reset
// ============================================================

void ESEKF::inject_and_reset() {
    // ----------------------------------------------------------
    // 1. Inject error-state corrections into nominal state
    // ----------------------------------------------------------
    x_nom_.p  += dx_.segment<3>(P_IDX);
    x_nom_.v  += dx_.segment<3>(V_IDX);
    x_nom_.ba += dx_.segment<3>(BA_IDX);
    x_nom_.bw += dx_.segment<3>(BW_IDX);
    x_nom_.bv += dx_.segment<3>(BV_IDX);

    // Attitude: q_true = q_nom ⊗ [1, ½δθ]
    Eigen::Vector3f dtheta = dx_.segment<3>(TH_IDX);
    Eigen::Quaternionf dq;
    dq.w()   = 1.0f;
    dq.vec() = 0.5f * dtheta;
    x_nom_.q = (x_nom_.q * dq).normalized();

    // ----------------------------------------------------------
    // 2. Reset error state to zero
    // ----------------------------------------------------------
    dx_.setZero();

    // TODO: Covariance reset with G matrix (G ≈ I for small δθ)
    //       P = G * P * G^T  where G = I - [½δθ]×  (in θ block)
    //       Currently skipped since δθ is small after injection
}

} // namespace estimation_model
