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

ESEKF::ESEKF(float dt) : dt_(dt) {
    dx_ = Eigen::VectorXf::Zero(ERR_STATE_DIM);

    // Initial error covariance: small for biases, larger for position/velocity
    P_ = Eigen::MatrixXf::Identity(ERR_STATE_DIM, ERR_STATE_DIM) * 1e-4f;
    P_.block<3,3>(P_IDX, P_IDX)  *= 100.0f;   // position uncertainty
    P_.block<3,3>(V_IDX, V_IDX)  *= 100.0f;   // velocity uncertainty
}

void ESEKF::init(const NominalState& x0) {
    x_nom_ = x0;
    dx_.setZero();
}

// ============================================================
// Predict (IMU propagation)
// ============================================================

void ESEKF::predict(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m) {
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
    Eigen::Vector3f dw = w_hat * dt_;
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
    x_nom_.p += R_body * x_nom_.v * dt_;
    x_nom_.v  = R_delta.transpose() * x_nom_.v + (a_hat + g_body) * dt_;
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
    Fx.block<3,3>(P_IDX, V_IDX)   =  R_body * dt_;
    Fx.block<3,3>(P_IDX, TH_IDX)  = -R_body * skew(x_nom_.v) * dt_;

    // δv row  (v_{k+1} = R_Δ^T v_k + ...)
    Fx.block<3,3>(V_IDX, V_IDX)   =  R_delta.transpose();
    Eigen::Vector3f a_total = a_hat + g_body;
    Fx.block<3,3>(V_IDX, TH_IDX)  = -R_delta.transpose() * skew(a_total) * dt_;
    Fx.block<3,3>(V_IDX, BA_IDX)  = -R_delta.transpose() * dt_;

    // δθ row
    Fx.block<3,3>(TH_IDX, TH_IDX) =  R_delta.transpose();
    Fx.block<3,3>(TH_IDX, BW_IDX) = -Eigen::Matrix3f::Identity() * dt_;

    // δba, δbw, δbv rows: already Identity on diagonal

    // ----------------------------------------------------------
    // 5. Process noise covariance Qc (18×18)
    //
    //    Qc = diag(0, σ_a² dt² I, σ_ω² dt² I, σ_ba² I, σ_bω² I, σ_bv² I)
    // ----------------------------------------------------------
    Eigen::MatrixXf Qc = Eigen::MatrixXf::Zero(ERR_STATE_DIM, ERR_STATE_DIM);

    // TODO: Verify noise scaling — current: element-wise σ² * dt²
    //       Alternatively use σ² * dt for continuous-time discretization
    Qc.block<3,3>(V_IDX, V_IDX)   = noise_.sigma_a.array().square().matrix().asDiagonal() * (dt_ * dt_);
    Qc.block<3,3>(TH_IDX, TH_IDX) = noise_.sigma_w.array().square().matrix().asDiagonal() * (dt_ * dt_);
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

void ESEKF::update_leg(LegObservation& obs, const Eigen::Vector3f& w_m) {
    if (!obs.in_contact) return;

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

    // Use bias-corrected gyro for the ω×r term
    // Zero w_x/w_z: robot moves in XZ-plane, only pitch rate (w_y) is
    // meaningful. Noise in roll/yaw gyro × large y-offset creates
    // spurious velocity in the observation.
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
    //    Observation model:  z_leg = v + bv + noise
    //    Predicted:          z_hat = x_nom.v + x_nom.bv
    //    Innovation:         y = z_leg - z_hat
    // ----------------------------------------------------------
    Eigen::Vector3f z_predicted = x_nom_.v + x_nom_.bv;
    Eigen::Vector3f innovation = z_leg - z_predicted;

    // ----------------------------------------------------------
    // 3. Observation Jacobian H (3×18)
    //
    //    z_leg depends on ω_used = (0, w_m_y - bw_y, 0) through
    //    the ω×r_c term.  Since w_x and w_z are zeroed, only bw_y
    //    has a non-zero partial derivative:
    //
    //      ∂z_leg/∂δbw = -skew(r_c) · diag(0,-1,0)
    //
    //    which yields a matrix with only the bw_y column non-zero:
    //      H_bw = [[0, -r_cz, 0],
    //              [0,   0,   0],
    //              [0,  r_cx, 0]]
    //
    //    h(δx) = δv + δbv + H_bw · δbw + noise
    // ----------------------------------------------------------
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, ERR_STATE_DIM);
    H.block<3,3>(0, V_IDX)  = Eigen::Matrix3f::Identity();  // ∂h/∂δv
    H.block<3,3>(0, BV_IDX) = Eigen::Matrix3f::Identity();  // ∂h/∂δbv
    // ∂h/∂δbw: only bw_y column (index 1) is non-zero
    //   since w_x and w_z are zeroed before computing z_leg
    const Eigen::Vector3f& r_c = obs.leg->contact_point;
    H(0, BW_IDX + 1) = -r_c.z();   // ∂z_leg_x / ∂δbw_y
    H(2, BW_IDX + 1) =  r_c.x();   // ∂z_leg_z / ∂δbw_y

    // ----------------------------------------------------------
    // 4. Kalman gain & state/covariance update
    //
    //    S = H P H^T + R_leg
    //    K = P H^T S^{-1}           (18×3)
    //    δx += K * innovation
    //    P  = (I - KH) P (I - KH)^T + K R_leg K^T   (Joseph form)
    // ----------------------------------------------------------

    // TODO: Consider per-leg or state-dependent measurement noise
    Eigen::Matrix3f R_leg = Eigen::Matrix3f::Identity() * noise_.sigma_leg;

    Eigen::Matrix3f S = H * P_ * H.transpose() + R_leg;
    Eigen::MatrixXf K = P_ * H.transpose() * S.inverse();  // 18×3

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
            update_leg(obs[i], w_m);
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
