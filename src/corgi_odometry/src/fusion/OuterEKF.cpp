#include "fusion/OuterEKF.hpp"
#include <cmath>

namespace fusion {

// ----------------------------------------------------------------
// static helper
// ----------------------------------------------------------------
Eigen::Matrix3f OuterEKF::skew(const Eigen::Vector3f& v) {
    Eigen::Matrix3f S;
    S <<    0, -v(2),  v(1),
         v(2),     0, -v(0),
        -v(1),  v(0),     0;
    return S;
}

// ----------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------
OuterEKF::OuterEKF() {
    P_ = Eigen::MatrixXf::Identity(DIM, DIM);
    // Initial uncertainty: generous but bounded
    P_.block<3,3>(P_IDX,  P_IDX)  *= InitCovariance::p_mo_m2;   // p_mo ± 1 m
    P_.block<3,3>(TH_IDX, TH_IDX) *= InitCovariance::th_mo_r2;  // θ_mo ± 0.1 rad
    P_.block<3,3>(BV_IDX, BV_IDX) *= InitCovariance::bv_ms2;    // bv   ± 0.01 m/s
}

// ----------------------------------------------------------------
// init
// ----------------------------------------------------------------
void OuterEKF::init(const Eigen::Vector3f& p_mo,
                    const Eigen::Quaternionf& q_mo) {
    p_mo_        = p_mo;
    q_mo_        = q_mo.normalized();
    bv_          = Eigen::Vector3f::Zero();
    initialized_ = true;
}

// ----------------------------------------------------------------
// predict  — constant model for nominal + Fx coupling for bv→p_mo
// ----------------------------------------------------------------
void OuterEKF::predict(float dt) {
    if (!initialized_) return;

    // Fx (9×9): identity + bv→p_mo coupling
    // If bv_world is nonzero, inner-ESEKF p_odom drifts by bv_world*dt each step.
    // This makes p_mo_error also accumulate at that rate in map frame:
    //   δp_mo(k+1) = δp_mo(k) - R_mo * δbv * dt
    // This Fx builds up P[p_mo, bv] cross-covariance so that LiDAR updates
    // (H_bv = 0) can still observably correct bv.
    const Eigen::Matrix3f R_mo = q_mo_.toRotationMatrix();

    Eigen::MatrixXf Fx = Eigen::MatrixXf::Identity(DIM, DIM);
    Fx.block<3,3>(P_IDX, BV_IDX) = -R_mo * dt;

    // Process noise Q (added per-step)
    Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(DIM, DIM);
    Q.block<3,3>(P_IDX,  P_IDX)  = Eigen::Matrix3f::Identity() * (noise_.q_p  * dt);
    Q.block<3,3>(TH_IDX, TH_IDX) = Eigen::Matrix3f::Identity() * (noise_.q_th * dt);
    Q.block<3,3>(BV_IDX, BV_IDX) = Eigen::Matrix3f::Identity() * (noise_.q_bv * dt);

    P_ = Fx * P_ * Fx.transpose() + Q;
}

// ----------------------------------------------------------------
// update_lidar
// ----------------------------------------------------------------
void OuterEKF::update_lidar(const Eigen::Vector3f&    p_lidar,
                             const Eigen::Quaternionf& q_lidar,
                             const Eigen::Vector3f&    p_odom,
                             const Eigen::Quaternionf& q_odom,
                             float                     /*dt_lidar*/,
                             UpdateDiag*               diag) {
    if (!initialized_) return;

    const Eigen::Matrix3f R_mo = q_mo_.toRotationMatrix();

    // ── Position innovation ─────────────────────────────────────
    // z_p = p_lidar
    // h_p = p_mo + R_mo * p_odom
    Eigen::Vector3f h_p = p_mo_ + R_mo * p_odom;
    Eigen::Vector3f y_p = p_lidar - h_p;

    // H_p  (3×9)
    // ∂h_p/∂δp_mo  = I
    // ∂h_p/∂δθ_mo  = -R_mo * [p_odom]×     (from right-perturbation of R_mo)
    // ∂h_p/∂δbv    = 0  (bv observability comes from Fx cross-covariance, not H)
    Eigen::MatrixXf Hp(3, DIM);
    Hp.setZero();
    Hp.block<3,3>(0, P_IDX)  =  Eigen::Matrix3f::Identity();
    Hp.block<3,3>(0, TH_IDX) = -R_mo * skew(p_odom);

    // R_p (3×3) diagonal
    Eigen::Matrix3f Rp = Eigen::Matrix3f::Identity() * noise_.r_p;

    // ── Attitude innovation ─────────────────────────────────────
    // z_q = q_lidar
    // h_q = q_mo * q_odom
    Eigen::Quaternionf q_pred = (q_mo_ * q_odom).normalized();
    // Residual as rotation vector: δq = q_pred^{-1} * q_lidar
    Eigen::Quaternionf dq = (q_pred.inverse() * q_lidar).normalized();
    // Convert to rotation vector (small angle)
    Eigen::AngleAxisf aa(dq);
    Eigen::Vector3f y_th = aa.axis() * aa.angle();  // in q_pred frame

    // H_th (3×9)
    // ∂h_q/∂δθ_mo  = I (right-perturbation: q_mo * Exp(δθ) * q_odom)
    // ∂h_q/∂others = 0
    Eigen::MatrixXf Hth(3, DIM);
    Hth.setZero();
    Hth.block<3,3>(0, TH_IDX) = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f Rth = Eigen::Matrix3f::Identity() * noise_.r_th;

    // ── Sequential EKF update: position first, then attitude ───

    // --- Position update ---
    {
        Eigen::MatrixXf S = Hp * P_ * Hp.transpose() + Rp;
        Eigen::MatrixXf K = P_ * Hp.transpose() * S.inverse();
        Eigen::VectorXf dx = K * y_p;

        // Apply correction to nominal state
        p_mo_ += dx.segment<3>(P_IDX);

        // Attitude correction from δθ_mo
        Eigen::Vector3f dth_p = dx.segment<3>(TH_IDX);
        float ang = dth_p.norm();
        if (ang > 1e-8f) {
            Eigen::Quaternionf dq_p(Eigen::AngleAxisf(ang, dth_p / ang));
            q_mo_ = (q_mo_ * dq_p).normalized();
        }
        bv_ += dx.segment<3>(BV_IDX);

        // Covariance update (Joseph form for numerical stability)
        Eigen::MatrixXf IKH = Eigen::MatrixXf::Identity(DIM, DIM) - K * Hp;
        P_ = IKH * P_ * IKH.transpose() + K * Rp * K.transpose();

        if (diag) {
            diag->innov_p = y_p;
            diag->mahal_p = static_cast<float>(y_p.transpose() * S.inverse() * y_p);
        }
    }

    // Re-compute R_mo after position update (q_mo_ may have changed).
    // (Used implicitly via q_mo_ in attitude update block below.)

    // --- Attitude update ---
    {
        Eigen::MatrixXf S = Hth * P_ * Hth.transpose() + Rth;
        Eigen::MatrixXf K = P_ * Hth.transpose() * S.inverse();
        Eigen::VectorXf dx = K * y_th;

        p_mo_ += dx.segment<3>(P_IDX);

        Eigen::Vector3f dth_q = dx.segment<3>(TH_IDX);
        float ang = dth_q.norm();
        if (ang > 1e-8f) {
            Eigen::Quaternionf dq_q(Eigen::AngleAxisf(ang, dth_q / ang));
            q_mo_ = (q_mo_ * dq_q).normalized();
        }
        bv_ += dx.segment<3>(BV_IDX);

        Eigen::MatrixXf IKH = Eigen::MatrixXf::Identity(DIM, DIM) - K * Hth;
        P_ = IKH * P_ * IKH.transpose() + K * Rth * K.transpose();

        if (diag) {
            diag->innov_th = y_th;
            diag->mahal_th = static_cast<float>(y_th.transpose() * S.inverse() * y_th);
            diag->valid    = true;
        }
    }

    // Ensure q_mo_ stays normalised
    q_mo_.normalize();
}

// ----------------------------------------------------------------
// Accessors
// ----------------------------------------------------------------
Eigen::Vector3f OuterEKF::map_position(const Eigen::Vector3f& p_odom) const {
    return p_mo_ + q_mo_.toRotationMatrix() * p_odom;
}

Eigen::Quaternionf OuterEKF::map_orientation(const Eigen::Quaternionf& q_odom) const {
    return (q_mo_ * q_odom).normalized();
}

}  // namespace fusion
