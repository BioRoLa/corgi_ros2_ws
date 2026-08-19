#include "estimation/EstimationPipeline.hpp"
#include <cmath>

namespace corgi {

Leg EstimationPipeline::createLeg(double x_sign, double y_sign, int leg_idx) const {
    (void)leg_idx;  // reserved for per-leg calibration (eccentricity)
    // R = 0.1 (linkage joint circle) never changes; the radius switch acts
    // on little-r only. r_skin(): legacy 0.019 / design 0.045 / calibrated
    // rolling_radius_calibrated − 0.1 (stage15 fit: sim, roll state, kp 500).
    //
    // Camber on: leg y offset is the ABAD hinge axis (±hip_y_abad_axis,
    // 0.12) and the wheel-plane arm moves into Leg's d_wheel — the legacy
    // 0.193 and the 0.0917 arm are never summed. Camber off: legacy 0.193.
    const double y_off = params_.camber_enabled
        ? static_cast<double>(params_.hip_y_abad_axis)
        : Config::LEG_Y_OFFSET;
    Leg leg{
        Eigen::Vector3f(x_sign * Config::LEG_X_OFFSET,
                        y_sign * y_off,
                        Config::LEG_Z_OFFSET),
        static_cast<float>(Config::WHEEL_RADIUS),
        params_.r_skin()
    };
    if (params_.camber_enabled) {
        leg.set_camber_mode(true);
        leg.set_abad_geometry(params_.abad_axis_to_wheel_plane,
                              params_.wheel_half_width);
    }
    return leg;
}

EstimationPipeline::EstimationPipeline(const Params& params)
    : params_(params),
      lf_leg_(createLeg( 1,  1, 0)),
      rf_leg_(createLeg( 1, -1, 1)),
      rh_leg_(createLeg(-1, -1, 2)),
      lh_leg_(createLeg(-1,  1, 3)),
      observer_(Config::DT,
                params.observer_cutoff_freq,
                Config::DOF,
                params.enable_logging,
                params.csv_filename,
                params.log_details),
      esekf_()
{
    legs_[0] = &lf_leg_;
    legs_[1] = &rf_leg_;
    legs_[2] = &rh_leg_;
    legs_[3] = &lh_leg_;

    // Schmitt trigger thresholds
    for (auto& t : contact_triggers_) {
        t = ContactSchmittTrigger(
            params.contact_rm_threshold_high,
            params.contact_rm_threshold_low,
            params.contact_beta_threshold_high,
            params.contact_beta_threshold_low);
    }

    // ESEKF noise params
    estimation_model::NoiseParams np;
    np.sigma_a       = params.sigma_a;
    np.sigma_w       = params.sigma_w;
    np.sigma_ba      = params.sigma_ba;
    np.sigma_bw      = params.sigma_bw;
    np.sigma_leg_vec = params.sigma_leg_vec;
    np.mahalanobis_threshold = params.mahalanobis_threshold;
    esekf_.set_noise_params(np);
}

void EstimationPipeline::init(const Eigen::Quaternionf& q0) {
    estimation_model::NominalState x0;
    x0.q = q0.normalized();
    esekf_.init(x0);
    initialized_ = true;
}

void EstimationPipeline::override_processed_with_esekf(
        DataProcessor::ProcessedData& processed) const {
    const auto& est = esekf_.nominal();
    processed.q(0) = static_cast<double>(est.p.x());
    processed.q(1) = static_cast<double>(est.p.z());
    Eigen::Matrix3f R_est = est.q.toRotationMatrix();
    Eigen::Vector3f v_world = R_est * est.v;
    processed.q_dot(0) = static_cast<double>(v_world.x());
    processed.q_dot(1) = static_cast<double>(v_world.z());
}

StepResult EstimationPipeline::step(
        const DataProcessor::ProcessedData& processed,
        const Eigen::Vector3f& a_m,
        const Eigen::Vector3f& w_m,
        const RawRecord& raw,
        size_t index)
{
    StepResult result;

    // ── GMO disturbance estimation (always, every tick @ 1000 Hz) ──
    result.disturbance = observer_.estimate_disturbance(
        processed.q, processed.q_dot, processed.tau, processed.I_c,
        index, false);

    // ── Schmitt trigger contact detection (always) ──────────────
    // Disturbance layout: [4]=beta_a [5]=rm_a [6]=beta_b [7]=rm_b
    //                     [8]=beta_c [9]=rm_c [10]=beta_d [11]=rm_d
    constexpr int rm_idx[4]   = {5, 7, 9, 11};
    constexpr int beta_idx[4] = {4, 6, 8, 10};
    for (int j = 0; j < 4; ++j) {
        contact_triggers_[j].update(
            result.disturbance(rm_idx[j]),
            result.disturbance(beta_idx[j]));
        result.contacts[j] = contact_triggers_[j].state();
    }

    // ── ESEKF decimation (runs every ESEKF_DECIMATION ticks @ 500 Hz) ──
    esekf_tick_++;
    if (esekf_tick_ < static_cast<size_t>(Config::ESEKF_DECIMATION)) {
        // Non-ESEKF tick: buffer IMU for trapezoidal averaging
        prev_imu_a_ = a_m;
        prev_imu_w_ = w_m;
        prev_imu_valid_ = true;

        // Return current (stale) ESEKF state
        result.state = esekf_.nominal();
        result.P     = esekf_.covariance();
        result.esekf_updated = false;
        return result;
    }
    esekf_tick_ = 0;

    // ── Compute ESEKF dt from IMU timestamps ────────────────────
    float actual_dt = static_cast<float>(Config::ESEKF_DT);  // nominal 2ms
    if (params_.use_dynamic_dt) {
        if (prev_esekf_time_valid_ && (raw.imu_sec != 0 || raw.imu_nsec != 0)) {
            double cur_t  = raw.imu_sec  + raw.imu_nsec  * 1e-9;
            double prev_t = prev_esekf_imu_sec_ + prev_esekf_imu_nsec_ * 1e-9;
            double dt_sec = cur_t - prev_t;
            constexpr double lo = Config::ESEKF_DT * 0.5;
            constexpr double hi = Config::ESEKF_DT * 2.0;
            if (dt_sec > lo && dt_sec < hi)
                actual_dt = static_cast<float>(dt_sec);
        }
        if (raw.imu_sec != 0 || raw.imu_nsec != 0) {
            prev_esekf_imu_sec_  = raw.imu_sec;
            prev_esekf_imu_nsec_ = raw.imu_nsec;
            prev_esekf_time_valid_ = true;
        }
    }

    // ── Trapezoidal average of IMU for predict ──────────────────
    Eigen::Vector3f a_pred = prev_imu_valid_ ? 0.5f * (prev_imu_a_ + a_m) : a_m;
    Eigen::Vector3f w_pred = prev_imu_valid_ ? 0.5f * (prev_imu_w_ + w_m) : w_m;
    prev_imu_valid_ = false;  // consumed

    // ── ESEKF predict (averaged IMU, ESEKF-rate dt) ─────────────
    esekf_.predict(a_pred, w_pred, actual_dt);

    // Capture prediction-only velocity (before measurement update)
    result.pred_vel = esekf_.nominal().v;

    // ── Build per-leg observations ──────────────────────────────
    double thetas[4] = {raw.state_theta_a, raw.state_theta_b,
                        raw.state_theta_c, raw.state_theta_d};
    double betas[4]  = {raw.state_beta_a,  raw.state_beta_b,
                        raw.state_beta_c,  raw.state_beta_d};
    double vel_r[4]  = {raw.state_vel_r_a, raw.state_vel_r_b,
                        raw.state_vel_r_c, raw.state_vel_r_d};
    double vel_l[4]  = {raw.state_vel_l_a, raw.state_vel_l_b,
                        raw.state_vel_l_c, raw.state_vel_l_d};
    double gammas[4] = {raw.state_gamma_a, raw.state_gamma_b,
                        raw.state_gamma_c, raw.state_gamma_d};
    double vel_h[4]  = {raw.state_vel_h_a, raw.state_vel_h_b,
                        raw.state_vel_h_c, raw.state_vel_h_d};

    std::vector<estimation_model::LegObservation> observations;
    observations.reserve(4);
    std::array<bool, 4> exclude_flags{};

    for (int j = 0; j < 4; ++j) {
        bool is_right_side = (j == 1 || j == 2);

        float theta   = static_cast<float>(thetas[j]);
        float beta    = is_right_side
            ? -static_cast<float>(betas[j])
            :  static_cast<float>(betas[j]);
        float theta_d = static_cast<float>((-vel_r[j] + vel_l[j]) / 2.0);
        float beta_d  = static_cast<float>(( vel_r[j] + vel_l[j]) / 2.0);
        if (is_right_side) beta_d = -beta_d;

        // Pitch compensation for rim lookup
        float w = esekf_.nominal().q.w();
        float x = esekf_.nominal().q.x();
        float y = esekf_.nominal().q.y();
        float z = esekf_.nominal().q.z();
        float sinp = 2.0f * (w * y - z * x);
        float pitch = std::abs(sinp) >= 1.0f
            ? std::copysign(static_cast<float>(M_PI) / 2.0f, sinp)
            : std::asin(sinp);

        float compensated_beta = is_right_side ? (beta - pitch) : (beta + pitch);
        RIM rim = contact_map_.lookup(theta, compensated_beta);
        float alpha = 0.0f;

        bool in_contact = result.contacts[j] && (rim != NO_CONTACT);

        // Stage 1.5 camber: measured ABAD tilt gamma (sign-corrected per
        // leg; adjudicated all +1 in sim wheel mode) and its rate from the
        // hip motor velocity. Zeroed unless camber_enabled.
        float gamma = 0.0f, gamma_d = 0.0f;
        if (params_.camber_enabled) {
            gamma   = params_.gamma_signs[j] * static_cast<float>(gammas[j]);
            gamma_d = params_.gamma_signs[j] * static_cast<float>(vel_h[j]);
        }

        observations.push_back(
            {legs_[j], theta, theta_d, beta, beta_d, rim, alpha, in_contact,
             gamma, gamma_d});
        exclude_flags[j] = !in_contact;
    }

    // ── Compute average z_leg diagnostic ────────────────────────
    // Uses full w_m (consistent with actual ESEKF update that also uses full w_m).
    {
        Eigen::Vector3f v_zero = Eigen::Vector3f::Zero();
        int n_contact = 0;
        for (int j = 0; j < 4; ++j) {
            if (!exclude_flags[j]) {
                auto& o = observations[j];
                // 9-arg call — keeps the diagnostic recomputation
                // consistent with the actual ESEKF update (same gamma).
                o.leg->Calculate(o.theta, o.theta_d, 0, o.beta, o.beta_d, 0,
                                 o.gamma, o.gamma_d, 0);
                o.leg->PointContact(o.rim, o.alpha);
                o.leg->PointVelocity(v_zero, w_m, o.rim, o.alpha, true);
                result.z_avg += -o.leg->contact_velocity;
                n_contact++;
            }
        }
        if (n_contact > 0)
            result.z_avg /= static_cast<float>(n_contact);
    }

    // ── ESEKF update & inject ───────────────────────────────────
    esekf_.update_all_legs(observations, w_m, exclude_flags);
    esekf_.inject_and_reset();

    // Collect diagnostics
    result.esekf_updated = true;
    result.state = esekf_.nominal();
    result.diag  = esekf_.leg_diag();
    result.P     = esekf_.covariance();

    return result;
}

}  // namespace corgi
