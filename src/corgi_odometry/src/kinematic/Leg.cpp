/**
 * @file Leg.cpp
 * 
 * @author peichunhuang
 */
#include "kinematic/Leg.hpp"
#include <cmath>

// ── Stage 1.5 camber helpers (ABAD-axis frame) ──────────────────────
// Hand-ported shape from ABAD/state-estimation, geometry corrected per
// corgi_utils leg_model.cpp contact_map_3d: the lateral arm is the full
// ABAD-axis→wheel-plane distance plus the wheel edge offset (the archived
// kTireHalfThickness = 0.006125 constant was wrong and is NOT used).

float Leg::wheel_offset_by_gamma() const
{
    const float sin_g = std::sin(this->gamma);
    float edge_offset = 0.0f;
    if (std::abs(sin_g) >= 1e-4f) {
        edge_offset = (sin_g > 0.0f) ? -this->wheel_half_width
                                     :  this->wheel_half_width;
    }
    return this->d_axis_to_wheel_plane + edge_offset;
}

Eigen::Vector3f Leg::rotate_point_with_gamma(float x_2d, float z_2d) const
{
    // contact_map_3d rotation about the ABAD axis:
    //   Y = d_wheel·cosγ − z_2d·sinγ ;  Z = d_wheel·sinγ + z_2d·cosγ
    // computed in the leg-local frame (lateral + = outboard), then the
    // lateral component is mirrored onto the body y axis by the leg side.
    const float d_wheel = wheel_offset_by_gamma();
    const float sin_g = std::sin(this->gamma);
    const float cos_g = std::cos(this->gamma);
    const float sy = (this->offset(1) < 0.0f) ? -1.0f : 1.0f;
    return Eigen::Vector3f(
        x_2d,
        sy * (d_wheel * cos_g - z_2d * sin_g),
        d_wheel * sin_g + z_2d * cos_g);
}

Eigen::Vector3f Leg::rotate_velocity_with_gamma(float x_2d, float z_2d,
                                                float x_2d_d, float z_2d_d) const
{
    // Time derivative of rotate_point_with_gamma (d_wheel piecewise
    // constant in gamma — the edge-offset switch is not differentiated).
    (void)x_2d;
    const float d_wheel = wheel_offset_by_gamma();
    const float sin_g = std::sin(this->gamma);
    const float cos_g = std::cos(this->gamma);
    const float sy = (this->offset(1) < 0.0f) ? -1.0f : 1.0f;
    const float y_dot = -d_wheel * sin_g * this->gamma_d
                        - z_2d_d * sin_g
                        - z_2d * cos_g * this->gamma_d;
    const float z_dot = d_wheel * cos_g * this->gamma_d
                        + z_2d_d * cos_g
                        - z_2d * sin_g * this->gamma_d;
    return Eigen::Vector3f(x_2d_d, sy * y_dot, z_dot);
}

void Leg::Calculate(float theta, float theta_d, float theta_dd, float beta, float beta_d, float beta_dd)
{
    // Sagittal shim: gamma = gamma_d = gamma_dd = 0. Numerically identical
    // to the pre-camber code path (gamma is only read in the camber-mode
    // branches of PointContact/PointVelocity).
    Calculate(theta, theta_d, theta_dd, beta, beta_d, beta_dd, 0.0f, 0.0f, 0.0f);
}

void Leg::Calculate(float theta, float theta_d, float theta_dd,
                    float beta, float beta_d, float beta_dd,
                    float gamma, float gamma_d, float gamma_dd)
{
    this->beta = beta;
    this->beta_d = beta_d;
    this->beta_dd = beta_dd;
    this->gamma = gamma;
    this->gamma_d = gamma_d;
    this->gamma_dd = gamma_dd;
    std::complex<float> rot_ang =  std::polar(1.f, beta);
    std::complex<float> rot_vel =  beta_d * std::polar(1.f, beta + M_PI_2_F);
    this->calculate(theta, theta_d, theta_dd);
    this->O1 = rot_ang * this->O1;
    this->O1_ = rot_ang * this->O1_;
    this->O2 = rot_ang * this->O2;
    this->O2_ = rot_ang * this->O2_;
    this->O1_d = rot_ang * this->O1_d + rot_vel * this->O1;
    this->O1_d_ = rot_ang * this->O1_d_ + rot_vel * this->O1_;
    this->O2_d = rot_ang * this->O2_d + rot_vel * this->O2;
    this->O2_d_ = rot_ang * this->O2_d_ + rot_vel * this->O2_;
    this->G = rot_ang * this->G;
    this->F = rot_ang * this->F;
    this->F_ = rot_ang * this->F_;
    this->H = rot_ang * this->H;
    this->H_ = rot_ang * this->H_;
    this->G_d = rot_ang * this->G_d + rot_vel * this->G;
    this->O1_w += beta_d;
    this->O1_w_ += beta_d;
    this->O2_w += beta_d;
    this->O2_w_ += beta_d;
}

Eigen::Vector3f Leg::RimCentorVelocity(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim) {
    switch (rim) {
        case G_POINT:
        {
            return v + w.cross(this->offset + Eigen::Vector3f(this->G.imag(), 0, this->G.real())) + Eigen::Vector3f(this->G_d.imag(), 0, this->G_d.real());
        break;
        }
        case UPPER_RIM_R:
        {
            return  v + w.cross(this->offset + Eigen::Vector3f(this->O1.imag(), 0, this->O1.real())) + Eigen::Vector3f(this->O1_d.imag(), 0, this->O1_d.real());
        break;
        }
        case LOWER_RIM_R:
        {
            return v + w.cross(this->offset + Eigen::Vector3f(this->O2.imag(), 0, this->O2.real())) + Eigen::Vector3f(this->O2_d.imag(), 0, this->O2_d.real());
        break;
        }
        case LOWER_RIM_L:
        {
            return v + w.cross(this->offset + Eigen::Vector3f(this->O2_.imag(), 0, this->O2_.real())) + Eigen::Vector3f(this->O2_d_.imag(), 0, this->O2_d_.real());
        break;
        }
        case UPPER_RIM_L:
        {
            return v + w.cross(this->offset + Eigen::Vector3f(this->O1_.imag(), 0, this->O1_.real())) + Eigen::Vector3f(this->O1_d_.imag(), 0, this->O1_d_.real());
        break;
        }
        default:
        {
            return v + w.cross(this->offset);
        }
        break;
    }
}

Eigen::Vector3f Leg::RimCentorPosition(RIM rim) {
    switch (rim) {
        case G_POINT:
        {
            return this->offset + Eigen::Vector3f(this->G.imag(), 0, this->G.real());
        break;
        }
        case UPPER_RIM_R:
        {
            return  this->offset + Eigen::Vector3f(this->O1.imag(), 0, this->O1.real());
        break;
        }
        case LOWER_RIM_R:
        {
            return this->offset + Eigen::Vector3f(this->O2.imag(), 0, this->O2.real());
        break;
        }
        case LOWER_RIM_L:
        {
            return this->offset + Eigen::Vector3f(this->O2_.imag(), 0, this->O2_.real());
        break;
        }
        case UPPER_RIM_L:
        {
            return this->offset + Eigen::Vector3f(this->O1_.imag(), 0, this->O1_.real());
        break;
        }
        default:
        {
            return this->offset;
        }
        break;
    }
}

void Leg::PointContact(RIM rim, float alpha) {
    if (this->camber_mode) {
        PointContactCamber(rim, alpha);
        return;
    }
    float rim_radius = rim == G_POINT? this->r : this->r + this->R;
    std::complex<float> rim_p = std::polar((float) rim_radius, (float)(M_PI_F + alpha));
    switch (rim) {
        case G_POINT:
        {
            this->contact_point = this->offset + Eigen::Vector3f(this->G.imag() + rim_p.imag(), 0, this->G.real() + rim_p.real());
        break;
        }
        case UPPER_RIM_R:
        {
            this->contact_point = this->offset + Eigen::Vector3f(this->O1.imag() + rim_p.imag(), 0, this->O1.real() + rim_p.real());
        break;
        }
        case LOWER_RIM_R:
        {
            this->contact_point = this->offset + Eigen::Vector3f(this->O2.imag() + rim_p.imag(), 0, this->O2.real() + rim_p.real());
        break;
        }
        case LOWER_RIM_L:
        {
            this->contact_point = this->offset + Eigen::Vector3f(this->O2_.imag() + rim_p.imag(), 0, this->O2_.real() + rim_p.real());
        break;
        }
        case UPPER_RIM_L:
        {
            this->contact_point = this->offset + Eigen::Vector3f(this->O1_.imag() + rim_p.imag(), 0, this->O1_.real() + rim_p.real());
        break;
        }
        default:
        {
            this->contact_point = this->offset;
        }
        break;
    }
}

void Leg::PointVelocity(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim, float alpha, bool inbody_coord) {
    if (this->camber_mode) {
        PointVelocityCamber(v, w, rim, alpha, inbody_coord);
        return;
    }
    float rim_radius = rim == G_POINT? this->r : this->r + this->R;
    if (this->offset(1) < 0) link_w = O2_w_; // right side leg, left side lower rim
    else link_w = O2_w;
    rim_p = std::polar((float) rim_radius, (float)(M_PI_F + alpha));
    switch (rim) {
        case G_POINT:
        {
            this->contact_velocity = v + w.cross(this->contact_point) + Eigen::Vector3f(this->G_d.imag(), 0, this->G_d.real()) + Eigen::Vector3f(0, link_w, 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case UPPER_RIM_R:
        {
            link_w = O1_w;
            this->contact_velocity = v + w.cross(this->contact_point) + Eigen::Vector3f(this->O1_d.imag(), 0, this->O1_d.real()) + Eigen::Vector3f(0, link_w, 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case LOWER_RIM_R:
        {
            link_w = O2_w;
            this->contact_velocity = v + w.cross(this->contact_point) + Eigen::Vector3f(this->O2_d.imag(), 0, this->O2_d.real()) + Eigen::Vector3f(0, link_w, 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case LOWER_RIM_L:
        {
            link_w = O2_w_;
            this->contact_velocity = v + w.cross(this->contact_point) + Eigen::Vector3f(this->O2_d_.imag(), 0, this->O2_d_.real()) + Eigen::Vector3f(0, link_w, 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case UPPER_RIM_L:
        {
            link_w = O1_w_;
            this->contact_velocity = v + w.cross(this->contact_point) + Eigen::Vector3f(this->O1_d_.imag(), 0, this->O1_d_.real()) + Eigen::Vector3f(0, link_w, 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        default:
        {
            link_w = 0;
            this->contact_velocity = v + w.cross(this->contact_point);
        }
        break;
    }
    if (!inbody_coord) {
        std::complex<float> normal_vec = std::polar(1.f, alpha);
        std::complex<float> tangent_vec = std::polar(1.f, alpha + M_PI_2_F);
        this->contact_velocity = Eigen::Vector3f(this->contact_velocity.dot(Eigen::Vector3f(tangent_vec.imag(), 0, tangent_vec.real())), this->contact_velocity(1), this->contact_velocity.dot(Eigen::Vector3f(normal_vec.imag(), 0, normal_vec.real())));
    }
}

// ── Stage 1.5 camber-path implementations ───────────────────────────
// Same 2-D linkage solution as the legacy paths, but the contact point
// and velocity are rotated about the ABAD axis by gamma with the full
// d_wheel lateral arm. this->offset y is the ABAD hinge axis
// (±hip_y_abad_axis via createLeg) — never the legacy ±0.193.

void Leg::PointContactCamber(RIM rim, float alpha) {
    float rim_radius = rim == G_POINT? this->r : this->r + this->R;
    // k=1 eccentricity: r(β) = r_base + e·cos(β + φ), β continuous (as
    // passed to Calculate, no re-wrap). Config-gated, off by default.
    if (this->ecc_enabled)
        rim_radius += this->ecc_e * std::cos(this->beta + this->ecc_phi);
    std::complex<float> rim_pc = std::polar((float) rim_radius, (float)(M_PI_F + alpha));
    float x_2d = 0.0f;
    float z_2d = 0.0f;
    switch (rim) {
        case G_POINT:
        {
            x_2d = this->G.imag() + rim_pc.imag();
            z_2d = this->G.real() + rim_pc.real();
        break;
        }
        case UPPER_RIM_R:
        {
            x_2d = this->O1.imag() + rim_pc.imag();
            z_2d = this->O1.real() + rim_pc.real();
        break;
        }
        case LOWER_RIM_R:
        {
            x_2d = this->O2.imag() + rim_pc.imag();
            z_2d = this->O2.real() + rim_pc.real();
        break;
        }
        case LOWER_RIM_L:
        {
            x_2d = this->O2_.imag() + rim_pc.imag();
            z_2d = this->O2_.real() + rim_pc.real();
        break;
        }
        case UPPER_RIM_L:
        {
            x_2d = this->O1_.imag() + rim_pc.imag();
            z_2d = this->O1_.real() + rim_pc.real();
        break;
        }
        default:
        {
            this->contact_point = this->offset;
            return;
        }
        break;
    }
    this->contact_point = this->offset + rotate_point_with_gamma(x_2d, z_2d);
}

void Leg::PointVelocityCamber(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim,
                              float alpha, bool inbody_coord) {
    float rim_radius = rim == G_POINT? this->r : this->r + this->R;
    // k=1 eccentricity: same r(β) modulation as PointContactCamber so the
    // value/derivative pair stays consistent (radial rate added below).
    if (this->ecc_enabled)
        rim_radius += this->ecc_e * std::cos(this->beta + this->ecc_phi);
    if (this->offset(1) < 0) link_w = O2_w_; // right side leg, left side lower rim
    else link_w = O2_w;
    rim_p = std::polar((float) rim_radius, (float)(M_PI_F + alpha));
    float x_2d = 0.0f;
    float z_2d = 0.0f;
    float x_2d_d = 0.0f;
    float z_2d_d = 0.0f;
    switch (rim) {
        case G_POINT:
        {
            x_2d = this->G.imag() + rim_p.imag();
            z_2d = this->G.real() + rim_p.real();
            x_2d_d = this->G_d.imag() + link_w * rim_p.real();
            z_2d_d = this->G_d.real() - link_w * rim_p.imag();
        break;
        }
        case UPPER_RIM_R:
        {
            link_w = O1_w;
            x_2d = this->O1.imag() + rim_p.imag();
            z_2d = this->O1.real() + rim_p.real();
            x_2d_d = this->O1_d.imag() + link_w * rim_p.real();
            z_2d_d = this->O1_d.real() - link_w * rim_p.imag();
        break;
        }
        case LOWER_RIM_R:
        {
            link_w = O2_w;
            x_2d = this->O2.imag() + rim_p.imag();
            z_2d = this->O2.real() + rim_p.real();
            x_2d_d = this->O2_d.imag() + link_w * rim_p.real();
            z_2d_d = this->O2_d.real() - link_w * rim_p.imag();
        break;
        }
        case LOWER_RIM_L:
        {
            link_w = O2_w_;
            x_2d = this->O2_.imag() + rim_p.imag();
            z_2d = this->O2_.real() + rim_p.real();
            x_2d_d = this->O2_d_.imag() + link_w * rim_p.real();
            z_2d_d = this->O2_d_.real() - link_w * rim_p.imag();
        break;
        }
        case UPPER_RIM_L:
        {
            link_w = O1_w_;
            x_2d = this->O1_.imag() + rim_p.imag();
            z_2d = this->O1_.real() + rim_p.real();
            x_2d_d = this->O1_d_.imag() + link_w * rim_p.real();
            z_2d_d = this->O1_d_.real() - link_w * rim_p.imag();
        break;
        }
        default:
        {
            link_w = 0;
            this->contact_velocity = v + w.cross(this->contact_point);
            if (!inbody_coord) {
                std::complex<float> normal_vec = std::polar(1.f, alpha);
                std::complex<float> tangent_vec = std::polar(1.f, alpha + M_PI_2_F);
                this->contact_velocity = Eigen::Vector3f(this->contact_velocity.dot(Eigen::Vector3f(tangent_vec.imag(), 0, tangent_vec.real())), this->contact_velocity(1), this->contact_velocity.dot(Eigen::Vector3f(normal_vec.imag(), 0, normal_vec.real())));
            }
            return;
        }
        break;
    }
    // k=1 eccentricity radial rate: ṙ = −e·sin(β + φ)·β̇ along the radial
    // contact direction (matches the r(β) modulation above).
    if (this->ecc_enabled) {
        const float r_rate = -this->ecc_e
            * std::sin(this->beta + this->ecc_phi) * this->beta_d;
        const std::complex<float> radial_unit =
            std::polar(1.f, (float)(M_PI_F + alpha));
        x_2d_d += r_rate * radial_unit.imag();
        z_2d_d += r_rate * radial_unit.real();
    }
    this->contact_velocity = v + w.cross(this->contact_point)
        + rotate_velocity_with_gamma(x_2d, z_2d, x_2d_d, z_2d_d);
    if (!inbody_coord) {
        std::complex<float> normal_vec = std::polar(1.f, alpha);
        std::complex<float> tangent_vec = std::polar(1.f, alpha + M_PI_2_F);
        this->contact_velocity = Eigen::Vector3f(this->contact_velocity.dot(Eigen::Vector3f(tangent_vec.imag(), 0, tangent_vec.real())), this->contact_velocity(1), this->contact_velocity.dot(Eigen::Vector3f(normal_vec.imag(), 0, normal_vec.real())));
    }
}

float Leg::RimRoll(RIM rim) {
    if (this->offset(1) < 0) {
        link_w = O2_w_; // right side leg, left side lower rim
        link_w_d = O2_w_d_;
    }
    else {
        link_w = O2_w;
        link_w_d = O2_w_d_;
    }
    switch (rim) {
        case G_POINT:
        {
        break;
        }
        case UPPER_RIM_R:
        {
            link_w = O1_w;
            link_w_d = O1_w_d;
        break;
        }
        case LOWER_RIM_R:
        {
            link_w = O2_w;
            link_w_d = O2_w_d;
        break;
        }
        case LOWER_RIM_L:
        {
            link_w = O2_w_;
            link_w_d = O2_w_d_;
        break;
        }
        case UPPER_RIM_L:
        {
            link_w = O1_w_;
            link_w_d = O1_w_d_;
        break;
        }
        default:
        {
            link_w = 0;
            link_w_d = 0;
        }
        break;
    }
    return link_w;
}

Eigen::Vector3f Leg::RollVelocity(Eigen::Vector3f w, RIM rim, float alpha) {
    float rim_radius = rim == G_POINT? this->r : this->r + this->R;
    if (this->offset(1) < 0) {
        link_w = O2_w_; // right side leg, left side lower rim
        link_w_d = O2_w_d_;
    }
    else {
        link_w = O2_w;
        link_w_d = O2_w_d_;
    }
    rim_p = std::polar((float) rim_radius, (float)(M_PI_F + alpha));
    switch (rim) {
        case G_POINT:
        {
            return Eigen::Vector3f(0, link_w + w(1), 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case UPPER_RIM_R:
        {
            link_w = O1_w;
            link_w_d = O1_w_d;
            return Eigen::Vector3f(0, link_w + w(1), 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case LOWER_RIM_R:
        {
            link_w = O2_w;
            link_w_d = O2_w_d;
            return Eigen::Vector3f(0, link_w + w(1), 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case LOWER_RIM_L:
        {
            link_w = O2_w_;
            link_w_d = O2_w_d_;
            return Eigen::Vector3f(0, link_w + w(1), 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        case UPPER_RIM_L:
        {
            link_w = O1_w_;
            link_w_d = O1_w_d_;
            return Eigen::Vector3f(0, link_w + w(1), 0).cross(Eigen::Vector3f(rim_p.imag(), 0, rim_p.real()));
        break;
        }
        default:
        {
            link_w = 0;
            link_w_d = 0;
            return Eigen::Vector3f(0, 0, 0);
        }
        break;
    }
}

std::pair<float, float> Leg::Inverse(Eigen::Vector3f p, RIM rim=G_POINT) {
    p = p - this->offset;
    std::pair<float, float> motor_angles;
    float r = sqrt(p(0) * p(0) + p(2) * p(2));
    motor_angles.first = this->inverse(r, rim); // theta: 開合角
    motor_angles.second = atan2(-p(0), -p(2)); // beta: 腳的轉角
    return motor_angles;
}