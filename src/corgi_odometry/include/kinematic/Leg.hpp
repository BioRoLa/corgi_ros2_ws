#ifndef LEG_HPP
#define LEG_HPP
#include <Eigen/Dense>
#include "LinkLegModel.hpp"

class Leg : public LinkLegModel
{
    public:
    Leg(Eigen::Vector3f offset, float R = 0.1, float r = 0.019)
    : LinkLegModel(r, R)
    {
        this->offset = offset;
    }
    void Calculate(float theta, float theta_d, float theta_dd, float beta, float beta_d, float beta_dd);
    // Stage 1.5 camber overload: additionally stores the measured ABAD tilt
    // gamma [rad] and its rate. Gamma only affects PointContact/PointVelocity
    // when camber_mode is enabled (set_camber_mode); otherwise the legacy
    // sagittal path is taken bit-identically.
    void Calculate(float theta, float theta_d, float theta_dd,
                   float beta, float beta_d, float beta_dd,
                   float gamma, float gamma_d, float gamma_dd);
    Eigen::Vector3f RimCentorVelocity(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim);
    Eigen::Vector3f RimCentorPosition(RIM rim);
    void PointContact(RIM rim, float alpha=0);
    void PointVelocity(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim, float alpha=0, bool inbody_coord=true);
    float RimRoll(RIM rim);
    Eigen::Vector3f RollVelocity(Eigen::Vector3f w, RIM rim, float alpha=0);
    float beta = 0;
    float beta_d = 0;
    float beta_dd = 0;
    float gamma = 0;
    float gamma_d = 0;
    float gamma_dd = 0;
    Eigen::Vector3f offset;
    Eigen::Vector3f contact_point;
    Eigen::Vector3f contact_velocity;
    float Radius() {return this->R;}
    float radius() {return this->r;}
    std::pair<float, float> Inverse(Eigen::Vector3f p, RIM rim);
    float link_w = 0;
    float link_w_d = 0;
    std::complex<float> rim_p;

    // ── Stage 1.5 camber correction (ABAD-axis frame) ────────────────
    // When camber_mode is on, the leg offset y is the ABAD hinge axis
    // (±hip_y_abad_axis) and the contact point/velocity are rotated about
    // that axis by gamma with lateral arm
    //   d_wheel = d_axis_to_wheel_plane + edge_offset(gamma),
    //   edge_offset = -wheel_half_width if sin(gamma) > 0
    //                 +wheel_half_width if sin(gamma) < 0
    //                 0                if |sin(gamma)| < 1e-4
    // (matches corgi_utils leg_model.cpp contact_map_3d). The legacy
    // sagittal path (camber_mode off) is untouched and bit-identical.
    void set_camber_mode(bool on) { camber_mode = on; }
    void set_abad_geometry(float d_axis_to_wheel_plane_, float wheel_half_width_) {
        d_axis_to_wheel_plane = d_axis_to_wheel_plane_;
        wheel_half_width = wheel_half_width_;
    }
    bool camber_mode = false;
    float d_axis_to_wheel_plane = 0.091675f;  // ABAD axis → wheel mid-plane [m]
    float wheel_half_width = 0.02f;           // edge offset magnitude [m]

    private:
    /// d_wheel: lateral arm from ABAD axis to the contact edge [m].
    float wheel_offset_by_gamma() const;
    /// Rotate a sagittal-plane point (x_2d, z_2d) about the ABAD axis by
    /// gamma, adding the lateral d_wheel arm. Returns a body-frame vector
    /// relative to the ABAD-axis offset; the lateral component is mirrored
    /// by the sign of offset(1) (left/right leg).
    Eigen::Vector3f rotate_point_with_gamma(float x_2d, float z_2d) const;
    /// Matching velocity: value/derivative-consistent with
    /// rotate_point_with_gamma (d_wheel treated as piecewise constant).
    Eigen::Vector3f rotate_velocity_with_gamma(float x_2d, float z_2d,
                                               float x_2d_d, float z_2d_d) const;
    /// Camber-path implementations (legacy paths remain untouched above).
    void PointContactCamber(RIM rim, float alpha);
    void PointVelocityCamber(Eigen::Vector3f v, Eigen::Vector3f w, RIM rim,
                             float alpha, bool inbody_coord);
};
#endif
