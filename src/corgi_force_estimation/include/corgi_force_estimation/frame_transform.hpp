#ifndef CORGI_FORCE_ESTIMATION_FRAME_TRANSFORM_HPP
#define CORGI_FORCE_ESTIMATION_FRAME_TRANSFORM_HPP

#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <stdexcept>

namespace corgi_force {
struct LegFrame {
    Eigen::Matrix3d body_from_leg = Eigen::Matrix3d::Identity();
    bool sim = false;
    double theta_dir = 1.0, beta_dir = 1.0;
    double motor_l_dir = 1.0, motor_r_dir = 1.0, abad_dir = 1.0;
    double abad_axis = 1.0;

    Eigen::Matrix2d joint_directions() const {
        Eigen::Matrix2d result;
        result << (beta_dir + theta_dir) / 2.0, (beta_dir - theta_dir) / 2.0,
                  (beta_dir - theta_dir) / 2.0, (beta_dir + theta_dir) / 2.0;
        return result;
    }
    // Published joint coordinates -> actual mechanism coordinates used by LegModel.
    Eigen::Vector3d model_angles(double theta, double beta, double gamma) const {
        if (!sim) return {theta, beta, gamma};
        constexpr double theta0 = 17.0 * M_PI / 180.0;
        double l = motor_l_dir * (beta_dir * beta + theta_dir * theta - theta0);
        double r = motor_r_dir * (beta_dir * beta - theta_dir * theta + theta0);
        return {(l - r) / 2.0 + theta0, -(l + r) / 2.0, abad_axis * abad_dir * gamma};
    }
    Eigen::Vector3d motor_angles(const Eigen::Vector3d& model) const {
        if (!sim) return model;
        constexpr double theta0 = 17.0 * M_PI / 180.0;
        double l = motor_l_dir * (-model.y() + model.x() - theta0);
        double r = motor_r_dir * (-model.y() - model.x() + theta0);
        return {theta_dir * ((l - r) / 2.0 + theta0), beta_dir * (l + r) / 2.0,
                abad_axis * abad_dir * model.z()};
    }
    Eigen::Matrix3d model_motor_derivative() const {
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        if (!sim) return result;
        Eigen::Matrix2d reversed_beta;
        reversed_beta << 0.0, -1.0, -1.0, 0.0;
        result.topLeftCorner<2, 2>() = reversed_beta
            * Eigen::Vector2d(motor_l_dir, motor_r_dir).asDiagonal() * joint_directions();
        result(2, 2) = abad_axis * abad_dir;
        return result;
    }
};

inline LegFrame leg_frame(int module, bool sim) {
    if (module < 0 || module >= 4) throw std::out_of_range("Invalid module index");
    LegFrame frame;
    frame.sim = sim;
    // B/C wheel planes face the opposite direction to A/D.
    if (module == 1 || module == 2) {
        frame.body_from_leg(0, 0) = -1.0;
        frame.body_from_leg(1, 1) = -1.0;
    }
    // PROTO ABAD axes in body coordinates are [+X,+X,-X,-X].
    frame.abad_axis = (module == 1 || module == 3) ? -1.0 : 1.0;
    return frame;
}

inline bool update_directions(const std::vector<double>& data, std::array<LegFrame, 4>& frames) {
    // A/B/C/D, each [joint theta, joint beta, motor L, motor R, motor ABAD].
    if (data.size() != 20) return false;
    for (double v : data) if (v != 1.0 && v != -1.0) return false;
    for (int i = 0; i < 4; ++i) {
        frames[i].theta_dir = data[5 * i];
        frames[i].beta_dir = data[5 * i + 1];
        frames[i].motor_l_dir = data[5 * i + 2];
        frames[i].motor_r_dir = data[5 * i + 3];
        frames[i].abad_dir = data[5 * i + 4];
    }
    return true;
}

inline bool body_rotation(const Eigen::Quaterniond& imu, bool sim, Eigen::Matrix3d& world_from_body) {
    if (!imu.coeffs().allFinite() || imu.norm() < 1e-12) return false;
    world_from_body = imu.normalized().toRotationMatrix();
    if (!sim) {
        // Preserve the legacy hardware convention: roll unchanged, pitch/yaw reversed.
        Eigen::Matrix3d axes = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
        world_from_body = axes * world_from_body * axes;
    }
    return true;
}

inline Eigen::Matrix3d motor_jacobian(const Eigen::Matrix3d& model_jacobian, const LegFrame& frame) {
    return model_jacobian * frame.model_motor_derivative();
}
} // namespace corgi_force
#endif
