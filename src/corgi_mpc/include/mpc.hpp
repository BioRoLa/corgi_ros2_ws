#ifndef MPC_HPP
#define MPC_HPP

#include <array>
#include <string>
#include <Eigen/Dense>

// Convex MPC on a single rigid body for the ABAD Corgi.
//
// State (13): [roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz, -g]
//   Everything is expressed in the world frame (z up); the last entry is the
//   constant gravity term carried along as a state.
// Input (12): ground reaction force on the robot at each foot, world frame,
//   [fx, fy, fz] for module A, B, C, D.
class ModelPredictiveController {
    public:
        static constexpr int n_x = 13;
        static constexpr int n_u = 12;

        // Timing
        int freq = 100;
        double dt = 1.0 / freq;
        int N = 10;  // horizon; N-1 inputs are optimised

        // Rigid body. Inertia is the body-frame diagonal about the CoM.
        double m = 23.05;
        double gravity = 9.81;
        Eigen::Vector3d inertia;
        // CoM position in the body frame of corgi_utils/robot_params.hpp
        Eigen::Vector3d com_offset;

        // Force limits per stance foot: |fx|, |fy| <= friction_coef * fz
        double friction_coef = 0.6;
        double fz_min = 0.0;
        double fz_max = 250.0;

        // Weights
        Eigen::Matrix<double, n_x, 1> Q_diagonal;
        double R_weight = 1e-8;

        // False when the last step() fell back to the previous solution.
        bool solved = false;

        ModelPredictiveController();

        // Reads config/config.yaml of this package unless a path is given.
        void load_config(const std::string &path = "");

        // Linearise about the current yaw. r[i] is the vector from the CoM to
        // foot i in the world frame.
        void update_model(double yaw, const std::array<Eigen::Vector3d, 4> &r);

        // x_ref stacks the N-1 reference states. Feet with contact[i] == false
        // get zero force over the whole horizon.
        Eigen::VectorXd step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref,
                             const std::array<bool, 4> &contact);

    private:
        Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(n_x, n_x);
        Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(n_x, n_u);
        Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(n_x, n_x);
        Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(n_x, n_u);
        Eigen::VectorXd last_u = Eigen::VectorXd::Zero(n_u);
};

#endif
