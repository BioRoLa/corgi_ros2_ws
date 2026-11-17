#include "mpc.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <yaml-cpp/yaml.h>

#include "corgi_utils/robot_params.hpp"

ModelPredictiveController::ModelPredictiveController() {
    // Inertia of a uniform box with the chassis dimensions; the CoM is taken
    // at the height of the ABAD axes (LegWheel RobotParams.COM_BIAS_Z).
    const double l = corgi_robot::chassis_length;
    const double w = corgi_robot::chassis_width;
    const double h = corgi_robot::chassis_height;
    inertia << m * (w * w + h * h) / 12.0,
               m * (l * l + h * h) / 12.0,
               m * (l * l + w * w) / 12.0;
    com_offset << 0.0, 0.0, corgi_robot::abad_axis_height;

    Q_diagonal << 100, 100, 50,   // roll, pitch, yaw
                  0,   0,   50,   // x, y, z
                  0.1, 0.1, 0.1,  // angular velocity
                  1,   1,   2,    // linear velocity
                  0;              // gravity state
}

void ModelPredictiveController::load_config(const std::string &path) {
    std::string config_file_path = path;
    if (config_file_path.empty()) {
        const char *home_path = std::getenv("HOME");
        if (!home_path) {
            throw std::runtime_error("HOME environment variable not set");
        }
        config_file_path = std::string(home_path) + "/corgi_ws/corgi_ros2_ws/src/corgi_mpc/config/config.yaml";
    }
    YAML::Node config = YAML::LoadFile(config_file_path);

    std::vector<double> diag = config["Q_diagonal"].as<std::vector<double>>();
    if (static_cast<int>(diag.size()) != n_x) {
        throw std::runtime_error("Q_diagonal size mismatch with n_x");
    }
    for (int i = 0; i < n_x; ++i) Q_diagonal(i) = diag[i];

    auto read_vec3 = [&config](const char *key, Eigen::Vector3d &out) {
        if (!config[key]) return;
        std::vector<double> v = config[key].as<std::vector<double>>();
        if (v.size() != 3) throw std::runtime_error(std::string(key) + " needs three values");
        out << v[0], v[1], v[2];
    };
    if (config["R_weight"]) R_weight = config["R_weight"].as<double>();
    if (config["mass"]) m = config["mass"].as<double>();
    read_vec3("inertia_diagonal", inertia);
    read_vec3("com_offset", com_offset);
    if (config["friction_coef"]) friction_coef = config["friction_coef"].as<double>();
    if (config["fz_min"]) fz_min = config["fz_min"].as<double>();
    if (config["fz_max"]) fz_max = config["fz_max"].as<double>();
    if (config["horizon"]) N = config["horizon"].as<int>();
    if (N < 2) throw std::runtime_error("horizon must be at least 2");
}

void ModelPredictiveController::update_model(double yaw, const std::array<Eigen::Vector3d, 4> &r) {
    // Rotation about the vertical only: roll and pitch are assumed small.
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d I_world_inv = (Rz * inertia.asDiagonal() * Rz.transpose()).inverse();

    Ac.setZero();
    Ac.block<3, 3>(0, 6) = Rz.transpose();              // rpy rates from world angular velocity
    Ac.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity(); // position from velocity
    Ac(11, 12) = 1.0;                                   // gravity on vz

    Bc.setZero();
    for (int i = 0; i < 4; ++i) {
        Eigen::Matrix3d r_skew;
        r_skew <<        0, -r[i].z(),  r[i].y(),
                  r[i].z(),         0, -r[i].x(),
                 -r[i].y(),  r[i].x(),         0;
        Bc.block<3, 3>(6, 3 * i) = I_world_inv * r_skew;
        Bc.block<3, 3>(9, 3 * i) = Eigen::Matrix3d::Identity() / m;
    }

    // Zero-order hold discretisation
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_x + n_u, n_x + n_u);
    M.topLeftCorner(n_x, n_x) = Ac;
    M.topRightCorner(n_x, n_u) = Bc;
    Eigen::MatrixXd expM = (M * dt).exp();
    Ad = expM.topLeftCorner(n_x, n_x);
    Bd = expM.topRightCorner(n_x, n_u);
}

Eigen::VectorXd ModelPredictiveController::step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref,
                                                const std::array<bool, 4> &contact) {
    const int steps = N - 1;
    if (x.size() != n_x || x_ref.size() != steps * n_x) {
        throw std::runtime_error("MPC state or reference has the wrong size");
    }

    // Condensed prediction: X = A_qp * x + B_qp * U
    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(steps * n_x, n_x);
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(steps * n_x, steps * n_u);
    std::vector<Eigen::MatrixXd> A_pow(N);
    A_pow[0] = Eigen::MatrixXd::Identity(n_x, n_x);
    for (int k = 1; k < N; ++k) A_pow[k] = Ad * A_pow[k - 1];
    for (int i = 0; i < steps; ++i) {
        A_qp.block(i * n_x, 0, n_x, n_x) = A_pow[i + 1];
        for (int j = 0; j <= i; ++j) {
            B_qp.block(i * n_x, j * n_u, n_x, n_u) = A_pow[i - j] * Bd;
        }
    }

    Eigen::VectorXd Q_stack = Q_diagonal.replicate(steps, 1);
    Eigen::MatrixXd QB = Q_stack.asDiagonal() * B_qp;
    Eigen::MatrixXd H = 2.0 * (B_qp.transpose() * QB);
    H.diagonal().array() += 2.0 * R_weight + 1e-6;
    Eigen::VectorXd g = 2.0 * QB.transpose() * (A_qp * x - x_ref);

    // Per foot and step: vertical force bounds and a friction pyramid.
    // A swing foot has fz = 0, which the pyramid turns into fx = fy = 0.
    const int rows_per_foot = 5;
    const int total_constraints = steps * 4 * rows_per_foot;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(total_constraints * 2);
    Eigen::VectorXd lower_bound(total_constraints);
    Eigen::VectorXd upper_bound(total_constraints);
    const double inf = OsqpEigen::INFTY;

    int row = 0;
    for (int k = 0; k < steps; ++k) {
        for (int foot = 0; foot < 4; ++foot) {
            int idx = k * n_u + foot * 3;

            triplets.emplace_back(row, idx + 2, 1.0);
            lower_bound(row) = contact[foot] ? fz_min : 0.0;
            upper_bound(row) = contact[foot] ? fz_max : 0.0;
            ++row;

            for (int axis = 0; axis < 2; ++axis) {
                // f_axis - mu * fz <= 0
                triplets.emplace_back(row, idx + axis, 1.0);
                triplets.emplace_back(row, idx + 2, -friction_coef);
                lower_bound(row) = -inf;
                upper_bound(row) = 0.0;
                ++row;
                // f_axis + mu * fz >= 0
                triplets.emplace_back(row, idx + axis, 1.0);
                triplets.emplace_back(row, idx + 2, friction_coef);
                lower_bound(row) = 0.0;
                upper_bound(row) = inf;
                ++row;
            }
        }
    }
    Eigen::SparseMatrix<double> constraints(total_constraints, steps * n_u);
    constraints.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    H_sparse.makeCompressed();

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(steps * n_u);
    solver.data()->setNumberOfConstraints(total_constraints);
    solved = solver.data()->setHessianMatrix(H_sparse) &&
             solver.data()->setGradient(g) &&
             solver.data()->setLinearConstraintsMatrix(constraints) &&
             solver.data()->setLowerBound(lower_bound) &&
             solver.data()->setUpperBound(upper_bound) &&
             solver.initSolver() &&
             solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError &&
             solver.getStatus() == OsqpEigen::Status::Solved;

    // Keep the previous forces if the solver did not converge, without ever
    // pushing through a foot that has left the ground.
    if (solved) {
        last_u = solver.getSolution().head(n_u);
    }
    for (int foot = 0; foot < 4; ++foot) {
        if (!contact[foot]) last_u.segment<3>(3 * foot).setZero();
    }
    return last_u;
}
