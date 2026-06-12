#include "vmc.hpp"
#include <stdexcept>

// Global variables for walk_*.cpp files
bool sim = true;
LegModel legmodel;

void VirtualModelController::load_config(const std::string& profile) {
    const char* home_path = std::getenv("HOME");
    if (!home_path) {
        throw std::runtime_error("HOME environment variable not set");
    }
    std::string config_file_path = std::string(home_path) + "/corgi_ws/corgi_ros2_ws/src/corgi_vmc/config/config.yaml";
    YAML::Node config = YAML::LoadFile(config_file_path);
    YAML::Node common_config = config["common"];
    YAML::Node profile_config = config[profile];

    if (!profile_config) {
        throw std::runtime_error("Missing required profile: " + profile);
    }

    auto read_required_node = [&](const std::string& key) -> YAML::Node {
        if (profile_config[key]) {
            return profile_config[key];
        }
        if (common_config && common_config[key]) {
            return common_config[key];
        }
        throw std::runtime_error("Missing required key: " + key + " for profile: " + profile);
    };

    auto read_required_double = [&](const std::string& key) -> double {
        return read_required_node(key).as<double>();
    };

    std::vector<double> kp_vec = read_required_node("Kp").as<std::vector<double>>();
    std::vector<double> kd_vec = read_required_node("Kd").as<std::vector<double>>();
    if (kp_vec.size() != 6 || kd_vec.size() != 6) {
        throw std::runtime_error("Kp and Kd must each have 6 elements");
    }
    Kp = Eigen::Map<Eigen::VectorXd>(kp_vec.data(), 6);
    Kd = Eigen::Map<Eigen::VectorXd>(kd_vec.data(), 6);

    Mx = read_required_double("Mx");
    My = read_required_double("My");
    Bx_swing = read_required_double("Bx_swing");
    By_swing = read_required_double("By_swing");
    Bx_stance = read_required_double("Bx_stance");
    By_stance = read_required_double("By_stance");
    Kx_swing = read_required_double("Kx_swing");
    Ky_swing = read_required_double("Ky_swing");
    Kx_stance = read_required_double("Kx_stance");
    Ky_stance = read_required_double("Ky_stance");
    m = read_required_double("m");
    fx_upper_bound = read_required_double("fx_upper_bound");
    fx_lower_bound = read_required_double("fx_lower_bound");
    fz_upper_bound = read_required_double("fz_upper_bound");
    fz_lower_bound = read_required_double("fz_lower_bound");
    force_weight = read_required_double("force_weight");

    std::vector<double> ww = read_required_node("wrench_weight").as<std::vector<double>>();
    if (ww.size() != 6) {
        throw std::runtime_error("wrench_weight must have 6 elements");
    }
    wrench_weight = Eigen::Map<Eigen::VectorXd>(ww.data(), 6);

    if (m <= 0.0) {
        throw std::runtime_error("Invalid value: m must be > 0");
    }
    if (fx_lower_bound > fx_upper_bound) {
        throw std::runtime_error("Invalid bounds: fx_lower_bound > fx_upper_bound");
    }
    if (fz_lower_bound > fz_upper_bound) {
        throw std::runtime_error("Invalid bounds: fz_lower_bound > fz_upper_bound");
    }
}

Eigen::VectorXd VirtualModelController::compute_wrench(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref) {
    // x layout:     [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz, g]
    // x_ref layout: [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz, g]
    // wrench:       [tau_roll, tau_pitch, tau_yaw, Fx, Fy, Fz]

    Eigen::VectorXd F_virtual(6);

    // Position/orientation error: x_ref[0..5] - x[0..5]
    Eigen::VectorXd pos_error(6);
    pos_error << x_ref(0) - x(0),   // roll
                 x_ref(1) - x(1),   // pitch
                 x_ref(2) - x(2),   // yaw
                 x_ref(3) - x(3),   // x
                 x_ref(4) - x(4),   // y
                 x_ref(5) - x(5);   // z

    // Velocity error: x_ref[6..11] - x[6..11]
    Eigen::VectorXd vel_error(6);
    vel_error << x_ref(6) - x(6),   // omega_x
                 x_ref(7) - x(7),   // omega_y
                 x_ref(8) - x(8),   // omega_z
                 x_ref(9) - x(9),   // vx
                 x_ref(10) - x(10), // vy
                 x_ref(11) - x(11); // vz

    // PD control + gravity compensation
    F_virtual = Kp.asDiagonal() * pos_error + Kd.asDiagonal() * vel_error;

    // Gravity compensation on Fz (index 5)
    F_virtual(5) += m * gravity;

    return F_virtual;
}

Eigen::MatrixXd VirtualModelController::build_contact_matrix(const double *ra, const double *rb, const double *rc, const double *rd) {
    // Build 6x12 matrix mapping leg forces [fA(3), fB(3), fC(3), fD(3)] -> body wrench [tau(3), F(3)]
    // Rows 0-2: torques tau = sum(r_i x f_i)  [Nm]
    // Rows 3-5: forces  F   = sum(f_i)         [N]
    // NOTE: no division by inertia or mass — F_virtual is in wrench units (Nm, N), not accelerations.

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 12);
    std::array<const double*, 4> rs = {ra, rb, rc, rd};

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d ri(rs[i][0], rs[i][1], rs[i][2]);
        Eigen::Matrix3d r_skew;
        r_skew <<      0, -ri.z(),  ri.y(),
                  ri.z(),       0, -ri.x(),
                 -ri.y(),  ri.x(),       0;

        // Torque rows: r x f  (cross product)
        A.block<3, 3>(0, 3 * i) = r_skew;
        // Force rows: direct sum
        A.block<3, 3>(3, 3 * i) = Eigen::Matrix3d::Identity();
    }

    return A;
}

Eigen::VectorXd VirtualModelController::distribute_forces(const Eigen::VectorXd &F_virtual, const Eigen::MatrixXd &A, const bool *selection_matrix) {
    // Soft-objective QP (standard VMC force distribution):
    //   min  0.5 * f^T (A^T W1 A + W2) f  -  f^T A^T W1 F_virtual
    //   s.t. force bounds per leg
    //        swing legs zeroed
    //        fy = 0 per leg
    //
    // No hard equality — avoids infeasibility when constraints conflict.
    // W1 = diag(wrench_weight): weights each wrench component (Nm, N)
    // W2 = force_weight * I:    regularizes force magnitude

    const int n_vars = n_u;  // 12

    // Per foot: fx bounds + fy=0 + fz bounds + swing(3) = 7 rows
    const int total_constraints = 4 * 7;

    // Hessian and gradient from soft wrench tracking
    Eigen::MatrixXd W1 = wrench_weight.asDiagonal();
    Eigen::MatrixXd H = A.transpose() * W1 * A
                      + force_weight * Eigen::MatrixXd::Identity(n_vars, n_vars);
    H += 1e-8 * Eigen::MatrixXd::Identity(n_vars, n_vars);
    Eigen::VectorXd g = -A.transpose() * W1 * F_virtual;

    // Constraint matrix (only inequality)
    Eigen::SparseMatrix<double> constraints(total_constraints, n_vars);
    Eigen::VectorXd lower_bound(total_constraints);
    Eigen::VectorXd upper_bound(total_constraints);

    int row = 0;
    for (int foot = 0; foot < 4; ++foot) {
        int base_idx = foot * 3;

        // fx bounds
        constraints.insert(row, base_idx) = 1.0;
        lower_bound(row) = fx_lower_bound;
        upper_bound(row) = fx_upper_bound;
        ++row;

        // fy = 0
        constraints.insert(row, base_idx + 1) = 1.0;
        lower_bound(row) = -1e-6;
        upper_bound(row) = 1e-6;
        ++row;

        // fz bounds
        constraints.insert(row, base_idx + 2) = 1.0;
        lower_bound(row) = fz_lower_bound;
        upper_bound(row) = fz_upper_bound;
        ++row;

        // Swing selection: zero all forces when foot is not in stance
        double s = selection_matrix[foot] ? 0.0 : 1.0;
        for (int i = 0; i < 3; ++i) {
            constraints.insert(row, base_idx + i) = s;
            lower_bound(row) = 0.0;
            upper_bound(row) = 0.0;
            ++row;
        }

        // Explicit fy = 0
        constraints.insert(row, base_idx + 1) = 1.0;
        lower_bound(row) = 0.0;
        upper_bound(row) = 0.0;
        ++row;
    }
    constraints.makeCompressed();

    // OSQP Solver
    OsqpEigen::Solver solver;
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(total_constraints);

    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    H_sparse.makeCompressed();
    solver.data()->setHessianMatrix(H_sparse);

    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(constraints);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    solver.settings()->setVerbosity(false);
    solver.settings()->setAlpha(1.0);

    if (!solver.initSolver()) throw std::runtime_error("[VMC-OSQP] Initialization failed.");
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        throw std::runtime_error("[VMC-OSQP] Solver failed to converge.");

    return solver.getSolution();
}

Eigen::VectorXd VirtualModelController::step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref,
                                              const bool *selection_matrix,
                                              const double *ra, const double *rb, const double *rc, const double *rd) {
    Eigen::VectorXd F_virtual = compute_wrench(x, x_ref);
    Eigen::MatrixXd A = build_contact_matrix(ra, rb, rc, rd);
    return distribute_forces(F_virtual, A, selection_matrix);
}
