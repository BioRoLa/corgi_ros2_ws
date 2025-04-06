#include "mpc.hpp"
#include <stdexcept>

void ModelPredictiveController::init_matrices(const double *ra, const double *rb, const double *rc, const double *rd) {
    A << 1, 0, 0, dt, 0,  0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, dt,  0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0,  dt,0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,  1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0,  0, 1, 0, 0, dt,0, 0,
         0, 0, 0, 0, 0,  0, 0, 1, 0, 0, dt,0,
         0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0, dt,
         0, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1;

    B << 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         dt/m, 0, 0, dt/m, 0, 0, dt/m, 0, 0, dt/m, 0, 0,
         0, dt/m, 0, 0, dt/m, 0, 0, dt/m, 0, 0, dt/m, 0,
         0, 0, dt/m, 0, 0, dt/m, 0, 0, dt/m, 0, 0, dt/m,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, -60/m*ra[2]*dt, 60/m*ra[1]*dt, 0, -60/m*rb[2]*dt, 60/m*rb[1]*dt, 0, -60/m*rc[2]*dt, 60/m*rc[1]*dt, 0, -60/m*rd[2]*dt, 60/m*rd[1]*dt,
         30/m*ra[2]*dt, 0, -30/m*ra[0]*dt, 30/m*rb[2]*dt, 0, -30/m*rb[0]*dt, 30/m*rc[2]*dt, 0, -30/m*rc[0]*dt, 30/m*rd[2]*dt, 0, -30/m*rd[0]*dt,
         -300/13/m*ra[1]*dt, 300/13/m*ra[0]*dt, 0, -300/13/m*rb[1]*dt, 300/13/m*rb[0]*dt, 0, -300/13/m*rc[1]*dt, 300/13/m*rc[0]*dt, 0, -300/13/m*rd[1]*dt, 300/13/m*rd[0]*dt, 0;

    Q = Eigen::MatrixXd::Zero(n_x, n_x);
    Q.diagonal() << 5e6  , 0   , 5e7 ,   // x, y, z positions
                    1e6  , 0   , 1e4 ,   // x, y, z velocities
                    1e7  , 1e7 , 0   ,   // roll, pitch, yaw
                    1e5  , 1e5 , 0   ;   // angular velocities

    R = Eigen::MatrixXd::Identity(n_u, n_u);
}


Eigen::VectorXd ModelPredictiveController::step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref,
                                                const bool *selection_matrix, std::vector<corgi_msgs::ForceState*> force_state_modules) {
    // System dynamics formulation.
    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(N * n_x, n_x);
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(N * n_x, (N - 1) * n_u);

    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd A_pow = Eigen::MatrixXd::Identity(n_x, n_x);
        for (int k = 0; k < (i + 1); ++k){
            A_pow *= A;
        }
        A_qp.block(i * n_x, 0, n_x, n_x) = A_pow;

        int max_j = (i < (N - 1)) ? (i + 1) : (N - 1);
        for (int j = 0; j < max_j; ++j) {
            Eigen::MatrixXd A_temp = Eigen::MatrixXd::Identity(n_x, n_x);
            for (int k = 0; k < (i - j); ++k){
                A_temp *= A;
            }
            B_qp.block(i * n_x, j * n_u, n_x, n_u) = A_temp * B;
        }
    }

    // Gravity compensation.
    Eigen::VectorXd d = Eigen::VectorXd::Zero(n_x);
    d(2) = -0.5 * dt * dt * gravity;
    d(5) = -dt * gravity;

    Eigen::VectorXd d_qp = Eigen::VectorXd::Zero(N * n_x);
    for (int i = 0; i < N; i++) {
        d_qp.segment(i * n_x, n_x) = d;
    }

    // Construct the block-diagonal cost matrices.
    Eigen::MatrixXd Q_N = Eigen::MatrixXd::Zero(N * n_x, N * n_x);
    for (int i = 0; i < N; ++i){
        Q_N.block(i * n_x, i * n_x, n_x, n_x) = Q;
    }

    Eigen::MatrixXd R_N = Eigen::MatrixXd::Zero((N - 1) * n_u, (N - 1) * n_u);
    for (int i = 0; i < (N - 1); ++i){
        R_N.block(i * n_u, i * n_u, n_u, n_u) = R;
    }

    // QP formulation.
    Eigen::MatrixXd H = 2 * (B_qp.transpose() * Q_N * B_qp + R_N);
    Eigen::VectorXd g = 2 * B_qp.transpose() * Q_N * (A_qp * x - x_ref + d_qp);

    // Regularization for numerical stability.
    H += 1e-6 * Eigen::MatrixXd::Identity(H.rows(), H.cols());

    // Set up the OSQP solver.
    OsqpEigen::Solver solver;
    const int n_vars = (N - 1) * n_u;
    int total_constraints = 0;
    total_constraints += n_vars;       // Output boundary.
    total_constraints += n_vars;       // Output selection.
    total_constraints += (N - 1) * 4;  // Friction cone constraints.

    solver.data()->setNumberOfVariables(n_vars);
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    solver.data()->setHessianMatrix(H_sparse);
    solver.data()->setGradient(g);

    // Construct the constraint matrix and bounds.
    Eigen::SparseMatrix<double> constraints(total_constraints, n_vars);
    Eigen::VectorXd lower_bound(total_constraints);
    Eigen::VectorXd upper_bound(total_constraints);

    // Output limitation constraints.
    for (int i = 0; i < n_vars; i++) {
        constraints.insert(i, i) = 1.0;

        if (i % 3 == 0) {
            lower_bound(i) = fx_lower_bound;
            upper_bound(i) = fx_upper_bound;
        }
        else if (i % 3 == 1) {
            lower_bound(i) = 0;
            upper_bound(i) = 0;
        }
        else {
            lower_bound(i) = fz_lower_bound;
            upper_bound(i) = fz_upper_bound;
        }
    }
    
    // Output selection constraints.
    Eigen::VectorXd D = Eigen::VectorXd::Zero(n_vars);
    for (int k = 0; k < (N - 1); ++k) {
        for (int i = 0; i < 4; i++) {
            if (selection_matrix[i]) {
                D.segment(k * n_u + i * 3, 3) << 0, 1, 0;
            }
            else {
                D.segment(k * n_u + i * 3, 3) << 1, 1, 1;
            }
        }
    }

    for (int i = 0; i < n_vars; i++) {
        constraints.insert(n_vars + i, i) = D(i);
        lower_bound(n_vars + i) = 0.0;
        upper_bound(n_vars + i) = 0.0;
    }

    // Friction cone constraints.
    for (int k = 0; k < (N - 1); ++k) {
        for (int i = 0; i < 4; ++i) {
            int row_index = 2 * n_vars + k * 4 + i;
            int col_index = k * n_u + i * 3;
            constraints.insert(row_index, col_index) = 1.0;
            lower_bound(row_index) = std::min(0.0, -friction_coef * force_state_modules[i]->Fy);
            upper_bound(row_index) = std::max(0.0,  friction_coef * force_state_modules[i]->Fy);
        }
    }

    constraints.makeCompressed();
    
    solver.data()->setNumberOfConstraints(total_constraints);
    solver.data()->setLinearConstraintsMatrix(constraints);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    // Set solver settings.
    // ----------------------------------------------------------------
    solver.settings()->setVerbosity(false);
    // solver.settings()->setAbsoluteTolerance(1.0e-3);
    // solver.settings()->setRelativeTolerance(1.0e-3);
    // solver.settings()->setPrimalInfeasibilityTolerance(1.0e-4);
    // solver.settings()->setDualInfeasibilityTolerance(1.0e-4);
    
    // // ADMM parameters.
    // solver.settings()->setRho(1.0e-1);        // ADMM penalty parameter (adaptive if available)
    // solver.settings()->setSigma(1.0e-6);      // Regularization parameter
    // solver.settings()->setAlpha(1.60);        // Over-relaxation parameter

    // // Other settings.
    // solver.settings()->setMaxIteration(4000);
    // solver.settings()->setCheckTermination(25);
    // solver.settings()->setScaling(true);
    // solver.settings()->setScaledTerimination(false);
    // solver.settings()->setWarmStart(true);
    // solver.settings()->setPolish(false);
    // ----------------------------------------------------------------

    // Initialize and solve the QP.
    if (!solver.initSolver()) {
        throw std::runtime_error("OSQP initialization failed");
    }
    
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("OSQP solver failed");
    }
    
    // Return the first control input.
    Eigen::VectorXd u_opt = solver.getSolution();
    return u_opt.head(n_u);
}
