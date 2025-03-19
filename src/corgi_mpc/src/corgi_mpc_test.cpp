#include "corgi_mpc.hpp"


bool trigger = false;
corgi_msgs::SimDataStamped sim_data;
corgi_msgs::ForceStateStamped force_state;
corgi_msgs::MotorStateStamped motor_state;
geometry_msgs::Vector3 odom_pos;
geometry_msgs::Vector3 odom_vel;
sensor_msgs::Imu imu;

Eigen::MatrixXd A(n_x, n_x);
Eigen::MatrixXd B(n_x, n_u);
Eigen::MatrixXd Q(n_x, n_x);
Eigen::MatrixXd R(n_u, n_u);

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void sim_data_cb(const corgi_msgs::SimDataStamped data){
    sim_data = data;
}

void force_state_cb(const corgi_msgs::ForceStateStamped msg){
    force_state = msg;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped msg){
    motor_state = msg;
}

void odom_pos_cb(const geometry_msgs::Vector3::ConstPtr &msg){
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::Vector3::ConstPtr &msg){
    odom_vel = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    imu = *msg;
}

void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

void init_matrices(const double *ra, const double *rb, const double *rc, const double *rd) {
    A << 1, 0, 0, dt, 0,0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, dt,0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, dt,0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0, dt,0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt,0,
         0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
    Q.diagonal() << 2e9, 0, 1e9, 1e7, 0, 1e7, 1e9, 1e9, 1e9, 1e6, 1e6, 1e6;
    // Q.diagonal() << 1e9, 1e9, 1e9, 1e7, 1e7, 1e7, 0, 0, 0, 0, 0, 0;
    R = Eigen::MatrixXd::Identity(n_u, n_u);
}

Eigen::VectorXd model_predictive_control(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref, const bool *selection_matrix) {
    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(N * n_x, n_x);
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(N * n_x, (N - 1) * n_u);

    Eigen::VectorXd d = Eigen::VectorXd::Zero(n_x);
    d(2) = -0.5 * dt * dt * gravity;
    d(5) = -dt * gravity;

    Eigen::VectorXd d_qp = Eigen::VectorXd::Zero(N * n_x);
    for (int i = 0; i < N; i++) {
        d_qp.segment(i * n_x, n_x) = d;
    }

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

    Eigen::MatrixXd Q_N = Eigen::MatrixXd::Zero(N * n_x, N * n_x);
    for (int i = 0; i < N; ++i){
        Q_N.block(i * n_x, i * n_x, n_x, n_x) = Q;
    }

    Eigen::MatrixXd R_N = Eigen::MatrixXd::Zero((N - 1) * n_u, (N - 1) * n_u);
    for (int i = 0; i < (N - 1); ++i){
        R_N.block(i * n_u, i * n_u, n_u, n_u) = R;
    }

    Eigen::MatrixXd H = 2 * (B_qp.transpose() * Q_N * B_qp + R_N);
    Eigen::VectorXd g = 2 * B_qp.transpose() * Q_N * (A_qp * x - x_ref + d_qp);

    H += 1e-6 * Eigen::MatrixXd::Identity(H.rows(), H.cols());

    OsqpEigen::Solver solver;
    const int n_vars = (N - 1) * n_u;
    int total_constraints = n_vars + n_vars;

    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(total_constraints);

    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    solver.data()->setHessianMatrix(H_sparse);
    solver.data()->setGradient(g);

    Eigen::SparseMatrix<double> constraints(total_constraints, n_vars);
    Eigen::VectorXd lower_bound(total_constraints);
    Eigen::VectorXd upper_bound(total_constraints);

    for (int i = 0; i<n_vars; i++) {
        constraints.insert(i, i) = 1.0;

        if (i % 3 == 0) {
            lower_bound(i) = -50;
            upper_bound(i) = 50;
        }
        else if (i % 3 == 1) {
            lower_bound(i) = 0;
            upper_bound(i) = 0;
        }
        else {
            lower_bound(i) = -80;
            upper_bound(i) = 200;
        }
    }
    
    Eigen::VectorXd D(n_vars);
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
        constraints.insert(n_vars+i, i) = D(i);
        lower_bound(n_vars+i) = 0.0;
        upper_bound(n_vars+i) = 0.0;
    }

    constraints.makeCompressed();
    
    solver.data()->setNumberOfConstraints(total_constraints);
    solver.data()->setLinearConstraintsMatrix(constraints);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    // -------------------- OSQP Solver Settings --------------------
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


int main(int argc, char **argv) {
    ROS_INFO("Corgi MPC Starts");

    ros::init(argc, argv, "corgi_mpc");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Subscriber sim_data_sub = nh.subscribe<corgi_msgs::SimDataStamped>("sim/data", 1000, sim_data_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber odom_pos_sub = nh.subscribe<geometry_msgs::Vector3>("odometry/position", 1000, odom_pos_cb);
    ros::Subscriber odom_vel_sub = nh.subscribe<geometry_msgs::Vector3>("odometry/velocity", 1000, odom_vel_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1000, imu_cb);

    ros::Rate rate(100);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    double M = 0;
    double K = 5000;
    double B = 30;

    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fy = -55;

        cmd->Mx = M;
        cmd->My = M;
        cmd->Bx = B;
        cmd->By = B;
        cmd->Kx = K;
        cmd->Ky = K;
    }

    ROS_INFO("Transform Finished\n");


    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    WalkGait walk_gait(true, 0, 100);
    walk_gait.initialize(init_eta);
    std::array<std::array<double, 4>, 2> eta_list;
    double velocity     = 0.1;
    double stand_height = 0.25;
    double step_length  = 0.3;
    double step_height  = 0.1;
    double curvature = 0.0;
    int count = 0;
    bool touched[4] = {true, true, true, true};
    bool selection_matrix[4] = {true, true, true, true};
    
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    walk_gait.set_curvature(curvature);


    for (int i=0; i<200; i++) {
        for (int j=0; j<4; j++) {
            imp_cmd_modules[j]->theta += init_eta[2*j]/200.0;
            imp_cmd_modules[j]->beta += init_eta[2*j+1]/200.0;
        }
        imp_cmd.header.seq = -1;
        imp_cmd_pub.publish(imp_cmd);
        rate.sleep();
    }

    for (int i=0; i<200; i++) {
        imp_cmd.header.seq = -1;
        imp_cmd_pub.publish(imp_cmd);
        rate.sleep();
    }


    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        for (auto& cmd : imp_cmd_modules){
            cmd->Kx = 1000;
            cmd->Ky = 1000;
        }

        if (trigger){
            int seq = 0;
            while (ros::ok()) {
                ros::spinOnce();

                eta_list = walk_gait.step();

                for (int i=0; i<4; i++) {
                    imp_cmd_modules[i]->theta = eta_list[0][i];
                    imp_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
                    if (walk_gait.swing_phase[i] == 1 && touched[i]) {
                        selection_matrix[i] = false;
                        touched[i] = false;
                    }
                    else if (walk_gait.swing_phase[i] == 0 && !touched[i]) {
                        selection_matrix[i] = true;
                        touched[i] = true;
                    }
                }
                    
                // const double robot_pos[3] = {odom_pos.x, odom_pos.y, odom_pos.z};
                // const double robot_vel[3] = {odom_vel.x, odom_vel.y, odom_vel.z};
                robot_vel[0] = (sim_data.position.x-robot_pos[0])/dt;
                robot_vel[1] = (sim_data.position.y-robot_pos[1])/dt;
                robot_vel[2] = (sim_data.position.z-robot_pos[2])/dt;

                robot_pos[0] = sim_data.position.x;
                robot_pos[1] = sim_data.position.y;
                robot_pos[2] = sim_data.position.z;

                Eigen::Quaterniond robot_ang;
                robot_ang = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
                const double robot_ang_vel[3] = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};

                quaternion_to_euler(robot_ang, roll, pitch, yaw);

                Eigen::VectorXd x(n_x);
                x << robot_pos[0], robot_pos[1], robot_pos[2],
                     robot_vel[0], robot_vel[1], robot_vel[2],
                     roll,         pitch,        yaw,
                     robot_ang_vel[0], robot_ang_vel[1], robot_ang_vel[2];

                Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(N * n_x);
                for (int i = 0; i < N; ++i) {
                    x_ref.segment(i * n_x, n_x) << loop_count*velocity*dt, 0, stand_height, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                }

                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
                double ra[3] = {-legmodel.contact_p[0]+0.222, 0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
                double rb[3] = {legmodel.contact_p[0]+0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
                double rc[3] = {legmodel.contact_p[0]-0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
                double rd[3] = {-legmodel.contact_p[0]-0.222, 0.2, legmodel.contact_p[1]};

                init_matrices(ra, rb, rc, rd);
                
                Eigen::VectorXd force = model_predictive_control(x, x_ref, selection_matrix);

                double force_A[3] = {force(0), force(1), force(2)};
                double force_B[3] = {force(3), force(4), force(5)};
                double force_C[3] = {force(6), force(7), force(8)};
                double force_D[3] = {force(9), force(10), force(11)};

                Eigen::Matrix3d R = robot_ang.toRotationMatrix();
                Eigen::Matrix3d R_T = R.transpose();
                
                auto convertForceToLocal = [&](double *f_global) {
                    Eigen::Vector3d f_global_vec(f_global[0], f_global[1], f_global[2]);
                    Eigen::Vector3d f_local = R_T * f_global_vec;
                    f_global[0] = f_local(0);
                    f_global[1] = f_local(1);
                    f_global[2] = f_local(2);
                };

                convertForceToLocal(force_A);
                convertForceToLocal(force_B);
                convertForceToLocal(force_C);
                convertForceToLocal(force_D);

                std::cout << "Ref Pos = [" << x_ref[0] << ", " << x_ref[1] << ", " << x_ref[2] << "]" << std::endl << std::endl;
                std::cout << "Odom Pos = [" << robot_pos[0] << ", " << robot_pos[1] << ", " << robot_pos[2] << "]" << std::endl;
                std::cout << "Odom Vel = [" << robot_vel[0] << ", " << robot_vel[1] << ", " << robot_vel[2] << "]" << std::endl;
                std::cout << "Odom Ang = [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
                std::cout << "Imu = [" << imu.orientation.x << ", " << imu.orientation.y << ", " << imu.orientation.z << "," << imu.orientation.w << "]" << std::endl << std::endl;

                std::cout << "ra = [" << ra[0] << ", " << ra[1] << ", " << ra[2] << "]" << std::endl;
                std::cout << "rb = [" << rb[0] << ", " << rb[1] << ", " << rb[2] << "]" << std::endl;
                std::cout << "rc = [" << rc[0] << ", " << rc[1] << ", " << rc[2] << "]" << std::endl;
                std::cout << "rd = [" << rd[0] << ", " << rd[1] << ", " << rd[2] << "]" << std::endl << std::endl;

                std::cout << "Force A: [" << force_A[0] << ", " << force_A[1] << ", " << -force_A[2] << "]" << std::endl;
                std::cout << "Force B: [" << force_B[0] << ", " << force_B[1] << ", " << -force_B[2] << "]" << std::endl;
                std::cout << "Force C: [" << force_C[0] << ", " << force_C[1] << ", " << -force_C[2] << "]" << std::endl;
                std::cout << "Force D: [" << force_D[0] << ", " << force_D[1] << ", " << -force_D[2] << "]" << std::endl << std::endl;


                imp_cmd_modules[0]->Fx =  force_A[0];
                imp_cmd_modules[0]->Fy = -force_A[2];
                imp_cmd_modules[1]->Fx = -force_B[0];
                imp_cmd_modules[1]->Fy = -force_B[2];
                imp_cmd_modules[2]->Fx = -force_C[0];
                imp_cmd_modules[2]->Fy = -force_C[2];
                imp_cmd_modules[3]->Fx =  force_D[0];
                imp_cmd_modules[3]->Fy = -force_D[2];


                imp_cmd.header.seq = seq;
                imp_cmd_pub.publish(imp_cmd);

                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                seq++;
                rate.sleep();
            }
            break;
        }   
    }

    ROS_INFO("MPC Finished\n");

    ros::shutdown();

    return 0;
}
