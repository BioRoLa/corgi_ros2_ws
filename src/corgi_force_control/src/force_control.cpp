#include <sched.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include "corgi_force_control/force_control.hpp"

ForceControlNode::ForceControlNode()
    : Node("force_control"),
      kinematics_(sim_),
      phi_vel_prev_modules_(4, Eigen::MatrixXd::Zero(3, 1)),
      phi_prev_modules_(4, Eigen::MatrixXd::Zero(3, 1))
{
    RCLCPP_INFO(this->get_logger(), "Force Control Starts");

    // ---- CLOCK FF SIGN CHECK -----------------------------------------------
    //
    // PROOF OF SIGN, at startup, from the same LegModel the law uses.
    //
    // e_t = (-e_r_z, 0, e_r_x) is a fixed +90 deg rotation of the radial unit
    // vector. K and B take it as the outer product e_t*e_t^T, so its sign has
    // never mattered and nothing in the tree ever established it. The clocked
    // feedforward is LINEAR in e_t: get the sign wrong and the term doubles the
    // brake instead of cancelling it, which would read as a clean falsification
    // of a correct hypothesis. So assert it rather than reason about it.
    //
    // The assertion: sweeping beta forward must move the contact point along
    // +e_t. Checked by central difference at three poses across the stance
    // sweep, at the gait's median theta (85.4 deg, S151).
    {
        LegModel& lm = kinematics_.get_leg_model();
        const double th = 85.4 / 180.0 * M_PI;
        const double db = 1e-4;
        const double betas[3] = {-0.1614, 0.0, +0.1609};   // v070 TD, mid, LO
        double cosines[3] = {0.0, 0.0, 0.0};
        bool sign_ok = true;
        for (int i = 0; i < 3; ++i) {
            lm.contact_map_3d(th, betas[i], 0.0);
            const double rx = lm.contact_p_3d[0], rz = lm.contact_p_3d[2];
            const double rn = std::sqrt(rx*rx + rz*rz);
            const double etx = -rz / rn, etz = rx / rn;

            lm.contact_map_3d(th, betas[i] + db, 0.0);
            const double px = lm.contact_p_3d[0], pz = lm.contact_p_3d[2];
            lm.contact_map_3d(th, betas[i] - db, 0.0);
            const double mx = lm.contact_p_3d[0], mz = lm.contact_p_3d[2];

            const double dx = (px - mx) / (2.0 * db), dz = (pz - mz) / (2.0 * db);
            const double dn = std::sqrt(dx*dx + dz*dz);
            cosines[i] = (dn > 1e-12) ? (etx*dx + etz*dz) / dn : 0.0;
            if (!(cosines[i] > 0.5)) sign_ok = false;
        }
        RCLCPP_WARN(this->get_logger(),
                    "CLOCK FF SIGN CHECK: cos(e_t, d(contact)/dbeta) = "
                    "%+.4f %+.4f %+.4f at beta = -0.1614 / 0.0 / +0.1609, "
                    "theta = 85.4 deg -- %s (need all > +0.5)",
                    cosines[0], cosines[1], cosines[2],
                    sign_ok ? "PASS" : "FAIL");
        if (!sign_ok) {
            RCLCPP_FATAL(this->get_logger(),
                         "CLOCK FF SIGN CHECK FAILED: +dbeta_ref would push the "
                         "contact point BACKWARD along e_t, so the clocked "
                         "feedforward would ADD to the brake it is meant to "
                         "cancel. Refusing to run. Flip the sign of e_t in "
                         "force_control(), re-run this check, and note it in "
                         "the log before any campaign.");
            throw std::runtime_error("clock FF sign check failed");
        }
    }
    
    // Friction feedforward shaping. scale 0.0 turns the term off entirely,
    // which is the one-run test for whether it is the jerk.
    friction_ff_scale_ =
        this->declare_parameter<double>("friction_ff_scale", friction_ff_scale_);
    friction_deadband_ =
        this->declare_parameter<double>("friction_deadband_rad", friction_deadband_);
    RCLCPP_WARN(this->get_logger(),
                "FRICTION FF: scale=%.2f deadband=%.5f rad (%.4f deg; CAN LSB "
                "is 0.1648 deg). Latched direction, nothing until real "
                "motion. scale=0 disables the term.",
                friction_ff_scale_, friction_deadband_,
                friction_deadband_ * 180.0 / M_PI);

    // Wait for clock synchronization
    RCLCPP_INFO(this->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (this->now().seconds() > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    imp_cmd_sub_ = this->create_subscription<corgi_msgs::msg::ImpedanceCmdStamped>(
        "impedance/command", 10, 
        std::bind(&ForceControlNode::imp_cmd_cb, this, std::placeholders::_1));
    
    force_state_sub_ = this->create_subscription<corgi_msgs::msg::ForceStateStamped>(
        "force/state", 10, 
        std::bind(&ForceControlNode::force_state_cb, this, std::placeholders::_1));
    
    motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 10, 
        std::bind(&ForceControlNode::motor_state_cb, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 10, 
        std::bind(&ForceControlNode::imu_cb, this, std::placeholders::_1));
    
    motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>(
        "motor/command", 10);
}

void ForceControlNode::imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr msg) {
    imp_cmd_ = *msg;
}

void ForceControlNode::force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg) {
    force_state_ = *msg;
}

void ForceControlNode::motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state_ = *msg;
}

void ForceControlNode::imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu_ = *msg;
}

void ForceControlNode::quaternion_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

void ForceControlNode::force_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                                       Eigen::MatrixXd phi_vel_prev_, 
                                       corgi_msgs::msg::MotorState* motor_state_, 
                                       corgi_msgs::msg::ForceState* force_state_, 
                                       corgi_msgs::msg::MotorCmd* motor_cmd_, 
                                       double pitch) {
    // force command (3D)
    Eigen::MatrixXd force_des(3, 1);
    force_des << imp_cmd_->fx, imp_cmd_->fy, imp_cmd_->fz;

    Eigen::MatrixXd force_err(3, 1);
    force_err << force_des(0, 0)-force_state_->fx, 
                 force_des(1, 0)-force_state_->fy, 
                 force_des(2, 0)-force_state_->fz;
    
    // position command and state (3D)
    Eigen::MatrixXd pos_des(3, 1);

    LegModel& legmodel = kinematics_.get_leg_model();
            
    Eigen::MatrixXd P_poly = Eigen::MatrixXd::Zero(2, 8);
    Eigen::MatrixXd P_poly_deriv = Eigen::MatrixXd::Zero(2, 7);
    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);
    
    // Use contact_map_3d to get alpha and contact point
    legmodel.contact_map_3d(motor_state_->theta, motor_state_->beta + pitch, motor_state_->gamma);
    
    // calculate jacobian
    P_poly = kinematics_.calculate_P_poly_3d(legmodel.alpha);
    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);
    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(motor_state_->theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(motor_state_->theta, i);

    double beta_fb = motor_state_->beta + pitch;
    double gamma_fb = motor_state_->gamma;
    double cos_beta = std::cos(beta_fb);
    double sin_beta = std::sin(beta_fb);
    double cos_gamma = std::cos(gamma_fb);
    double sin_gamma = std::sin(gamma_fb);

    double p_poly_x = P_theta(0, 0);
    double p_poly_z = P_theta(1, 0);
    double p_beta_x = p_poly_x * cos_beta - p_poly_z * sin_beta;
    double p_beta_z = p_poly_x * sin_beta + p_poly_z * cos_beta;
    double p_expected_x = p_beta_x;
    double p_expected_y = legmodel.d_wheel * cos_gamma - p_beta_z * sin_gamma;
    double p_expected_z = legmodel.d_wheel * sin_gamma + p_beta_z * cos_gamma;
    double err_x = legmodel.contact_p_3d[0] - p_expected_x;
    double err_y = legmodel.contact_p_3d[1] - p_expected_y;
    double err_z = legmodel.contact_p_3d[2] - p_expected_z;

    Eigen::MatrixXd J_fb(3, 3);
    J_fb = kinematics_.calculate_jacobian_3d(P_theta, P_theta_deriv, motor_state_->beta + pitch, motor_state_->gamma, legmodel.d_wheel);

    Eigen::MatrixXd phi_vel(3, 1);
    phi_vel << motor_state_->velocity_l, motor_state_->velocity_r, motor_state_->velocity_h;
    Eigen::MatrixXd vel_fb = J_fb * phi_vel;
    Eigen::MatrixXd acc_fb = (vel_fb - J_fb * phi_vel_prev_) * 1000;
    
    // impedance control
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd eta_cmd(3, 1);
    Eigen::MatrixXd trq_cmd_base(3, 1);
    Eigen::MatrixXd trq_cmd(3, 1);

    M(0, 0) = imp_cmd_->mx; M(1, 1) = imp_cmd_->my; M(2, 2) = imp_cmd_->mz;

    // The leg-frame basis, built from the MEASURED contact point:
    //   e_r  radial, hip -> contact, projected into the sagittal (x-z) plane
    //   e_t  tangential, perpendicular to e_r in that plane
    //   e_y  lateral, body y
    //
    // Hoisted out of the leg_frame branch (2026-08-22) because the clocked
    // feedforward below needs e_t in BOTH phases: Lu & Lin eq 11 regulates the
    // leg in stance and in flight (p5, S2.3), and the flight branch commands
    // isotropic gains for which the rotation is the identity anyway.
    Eigen::Vector3d e_r(legmodel.contact_p_3d[0], 0.0, legmodel.contact_p_3d[2]);
    const double radial_norm = e_r.norm();
    const bool leg_basis_ok = radial_norm >= 1e-9;
    Eigen::Vector3d e_t(0.0, 0.0, 0.0);
    if (leg_basis_ok) {
        e_r /= radial_norm;
        e_t = Eigen::Vector3d(-e_r(2), 0.0, e_r(0));
    }

    if (imp_cmd_->leg_frame) {
        // The G-SLIP spring acts along the leg, so the commanded stiffness is
        // given in the leg frame and rotated here. Building the basis from the
        // contact point keeps K and B free of sign conventions -- e_t enters as
        // the outer product e_t*e_t^T, where its sign cancels. NOTE that the
        // feedforward below is LINEAR in e_t, so its sign does NOT cancel; that
        // is what the constructor's sign self-check exists to catch.
        if (!leg_basis_ok) {
            // Degenerate (contact directly under the hip axis): fall back to
            // body-frame axes rather than normalising a zero vector.
            K(0, 0) = imp_cmd_->kx; K(1, 1) = imp_cmd_->ky; K(2, 2) = imp_cmd_->kz;
            B(0, 0) = imp_cmd_->bx; B(1, 1) = imp_cmd_->by; B(2, 2) = imp_cmd_->bz;
        } else {
            Eigen::Vector3d e_y(0.0, 1.0, 0.0);

            K = imp_cmd_->kx * (e_r * e_r.transpose())
              + imp_cmd_->ky * (e_y * e_y.transpose())
              + imp_cmd_->kz * (e_t * e_t.transpose());
            B = imp_cmd_->bx * (e_r * e_r.transpose())
              + imp_cmd_->by * (e_y * e_y.transpose())
              + imp_cmd_->bz * (e_t * e_t.transpose());
        }
    } else {
        B(0, 0) = imp_cmd_->bx; B(1, 1) = imp_cmd_->by; B(2, 2) = imp_cmd_->bz;
        K(0, 0) = imp_cmd_->kx; K(1, 1) = imp_cmd_->ky; K(2, 2) = imp_cmd_->kz;
    }

    // ---- The clocked-torque feedforward (Lu & Lin 2024 eq 11's D term) -------
    //
    // The damper above brakes to ZERO velocity: its torque is -B_joint*phi_dot,
    // executed as the driver's diagonal kd plus the off-diagonal coupling below.
    // eq 11 damps to the CLOCK'S rate instead:  k_D*(dtheta_fp - dtheta).
    // The difference is a constant feedforward +B*v_ref, which is what this is.
    //
    // v_ref is the commanded tangential contact velocity: the leg-angle rate
    // dbeta_ref times the lever |hip -> contact|, which is radial_norm -- the
    // SAME measured geometry e_t came from, so the lever is never re-derived
    // anywhere else. Together with the B term above the tangential law is now
    // bz*(v_ref - v_measured).
    //
    // Co-gated with the damping it corrects by construction: same bz, same e_t,
    // same degeneracy guard. dbeta_ref == 0 reproduces the old behaviour bitwise.
    Eigen::MatrixXd f_clock = Eigen::MatrixXd::Zero(3, 1);
    if (leg_basis_ok && imp_cmd_->dbeta_ref != 0.0) {
        f_clock = imp_cmd_->bz * (imp_cmd_->dbeta_ref * radial_norm) * e_t;
    }

    // PROOF OF ACTION, from inside the law itself.
    //
    // S151 established that a printed PARAMETER proves only that it was
    // read -- S138's apex channel printed its parameters faithfully for ten
    // runs and never fired. This line is different: it prints the value the
    // impedance law just computed, from the live message and the measured
    // pose. If it reads 0.00 on an arm that commanded a non-zero scale, the
    // term did not reach the law and the run is void.
    //
    // Throttled to 5 s. Which leg wins the throttle is arbitrary and does not
    // matter -- all four are commanded the same clock.
    //
    // Needed because the arm-vs-arm statistical check is confounded: measured
    // on banked config-of-record captures, the stance median of
    // (t_ff_L + t_ff_R) moves 3.9 N.m between two cells that BOTH ran with no
    // feedforward at all, purely from the pose change. That is 60% of the
    // signal this term is meant to add.
    // ONLY when the term is actually live. The first version printed on every
    // throttle window regardless, which sounds harmless and is not: with
    // clock_ff_phase = "stance", 59.4% of the v070 cycle is flight where
    // dbeta_ref is 0, so a correctly engaged arm printed "tau_beta=+0.000"
    // most of the time -- and the harness assert reads ONE line with head -1.
    // A correct arm would have been marked INVALID roughly 3 times in 5.
    // Printing only when non-zero makes the LINE ITSELF the evidence.
    if (leg_basis_ok && imp_cmd_->dbeta_ref != 0.0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "CLOCK FF ACTIVE: dbeta_ref=%+.4f rad/s bz=%.1f lever=%.4f m "
            "-> |f_clock|=%.3f N, tau_beta=%+.3f N.m (%.3f N.m/motor)",
            imp_cmd_->dbeta_ref, imp_cmd_->bz, radial_norm, f_clock.norm(),
            imp_cmd_->bz * imp_cmd_->dbeta_ref * radial_norm * radial_norm,
            0.5 * imp_cmd_->bz * imp_cmd_->dbeta_ref * radial_norm * radial_norm);
    }

    eta_cmd << imp_cmd_->theta, imp_cmd_->beta, imp_cmd_->gamma;
    trq_cmd_base = J_fb.transpose() * (force_des + f_clock + M*(-acc_fb));

    // calculate phi command
    Eigen::MatrixXd phi_des(3, 1);
    phi_des << eta_cmd(1, 0) + eta_cmd(0, 0) - 17/180.0*M_PI,
               eta_cmd(1, 0) - eta_cmd(0, 0) + 17/180.0*M_PI,
               eta_cmd(2, 0);

    Eigen::MatrixXd phi_fb(3, 1);
    phi_fb << motor_state_->beta + pitch + motor_state_->theta - 17/180.0*M_PI,
              motor_state_->beta + pitch - motor_state_->theta + 17/180.0*M_PI,
              motor_state_->gamma;

    Eigen::MatrixXd phi_err = phi_des-phi_fb;

    // Equivalent joint PD gains
    Eigen::MatrixXd K_joint = J_fb.transpose() * K * J_fb;
    Eigen::MatrixXd B_joint = J_fb.transpose() * B * J_fb;

    // Extract diagonal terms for actual PD controller
    double kp_l_cmd = K_joint(0, 0);
    double kp_r_cmd = K_joint(1, 1);
    double kp_h_cmd = K_joint(2, 2);

    double kd_l_cmd = B_joint(0, 0);
    double kd_r_cmd = B_joint(1, 1);
    double kd_h_cmd = B_joint(2, 2);

    // Initial torque command
    trq_cmd = trq_cmd_base;

    // Coupling compensation
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            if (i != j) {
                trq_cmd(i) += K_joint(i, j) * phi_err(j);
                trq_cmd(i) += B_joint(i, j) * (-phi_vel(j));
            }
        }
    }

    // send to motor command
    motor_cmd_->theta = eta_cmd(0, 0);
    motor_cmd_->beta = eta_cmd(1, 0);
    motor_cmd_->gamma = eta_cmd(2, 0);
    motor_cmd_->kp_l = kp_l_cmd;
    motor_cmd_->kp_r = kp_r_cmd;
    motor_cmd_->kp_h = kp_h_cmd;
    motor_cmd_->kd_l = kd_l_cmd;
    motor_cmd_->kd_r = kd_r_cmd;
    motor_cmd_->kd_h = kd_h_cmd;
    motor_cmd_->torque_l = trq_cmd(0, 0);
    motor_cmd_->torque_r = trq_cmd(1, 0);
    motor_cmd_->torque_h = trq_cmd(2, 0);
}

void ForceControlNode::position_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, 
                                        corgi_msgs::msg::MotorCmd* motor_cmd_) {
    motor_cmd_->theta = imp_cmd_->theta;
    motor_cmd_->beta = imp_cmd_->beta;
    motor_cmd_->gamma = imp_cmd_->gamma;
    motor_cmd_->kp_r = 50;
    motor_cmd_->kp_l = 50;
    motor_cmd_->kp_h = 50;
    motor_cmd_->kd_r = 1;
    motor_cmd_->kd_l = 1;
    motor_cmd_->kd_h = 1;
    motor_cmd_->torque_r = 0;
    motor_cmd_->torque_l = 0;
    motor_cmd_->torque_h = 0;
}

void ForceControlNode::timer_cb() {
    Eigen::Quaterniond body_angle_quat;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    body_angle_quat = {imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z};
    quaternion_to_euler(body_angle_quat, roll, pitch, yaw);

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd_.module_a,
        &imp_cmd_.module_b,
        &imp_cmd_.module_c,
        &imp_cmd_.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state_.module_a,
        &force_state_.module_b,
        &force_state_.module_c,
        &force_state_.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state_.module_a,
        &motor_state_.module_b,
        &motor_state_.module_c,
        &motor_state_.module_d
    };

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd_.module_a,
        &motor_cmd_.module_b,
        &motor_cmd_.module_c,
        &motor_cmd_.module_d
    };

    for (int i=0; i<4; i++){
        if (imp_cmd_modules[i]->kx == 0 && imp_cmd_modules[i]->ky == 0 && imp_cmd_modules[i]->kz == 0 && 
            imp_cmd_modules[i]->bx == 0 && imp_cmd_modules[i]->by == 0 && imp_cmd_modules[i]->bz == 0) {
            position_control(imp_cmd_modules[i], motor_cmd_modules[i]);
        }
        else {
            if (imp_cmd_modules[i]->theta < 17/180.0*M_PI) { imp_cmd_modules[i]->theta = 17/180.0*M_PI; }

            if (i == 1 || i == 2) {
                force_control(imp_cmd_modules[i], phi_vel_prev_modules_[i], motor_state_modules[i], force_state_modules[i], motor_cmd_modules[i], -pitch);
            }
            else {
                force_control(imp_cmd_modules[i], phi_vel_prev_modules_[i], motor_state_modules[i], force_state_modules[i], motor_cmd_modules[i], pitch);
            }
            
        }
        phi_vel_prev_modules_[i] << motor_state_modules[i]->velocity_l, motor_state_modules[i]->velocity_r, motor_state_modules[i]->velocity_h;
    }

    // dynamic friction compensation
    if (!kinematics_.is_sim()){
        for (int i=0; i<4; i++) {
            double phi_l = motor_state_modules[i]->theta + motor_state_modules[i]->beta - 17/180.0*M_PI;
            double phi_r = motor_state_modules[i]->beta - motor_state_modules[i]->theta + 17/180.0*M_PI;
            double gamma_fb = motor_state_modules[i]->gamma;

            // Deadband + direction latch. The raw form was
            //     if (phi > phi_prev) -= f;  else += f;
            // which, on a CAN position word quantised at 0.1648 deg while
            // the standup ramp advances 0.0413 deg per tick, holds
            // phi == phi_prev for ~4 ticks in 5 -- so it read QUANTISATION,
            // not direction, and chattered +/-0.625 N.m (1.25 peak to peak)
            // at up to 1 kHz against a 2.6-3.3 N.m load. The equal case
            // fell to `else`, so a stationary joint also carried a
            // permanent +friction bias. Live in EVERY phase, standup and
            // pronk alike, which is why nothing about scheduling, QoS or
            // startup order ever touched the symptom.
            //
            // Corroborated in Alex's bag: cmd_trq_r_a has a monotonicity
            // ratio of 0.00 -- it reverses constantly and goes nowhere.
            //
            // The sign convention is unchanged; only WHEN it flips changes.
            // phi_prev advances ONLY on a decisive move, so slow real
            // motion accumulates instead of being lost to rounding, and the
            // direction starts at 0 so a parked leg carries no bias.
            const double d_r = phi_r - phi_prev_modules_[i](1, 0);
            if (d_r > friction_deadband_) {
                fric_dir_[i][1] = +1;
                phi_prev_modules_[i](1, 0) = phi_r;
            } else if (d_r < -friction_deadband_) {
                fric_dir_[i][1] = -1;
                phi_prev_modules_[i](1, 0) = phi_r;
            }
            if (fric_dir_[i][1] > 0)
                motor_cmd_modules[i]->torque_r -= friction_ff_scale_ * friction_[2*i];
            else if (fric_dir_[i][1] < 0)
                motor_cmd_modules[i]->torque_r += friction_ff_scale_ * friction_[2*i];

            const double d_l = phi_l - phi_prev_modules_[i](0, 0);
            if (d_l > friction_deadband_) {
                fric_dir_[i][0] = +1;
                phi_prev_modules_[i](0, 0) = phi_l;
            } else if (d_l < -friction_deadband_) {
                fric_dir_[i][0] = -1;
                phi_prev_modules_[i](0, 0) = phi_l;
            }
            if (fric_dir_[i][0] > 0)
                motor_cmd_modules[i]->torque_l -= friction_ff_scale_ * friction_[2*i+1];
            else if (fric_dir_[i][0] < 0)
                motor_cmd_modules[i]->torque_l += friction_ff_scale_ * friction_[2*i+1];

            // gamma has no friction term specified; only its history is kept.
            phi_prev_modules_[i](2, 0) = gamma_fb;
        }
    }

    motor_cmd_.header.stamp = this->now();
    
    motor_cmd_pub_->publish(motor_cmd_);

    loop_count_++;
}

void ForceControlNode::run() {
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        timer_cb();
        
        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}



// S313: real-time scheduling for the 1 kHz loop. Two of three hardware
// air runs on 2026-08-31 published motor commands whose VALUES changed at
// ~15 Hz while every timestamp still said 1 kHz -- this loop was
// publishing on schedule but consuming STALE imp_cmd, i.e. spin_some was
// starved of CPU. The robot is then step-commanded in ~50x jumps (the
// observed "spazzing"). SCHED_FIFO makes the loop immune to that load.
// Needs CAP_SYS_NICE or root; without it we warn and continue at normal
// priority rather than refusing to start. Off with CORGI_RT_PRIORITY=0.
static void corgi_try_realtime(const rclcpp::Logger& log) {
    const char* env = std::getenv("CORGI_RT_PRIORITY");
    int prio = env ? std::atoi(env) : 80;
    if (prio <= 0) {
        RCLCPP_WARN(log, "REALTIME: disabled (CORGI_RT_PRIORITY=%d)", prio);
        return;
    }
    struct sched_param sp;
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0) {
        RCLCPP_WARN(log, "REALTIME: SCHED_FIFO priority %d ACQUIRED -- the "
                         "1 kHz loop is protected from desktop/encoder load",
                    prio);
    } else {
        RCLCPP_WARN(log, "REALTIME: could NOT set SCHED_FIFO %d (%s). Running "
                         "at normal priority: commands may degrade to ~15 Hz "
                         "under CPU load (S313). Fix: sudo setcap "
                         "cap_sys_nice+ep on the binary, or 'chrt -f %d'.",
                    prio, std::strerror(errno), prio);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceControlNode>();
    corgi_try_realtime(node->get_logger());
    node->run();
    rclcpp::shutdown();
    return 0;
}
