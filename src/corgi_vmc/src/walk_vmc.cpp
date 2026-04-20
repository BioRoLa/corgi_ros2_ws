#include "walk_utils.hpp"
#include "vmc.hpp"
#include "rclcpp/rclcpp.hpp"

bool trigger = false;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::MotorStateStamped motor_state;
geometry_msgs::msg::Vector3 odom_pos;
geometry_msgs::msg::Vector3 odom_vel;
double odom_z;
corgi_msgs::msg::ImuStamped imu;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg){
    force_state = *msg;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
    motor_state = *msg;
}

void odom_pos_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
    odom_vel = *msg;
}

void odom_z_cb(const std_msgs::msg::Float64::SharedPtr msg){
    odom_z = msg->data;
}

void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg){
    imu = *msg;
}

void convert_force_to_local(double *f_global, const Eigen::Matrix3d& R_T) {
    Eigen::Vector3d f_global_vec(f_global[0], f_global[1], f_global[2]);
    Eigen::Vector3d f_local = R_T * f_global_vec;
    f_global[0] = f_local(0);
    f_global[1] = f_local(1);
    f_global[2] = f_local(2);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_vmc");
    
    RCLCPP_INFO(node->get_logger(), "Corgi VMC Starts");
    
    // Wait for clock synchronization
    RCLCPP_INFO(node->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    VirtualModelController vmc;
    vmc.load_config();
    vmc.target_loop = 11250;

    auto imp_cmd_pub = node->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 10);
    auto contact_pub = node->create_publisher<corgi_msgs::msg::ContactStateStamped>("odometry/legacy/contact", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);
    auto force_state_sub = node->create_subscription<corgi_msgs::msg::ForceStateStamped>("force/state", 10, force_state_cb);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 10, motor_state_cb);
    auto odom_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>("odometry/legacy/position", 10, odom_pos_cb);
    auto odom_vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>("odometry/legacy/velocity", 10, odom_vel_cb);
    auto odom_z_sub = node->create_subscription<std_msgs::msg::Float64>("odometry/legacy/z_position_hip", 10, odom_z_cb);
    auto imu_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>("imu", 10, imu_cb);

    rclcpp::Duration period(0, 1000000000.0 / vmc.freq);
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
    corgi_msgs::msg::ContactStateStamped contact_state;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::msg::ContactState*> contact_state_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    double init_eta[8];

    if (sim) {
        double tmp[8] = {1.3313651941315507, 0.4032814817188362, 1.1847611807810603, 0.10626486289107877, 1.1847611807810603, -0.10626486289107877, 1.3313651941315507, -0.4032814817188362};
        for (int i = 0; i < 8; ++i) init_eta[i] = tmp[i];
    } else {
        double tmp[8] = {1.2744470401482761, 0.4161719979302237, 1.1222141023936798, 0.11005079310996896, 1.1222141023936798, -0.11005079310996896, 1.2744470401482761, -0.4161719979302237};
        for (int i = 0; i < 8; ++i) init_eta[i] = tmp[i];
    }

    vmc.target_pos_z = 0.2;
    
    WalkGait walk_gait(sim, 0, vmc.freq);
    double velocity = 0.1;

    walk_gait.stand_height = vmc.target_pos_z;
    walk_gait.velocity = velocity;
    walk_gait.step_length = 0.2;
    walk_gait.step_height = 0.06;

    walk_gait.initialize(init_eta, walk_gait.step_length);
    walk_gait.set_velocity(vmc.target_vel_x);

    bool touched[4] = {true, true, true, true};
    bool selection_matrix[4] = {true, true, true, true};

    // Initialize impedance command
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->fy = -vmc.m*vmc.gravity/4.0;

        cmd->mx = vmc.Mx;
        cmd->my = vmc.My;
        cmd->bx = vmc.Bx_swing;
        cmd->by = vmc.By_swing;
        cmd->kx = vmc.Kx_swing;
        cmd->ky = vmc.Ky_swing;
    }

    RCLCPP_INFO(node->get_logger(), "Wait For Force Control Node ...");
    
    if (!sim) {
        for (int i=0; i<int(3*vmc.freq); i++) {
            next_time += period;
            node->get_clock()->sleep_until(next_time);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Transform Starts");

    for (int i=0; i<int(3*vmc.freq); i++) {
        for (int j=0; j<4; j++) {
            imp_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/(3*vmc.freq);
            imp_cmd_modules[j]->beta += init_eta[2*j+1]/(3*vmc.freq);
        }
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    // stay
    for (int i=0; i<int(2*vmc.freq); i++) {
        rclcpp::spin_some(node);
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }
    

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");

            if (!sim) {
                for (int i=0; i<int(3*vmc.freq); i++) {
                    for (auto& state: contact_state_modules) {
                        state->contact = true;
                    }
                    contact_pub->publish(contact_state);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
            }
            else {
                for (int i=0; i<int(1*vmc.freq); i++) {
                    for (auto& state: contact_state_modules) {
                        state->contact = true;
                    }
                    contact_pub->publish(contact_state);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
            }
            
            for (auto& cmd : imp_cmd_modules){
                cmd->bx = vmc.Bx_stance;
                cmd->by = vmc.By_stance;
                cmd->kx = vmc.Kx_stance;
                cmd->ky = vmc.Ky_stance;
            }

            RCLCPP_INFO(node->get_logger(), "VMC Controller Starts ...");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // update target vel and pos
                if (loop_count < int(1*vmc.freq)) {
                    vmc.target_vel_x += velocity/(1*vmc.freq);
                    walk_gait.set_velocity(vmc.target_vel_x);
                }
                else if (loop_count > vmc.target_loop-int(1*vmc.freq) && loop_count < vmc.target_loop) {
                    vmc.target_vel_x -= velocity/(1*vmc.freq);
                    walk_gait.set_velocity(vmc.target_vel_x);
                }

                vmc.target_pos_x += vmc.target_vel_x * vmc.dt;

                // get next eta
                vmc.eta_list = walk_gait.step();

                for (int i=0; i<4; i++) {
                    imp_cmd_modules[i]->theta = vmc.eta_list[0][i];
                    imp_cmd_modules[i]->beta = (i == 1 || i == 2) ? vmc.eta_list[1][i] : -vmc.eta_list[1][i];
                    if (walk_gait.get_swing_phase()[i] == 1 && touched[i]) {
                        selection_matrix[i] = false;
                        touched[i] = false;
                        imp_cmd_modules[i]->by = vmc.By_swing;
                        imp_cmd_modules[i]->ky = vmc.Ky_swing;
                    }
                    else if (walk_gait.get_swing_phase()[i] == 0 && !touched[i]) {
                        selection_matrix[i] = true;
                        touched[i] = true;
                        imp_cmd_modules[i]->by = vmc.By_stance;
                        imp_cmd_modules[i]->ky = vmc.Ky_stance;
                    }

                    if (walk_gait.get_duty()[i] < 0.75 && walk_gait.get_duty()[i] > 0.05) {
                        contact_state_modules[i]->contact = true;
                    }
                    else {
                        contact_state_modules[i]->contact = false;
                    }
                }

                // update state
                vmc.robot_vel[0] = odom_vel.x;
                vmc.robot_vel[1] = odom_vel.y;
                vmc.robot_vel[2] = odom_vel.z;
                
                vmc.robot_pos[0] = odom_pos.x;
                vmc.robot_pos[1] = odom_pos.y;
                vmc.robot_pos[2] = odom_z;

                vmc.robot_ang.x() = imu.orientation.x;
                vmc.robot_ang.y() = imu.orientation.y;
                vmc.robot_ang.z() = imu.orientation.z;
                vmc.robot_ang.w() = imu.orientation.w;
                
                vmc.robot_ang_vel[0] = imu.angular_velocity.x;
                vmc.robot_ang_vel[1] = imu.angular_velocity.y;
                vmc.robot_ang_vel[2] = imu.angular_velocity.z;

                quaternion_to_euler(vmc.robot_ang, vmc.roll, vmc.pitch, vmc.yaw);
                if (!sim) {
                    vmc.pitch *= -1;
                    vmc.yaw *= -1;
                }

                // Build state vector (same 13D layout as MPC for consistency)
                Eigen::VectorXd x(13);
                x << vmc.roll,             vmc.pitch,            vmc.yaw,
                     vmc.robot_pos[0],     vmc.robot_pos[1],     vmc.robot_pos[2],
                     vmc.robot_ang_vel[0], vmc.robot_ang_vel[1], vmc.robot_ang_vel[2],
                     vmc.robot_vel[0],     vmc.robot_vel[1],     vmc.robot_vel[2],
                     -vmc.gravity;

                // Reference: single step (VMC uses only one reference, not N-step horizon)
                Eigen::VectorXd x_ref(13);
                x_ref << 0,                  0,                  0,
                         vmc.target_pos_x,   0,                  vmc.target_pos_z,
                         0,                  0,                  0,
                         vmc.target_vel_x,   0,                  0,
                         -vmc.gravity;

                // Compute foot positions from motor state
                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
                double ra[3] = {-legmodel.contact_p[0]+0.222, 0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
                double rb[3] = {legmodel.contact_p[0]+0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
                double rc[3] = {legmodel.contact_p[0]-0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
                double rd[3] = {-legmodel.contact_p[0]-0.222, 0.2, legmodel.contact_p[1]};

                // VMC step: compute wrench -> distribute forces
                Eigen::VectorXd force = vmc.step(x, x_ref, selection_matrix, ra, rb, rc, rd);

                double force_A[3] = {force(0), force(1), force(2)};
                double force_B[3] = {force(3), force(4), force(5)};
                double force_C[3] = {force(6), force(7), force(8)};
                double force_D[3] = {force(9), force(10), force(11)};

                Eigen::Matrix3d R_T = vmc.robot_ang.toRotationMatrix().transpose();
                if (!sim) {
                    R_T(1, 2) *= -1;
                    R_T(2, 1) *= -1;
                }
                
                convert_force_to_local(force_A, R_T);
                convert_force_to_local(force_B, R_T);
                convert_force_to_local(force_C, R_T);
                convert_force_to_local(force_D, R_T);

                imp_cmd_modules[0]->fx = -force_A[0];
                imp_cmd_modules[0]->fy = -force_A[2];
                imp_cmd_modules[1]->fx =  force_B[0];
                imp_cmd_modules[1]->fy = -force_B[2];
                imp_cmd_modules[2]->fx =  force_C[0];
                imp_cmd_modules[2]->fy = -force_C[2];
                imp_cmd_modules[3]->fx = -force_D[0];
                imp_cmd_modules[3]->fy = -force_D[2];

                imp_cmd.header.stamp = node->now();
                imp_cmd_pub->publish(imp_cmd);

                contact_state.header.stamp = node->now();

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Ref Pos = [" << x_ref[3] << ", " << x_ref[4] << ", " << x_ref[5] << "]" << std::endl << std::endl;
                std::cout << "Odom Pos = [" << vmc.robot_pos[0] << ", " << vmc.robot_pos[1] << ", " << vmc.robot_pos[2] << "]" << std::endl;
                std::cout << "Odom Vel = [" << vmc.robot_vel[0] << ", " << vmc.robot_vel[1] << ", " << vmc.robot_vel[2] << "]" << std::endl;
                std::cout << "Odom Ang (deg) = [" << vmc.roll/M_PI*180 << ", " << vmc.pitch/M_PI*180 << ", " << vmc.yaw/M_PI*180 << "]" << std::endl << std::endl;

                std::cout << "Force A: [" << force_A[0] << ", " << force_A[2] << "]" << std::endl;
                std::cout << "Force B: [" << force_B[0] << ", " << force_B[2] << "]" << std::endl;
                std::cout << "Force C: [" << force_C[0] << ", " << force_C[2] << "]" << std::endl;
                std::cout << "Force D: [" << force_D[0] << ", " << force_D[2] << "]" << std::endl;

                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                if (loop_count >= vmc.target_loop) std::cout << "Finished" << std::endl;

                next_time += period;
                if(!node->get_clock()->sleep_until(next_time)){
                    RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                    break;
                }
            }
            break;
        }
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }
    return 0;
}
