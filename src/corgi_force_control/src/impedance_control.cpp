#include "force_estimation.hpp"

corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::ForceStateStamped force_state;
corgi_msgs::ImpedanceCmdStamped imp_cmd;
corgi_msgs::MotorCmdStamped motor_cmd;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}

void imp_cmd_cb(const corgi_msgs::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

Eigen::MatrixXd impedance_control(const corgi_msgs::ImpedanceCmd* imp_cmd_, const corgi_msgs::MotorState* motor_state_,
                                  const corgi_msgs::ForceState* force_state_, Eigen::MatrixXd& pos_err_hist,
                                  Eigen::MatrixXd& force_err_hist, Eigen::MatrixXd& force_state_hist){
    
    Eigen::MatrixXd trq_cmd(2, 1);

    Eigen::MatrixXd pos_err(2, 1);
    Eigen::MatrixXd pos_des(2, 1);
    Eigen::MatrixXd force_des(2, 1);
    Eigen::MatrixXd force_cmd(2, 1);
    Eigen::MatrixXd force_err(2, 1);

    legmodel.contact_map(imp_cmd_->theta, imp_cmd_->beta);
    pos_des << legmodel.contact_p[0], legmodel.contact_p[1];

    int target_rim = legmodel.rim;

    legmodel.contact_map(motor_state_->theta, motor_state_->beta);
    legmodel.forward(motor_state_->theta, motor_state_->beta);

    switch (target_rim)
    {
    case 1:
        pos_err << pos_des(0, 0)-legmodel.U_l[0], pos_des(1, 0)-legmodel.U_l[1]+legmodel.radius;
        break;
    case 2:
        pos_err << pos_des(0, 0)-legmodel.L_l[0], pos_des(1, 0)-legmodel.L_l[1]+legmodel.radius;
        break;
    case 3:
        pos_err << pos_des(0, 0)-legmodel.G[0], pos_des(1, 0)-legmodel.G[1]+legmodel.r;
        break;
    case 4:
        pos_err << pos_des(0, 0)-legmodel.L_r[0], pos_des(1, 0)-legmodel.L_r[1]+legmodel.radius;
        break;
    case 5:
        pos_err << pos_des(0, 0)-legmodel.U_r[0], pos_des(1, 0)-legmodel.U_r[1]+legmodel.radius;
        break;
    default:
        pos_err << 0, 0;
        break;
    }

    force_des << imp_cmd_->Fx, imp_cmd_->Fy;


    double T = 0.001;

    Eigen::MatrixXd M(2, 2);
    Eigen::MatrixXd B(2, 2);
    Eigen::MatrixXd K(2, 2);

    M << imp_cmd_->Mx, 0, 0, imp_cmd_->My;
    B << imp_cmd_->Bx, 0, 0, imp_cmd_->By;
    K << imp_cmd_->Kx, 0, 0, imp_cmd_->Ky;

    force_cmd =  force_des + M * (pos_err-2*pos_err_hist.col(0)+pos_err_hist.col(1))/T/T + B * (pos_err-pos_err_hist.col(0))/T + K*pos_err;

    std::cout << "pos_des: " << pos_des(0, 0) << ", " << pos_des(1, 0) << std::endl << std::endl;
    std::cout << "pos_state: " << legmodel.contact_p[0] << ", " << legmodel.contact_p[1] << std::endl << std::endl;
    std::cout << "pos_err: " << pos_err(0, 0) << ", " << pos_err(1, 0) << std::endl << std::endl;
    std::cout << "force_des: " << force_des(0, 0) << ", " << force_des(1, 0) << std::endl << std::endl;
    std::cout << "force_state: " << force_state_->Fx << ", " << force_state_->Fy << std::endl << std::endl;
    std::cout << "force_cmd: " << force_cmd(0, 0) << ", " << force_cmd(1, 0) << std::endl << std::endl;

    legmodel.contact_map(motor_state_->theta, motor_state_->beta);
    pos_err_hist.col(1) = pos_err_hist.col(0);
    pos_err_hist.col(0) = pos_err;
    force_err_hist.col(1) = force_err_hist.col(0);
    force_err_hist.col(0) = force_err;


    Eigen::MatrixXd P_poly = calculate_P_poly(legmodel.rim, legmodel.alpha);
    Eigen::MatrixXd P_poly_deriv(2, 7);

    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(motor_state_->theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(motor_state_->theta, i); 
    
    Eigen::MatrixXd jacobian(2, 2);
    jacobian = calculate_jacobian(P_theta, P_theta_deriv, motor_state_->beta);
    
    trq_cmd = jacobian.transpose() * force_cmd;

    std::cout << "trq_cmd: " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl << std::endl;

    return trq_cmd;
}


int main(int argc, char **argv) {

    ROS_INFO("Impedance Control Starts\n");

    ros::init(argc, argv, "impedance_control");

    ros::NodeHandle nh;
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Subscriber imp_cmd_sub = nh.subscribe<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Rate rate(1000);

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<Eigen::MatrixXd> pos_err_hist_modules = {
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2)
    };

    std::vector<Eigen::MatrixXd> force_err_hist_modules = {
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2)
    };

    std::vector<Eigen::MatrixXd> force_state_hist_modules = {
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1)
    };

    Eigen::MatrixXd trq_cmd = Eigen::MatrixXd::Zero(2, 1);

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        for (int i=0; i<1; i++){
            trq_cmd = impedance_control(imp_cmd_modules[i], motor_state_modules[i], force_state_modules[i],
                                        pos_err_hist_modules[i], force_err_hist_modules[i], force_state_hist_modules[i]);

            motor_cmd_modules[i]->kp_r = 0;
            motor_cmd_modules[i]->kp_l = 0;
            motor_cmd_modules[i]->kd_r = 0;
            motor_cmd_modules[i]->kd_l = 0;
            
            motor_cmd_modules[i]->theta = 1;
            motor_cmd_modules[i]->beta = 0;

            motor_cmd_modules[i]->torque_r = trq_cmd(0, 0);
            motor_cmd_modules[i]->torque_l = trq_cmd(1, 0);
        }

        std::cout << "= = = = =" << std::endl << std::endl;

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = ros::Time::now();
        
        motor_cmd_pub.publish(motor_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}