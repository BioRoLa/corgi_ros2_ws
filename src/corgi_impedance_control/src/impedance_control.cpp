#include "impedance_control.hpp"


AdmittanceController::AdmittanceController() : dt(0.0001), legmodel(true) {

    M = Eigen::MatrixXd::Zero(2, 2);
    K = Eigen::MatrixXd::Zero(2, 2);
    D = Eigen::MatrixXd::Zero(2, 2);

    pos_fb = Eigen::MatrixXd::Zero(2, 1);
    vel_fb = Eigen::MatrixXd::Zero(2, 1);
    pos_ref = Eigen::MatrixXd::Zero(2, 1);
    vel_ref = Eigen::MatrixXd::Zero(2, 1);

    eta_cmd = Eigen::MatrixXd::Zero(2, 1);
}


void AdmittanceController::update(const Eigen::MatrixXd& eta_fb, const Eigen::MatrixXd& eta_ref,
                                  const Eigen::MatrixXd& force_fb, const Eigen::MatrixXd& force_ref,
                                  const Eigen::MatrixXd& pos_fb_prev, const Eigen::MatrixXd& pos_ref_prev) {
    
    Eigen::MatrixXd H_l(2, 1);
    Eigen::MatrixXd U_l(2, 1);
    Eigen::MatrixXd F_l(2, 1);
    Eigen::MatrixXd L_l(2, 1);
    Eigen::MatrixXd H_r(2, 1);
    Eigen::MatrixXd U_r(2, 1);
    Eigen::MatrixXd F_r(2, 1);
    Eigen::MatrixXd L_r(2, 1);
    Eigen::MatrixXd G(2, 1);
    
    legmodel.contact_map(eta_fb(0, 0), eta_fb(1, 0));
    legmodel.forward(eta_fb(0, 0), eta_fb(1, 0));

    Eigen::Rotation2D<double> rotation(legmodel.alpha);
    Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();

    H_l << legmodel.H_l[0], legmodel.H_l[1];
    U_l << legmodel.U_l[0], legmodel.U_l[1];
    F_l << legmodel.F_l[0], legmodel.F_l[1];
    L_l << legmodel.L_l[0], legmodel.L_l[1];
    H_r << legmodel.H_r[0], legmodel.H_r[1];
    U_r << legmodel.U_r[0], legmodel.U_r[1];
    F_r << legmodel.F_r[0], legmodel.F_r[1];
    L_r << legmodel.L_r[0], legmodel.L_r[1];
    G << legmodel.G[0], legmodel.G[1];

    if (legmodel.rim == 1) pos_fb = rot_alpha * (H_l-U_l) + U_l;
    else if (legmodel.rim == 2) pos_fb = rot_alpha * (F_l-L_l) + L_l;
    else if (legmodel.rim == 3) pos_fb = G;
    else if (legmodel.rim == 4) pos_fb = rot_alpha * (G-L_r) + L_r;
    else if (legmodel.rim == 5) pos_fb = rot_alpha * (F_r-U_r) + U_r;
    else eta_cmd << eta_ref(0, 0), eta_ref(1, 0);


    legmodel.contact_map(eta_ref(0, 0), eta_ref(1, 0));
    legmodel.forward(eta_ref(0, 0), eta_ref(1, 0));

    Eigen::Rotation2D<double> rotation_(legmodel.alpha);
    Eigen::Matrix2d rot_alpha_ = rotation_.toRotationMatrix();

    H_l << legmodel.H_l[0], legmodel.H_l[1];
    U_l << legmodel.U_l[0], legmodel.U_l[1];
    F_l << legmodel.F_l[0], legmodel.F_l[1];
    L_l << legmodel.L_l[0], legmodel.L_l[1];
    H_r << legmodel.H_r[0], legmodel.H_r[1];
    U_r << legmodel.U_r[0], legmodel.U_r[1];
    F_r << legmodel.F_r[0], legmodel.F_r[1];
    L_r << legmodel.L_r[0], legmodel.L_r[1];
    G << legmodel.G[0], legmodel.G[1];

    if (legmodel.rim == 1) pos_ref = rot_alpha_ * (H_l-U_l) + U_l;
    else if (legmodel.rim == 2) pos_ref = rot_alpha_ * (F_l-L_l) + L_l;
    else if (legmodel.rim == 3) pos_ref = G;
    else if (legmodel.rim == 4) pos_ref = rot_alpha_ * (G-L_r) + L_r;
    else if (legmodel.rim == 5) pos_ref = rot_alpha_ * (F_r-U_r) + U_r;
    else eta_cmd << eta_ref(0, 0), eta_ref(1, 0);

    vel_fb = (pos_fb_prev - pos_fb) / dt;
    vel_ref = (pos_ref_prev - pos_ref) / dt;

    pos_cmd = pos_fb + dt * (vel_fb + dt*M.inverse()*((force_ref-force_fb) - D*(vel_ref-vel_fb) - K*(pos_ref-pos_fb)));

    std::cout << pos_ref(0, 0) << ", " << pos_ref(1, 0) << std::endl;
    std::cout << pos_cmd(0, 0) << ", " << pos_cmd(1, 0) << std::endl;

    eta_cmd << eta_ref(0, 0), eta_ref(1, 0);
}


void imp_cmd_cb(const corgi_msgs::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}


int main(int argc, char **argv) {

    ROS_INFO("Impedance Control Starts\n");

    ros::init(argc, argv, "impedance_control");

    ros::NodeHandle nh;
    ros::Subscriber imp_cmd_sub = nh.subscribe<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    
    ros::Rate rate(1000);


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

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };


    AdmittanceController admittance_controller;
    
    pos_fb_prev_modules = {Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1)};
    pos_ref_prev_modules = {Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1)};

    while (ros::ok()) {
        ros::spinOnce();

        for (int i=0; i<4; i++){
            printf("-\n");
            printf("imp_cmd: eta = [%lf, %lf], force = [%lf, %lf]\n", imp_cmd_modules[i]->theta, imp_cmd_modules[i]->beta, imp_cmd_modules[i]->Fx, imp_cmd_modules[i]->Fy);
            printf("motor_state: eta = [%lf, %lf]\n", motor_state_modules[i]->theta, motor_state_modules[i]->beta);
            printf("force_state: force = [%lf, %lf]\n", force_state_modules[i]->Fx, force_state_modules[i]->Fy);

            eta_fb << motor_state_modules[i]->theta, motor_state_modules[i]->beta;
            eta_ref << imp_cmd_modules[i]->theta, imp_cmd_modules[i]->beta;
            force_fb << force_state_modules[i]->Fx, force_state_modules[i]->Fy;
            force_ref << imp_cmd_modules[i]->Fx, imp_cmd_modules[i]->Fy;

            admittance_controller.M << imp_cmd_modules[i]->Mx, 0, 0, imp_cmd_modules[i]->My;
            admittance_controller.K << imp_cmd_modules[i]->Kx, 0, 0, imp_cmd_modules[i]->Ky;
            admittance_controller.D << imp_cmd_modules[i]->Dx, 0, 0, imp_cmd_modules[i]->Dy;

            admittance_controller.update(eta_fb, eta_ref, force_fb, force_ref, pos_fb_prev_modules[i], pos_ref_prev_modules[i]);

            pos_fb_prev_modules[i] = admittance_controller.pos_fb;
            pos_ref_prev_modules[i] = admittance_controller.pos_ref;

            motor_cmd_modules[i]->theta = admittance_controller.eta_cmd(0, 0);
            motor_cmd_modules[i]->beta = admittance_controller.eta_cmd(1, 0);
        }

        printf("\n");

        motor_cmd_pub.publish(motor_cmd);
        
        rate.sleep();
    }

    ros::shutdown();

    return 0;
}