#include "admittance_control.hpp"


LegModel legmodel(true);

corgi_msgs::ForceStateStamped force_state;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::MotorCmdStamped motor_cmd;


void force_state_cb(const corgi_msgs::ForceStateStamped state){
    force_state = state;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

Eigen::MatrixXd admittance_control(Eigen::MatrixXd eta_cmd, Eigen::MatrixXd force){
    
    Eigen::MatrixXd eta;

    return eta;
}


int main(int argc, char **argv) {

    ROS_INFO("Admittance Control Starts\n");

    ros::init(argc, argv, "admittance_control");

    ros::NodeHandle nh;
    ros::Subscriber force_state_sub = nh.subscribe<corgi_msgs::ForceStateStamped>("force/state", 1000, force_state_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Rate rate(1000);


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

    std::vector<Eigen::MatrixXd> force_cmd_modules = { Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1) };
    std::vector<Eigen::MatrixXd> eta_cmd_modules = { Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1), Eigen::MatrixXd::Zero(2, 1) };
    std::vector<Eigen::MatrixXd> P_err_modules = { Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Zero(2, 3) };
    std::vector<Eigen::MatrixXd> F_err_modules = { Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Zero(2, 3) };

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        for (int i=0; i<4; i++){
            legmodel.contact_map(motor_state_modules[i].theta, motor_state_modules[i].beta);
            Eigen::MatrixXd pos_fb(2, 1);
            pos_fb << legmodel.contact_p[0], legmodel.contact_p[1];

            legmodel.contact_map(eta_cmd_modules[i](0, 0), eta_cmd_modules[i](1, 0));
            Eigen::MatrixXd pos_cmd(2, 1);
            pos_cmd << legmodel.contact_p[0], legmodel.contact_p[1];
            
            P_err_modules.block(1, 0, 2, 2) = P_err_modules.block(0, 0, 2, 2);
            P_err_modules.block(0, 0, 2, 1) = pos_cmd - pos_fb;

            F_err_modules.block(1, 0, 2, 2) = F_err_modules.block(0, 0, 2, 2);
            F_err_modules.block(1, 0, 2, 1) << force_cmd_modules[i](0, 0) - force_state_modules[i].Fx, force_cmd_modules[i](1, 0) - force_state_modules[i].Fy,;
            
            // torque_cmd_modules[i] = calculate_torque(eta_cmd_modules[i], force_cmd_modules[i]);
            
            eta_cmd_modules[i] = admittance_control(P_err_modules[i], F_err_modules[i], )

            motor_cmd_modules[i]->kp = 90;
            motor_cmd_modules[i]->kd = 1.75;
            motor_cmd_modules[i]->theta = eta_cmd_modules[i](0, 0);
            motor_cmd_modules[i]->beta = eta_cmd_modules[i](1, 0);
            motor_cmd_modules[i]->torque_r = torque_cmd_modules[i](0, 0);
            motor_cmd_modules[i]->torque_l = torque_cmd_modules[i](1, 0);
        }

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = ros::Time::now();
        
        motor_cmd_pub.publish(motor_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}