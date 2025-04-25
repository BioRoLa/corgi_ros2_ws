#include "force_estimation.hpp"
#include "hybrid_gen.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    ROS_INFO("Corgi WLW Starts");

    ros::init(argc, argv, "corgi_wlw");

    ros::NodeHandle nh;
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    
    ros::Rate rate(1000);

    corgi_msgs::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    // initialize gait
    double CoM_bias = 0.0;

    GaitSelector gait_selector(nh, sim, CoM_bias, 1000);
    Hybrid hybrid_gait(nh);   

    std::cout << "hybrid" << std::endl;
    double velocity = 0.1;
    gait_selector.stand_height = 0.16;
    gait_selector.step_length = 0.3;
    hybrid_gait.Initialize(0, 0, 0, 0, 0, 0, -0.03);
    
    double init_eta[8];
    for (int i=0; i<4; i++) {
        init_eta[2*i] = gait_selector.eta[i][0];
        init_eta[2*i+1] = gait_selector.eta[i][1];
    }

    init_eta[3] *= -1;
    init_eta[5] *= -1;
    
    std::array<std::array<double, 4>, 2> eta_list = {{{17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI}, {0, 0, 0, 0}}};
    int target_loop = 20000;

    // initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 150;
        cmd->kp_l = 150;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 0.75;
            cmd->kd_l = 0.75;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    ROS_INFO("Transform Starts\n");

    // transform
    for (int i=0; i<3000; i++) {
        for (int j=0; j<4; j++) {
            motor_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/3000.0;
            motor_cmd_modules[j]->beta += init_eta[2*j+1]/3000.0;
        }
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    ROS_INFO("Transform Finished\n");

    // stay
    for (int i=0; i<2000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();
        if (trigger){
            ROS_INFO("Controller Starts ...\n");

            int loop_count = 0;
            while (ros::ok()) {
                ros::spinOnce();

                if (loop_count > target_loop-3000 && loop_count < target_loop) {
                    velocity -= 0.1/3000.0;
                    hybrid_gait.change_Velocity(velocity);
                }

                // get next eta
                hybrid_gait.Step(0, 0, -0.03);

                for (int i=0; i<4; i++) {
                    eta_list[0][i] = gait_selector.eta[i][0];
                    eta_list[1][i] = gait_selector.eta[i][1];
                }


                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 0 || i == 3) ? eta_list[1][i] : -eta_list[1][i];
                }

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                loop_count++;
                if (loop_count >= target_loop) break;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}