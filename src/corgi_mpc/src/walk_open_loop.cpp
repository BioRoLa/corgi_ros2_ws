#include "force_estimation.hpp"
#include "mpc.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    ROS_INFO("Corgi Walk Starts");

    ModelPredictiveController mpc;

    ros::init(argc, argv, "corgi_walk");

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
    double init_eta[8] = {1.9427780616992942,0.4926850881734512,1.6666945447888537,0.133413303886169,1.6666945447888537,-0.133413303886169,1.9427780616992942,-0.4926850881734512};
    WalkGait walk_gait(sim, 0, 1000);
    walk_gait.initialize(init_eta);

    double velocity        = 0.1;
    double stand_height    = 0.25;
    double step_length     = 0.3;
    double step_height     = 0.1;
    
    walk_gait.set_velocity(mpc.target_vel_x);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);


    // initialize impedance command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 150;
        cmd->kp_l = 150;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        cmd->kd_r = 1.75;
        cmd->kd_l = 1.75;
    }

    ROS_INFO("Wait ...\n");
    
    for (int i=0; i<3000; i++) {
        rate.sleep();
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
    for (int i=0; i<1000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();
        if (trigger){
            ROS_INFO("Wait For Odometry Node Initializing ...\n");

            // wait for odometry node
            for (int i=0; i<5000; i++) {
                rate.sleep();
            }

            ROS_INFO("Controller Starts ...\n");

            int loop_count = 0;
            while (ros::ok()) {
                ros::spinOnce();

                // update target vel and pos
                if (loop_count < 3000) {
                    mpc.target_vel_x += velocity/3000.0;
                    walk_gait.set_velocity(mpc.target_vel_x);
                }
                if (loop_count > mpc.target_loop*10-3000) {
                    mpc.target_vel_x -= velocity/3000.0;
                    walk_gait.set_velocity(mpc.target_vel_x);
                }

                // mpc.target_vel_x = velocity;
                // walk_gait.set_velocity(mpc.target_vel_x);

                mpc.target_pos_x += mpc.target_vel_x * mpc.dt / 10.0;

                // get next eta
                mpc.eta_list = walk_gait.step();

                for (int i=0; i<4; i++) {
                    if (mpc.eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (mpc.eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = mpc.eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? mpc.eta_list[1][i] : -mpc.eta_list[1][i];
                }

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Target Position X: " << mpc.target_pos_x << std::endl << std::endl;
                std::cout << "Current Velocity X: " << mpc.target_vel_x << std::endl << std::endl;
                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                if (loop_count == mpc.target_loop*10) break;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}