#include "corgi_mpc.hpp"


bool trigger = false;
geometry_msgs::Vector3 odom_pos;
geometry_msgs::Vector3 odom_vel;
sensor_msgs::Imu imu;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
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


int main(int argc, char **argv) {
    ROS_INFO("Corgi MPC Starts");

    ros::init(argc, argv, "corgi_mpc");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Subscriber odom_pos_sub = nh.subscribe<geometry_msgs::Vector3>("odometry/position", 1000, odom_pos_cb);
    ros::Subscriber odom_vel_sub = nh.subscribe<geometry_msgs::Vector3>("odometry/velocity", 1000, odom_vel_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1000, imu_cb);

    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    double M = 0;
    double K = 1000;
    double B = 30;

    for (int i=0; i<2000; i++){
        for (auto& cmd : imp_cmd_modules){
            cmd->theta = 18/180.0*M_PI;
            cmd->beta = 0/180.0*M_PI;

            cmd->Mx = M;
            cmd->My = M;
            cmd->Bx = B;
            cmd->By = B;
            cmd->Kx = K;
            cmd->Ky = K;

            cmd->Fy = -55;
        }

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }

    ROS_INFO("Transform Finished\n");

    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        if (trigger){
            int seq = 0;
            while (ros::ok()) {
                ros::spinOnce();

                std::cout << "Odom Pos = [" << odom_pos.x << ", " << odom_pos.y << ", " << odom_pos.z << "]" << std::endl;
                std::cout << "Odom Vel = [" << odom_vel.x << ", " << odom_vel.y << ", " << odom_vel.z << "]" << std::endl;
                std::cout << "Imu = [" << imu.orientation.x << ", " << imu.orientation.y << ", " << imu.orientation.z << "," << imu.orientation.w << "]" << std::endl << std::endl;


                if (seq < 1000) {
                    for (int i=0; i<4; i++) {
                        imp_cmd_modules[i]->theta += 43/1000.0/180.0*M_PI;
                    }
                }



                imp_cmd.header.seq = seq;
                imp_cmd_pub.publish(imp_cmd);

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
