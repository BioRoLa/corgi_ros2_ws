#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"

#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"

bool trigger = false;

void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_csv_control");

    ros::NodeHandle nh;
    ros::Publisher imp_cmd_pub = nh.advertise<corgi_msgs::ImpedanceCmdStamped>("impedance/command", 1000);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1000, trigger_cb);
    ros::Rate rate(1000);

    corgi_msgs::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::ImpedanceCmd*> imp_cmds = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    double M = 0;
    double K = 1000;
    double B = 30;

    if (argc < 2){
        ROS_INFO("Please input csv file path\n");
        return 1;
    }
    
    std::string csv_file_path;
    csv_file_path = std::getenv("HOME");
    csv_file_path += "/corgi_ws/corgi_ros_ws/input_csv/";
    csv_file_path += argv[1];
    csv_file_path += ".csv";
    

    std::ifstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        ROS_INFO("Failed to open the CSV file\n");
        return 1;
    }

    std::string line;

    ROS_INFO("Leg Transform Starts\n");
    
    for (int i=0; i<5000; i++){
        std::getline(csv_file, line);
        std::vector<double> columns;
        std::stringstream ss(line);
        std::string item;
        
        for (auto& cmd : imp_cmds){
            std::getline(ss, item, ',');
            cmd->theta = std::stod(item);

            std::getline(ss, item, ',');
            cmd->beta = std::stod(item);

            cmd->Fy = -55;

            cmd->Mx = M;
            cmd->My = M;
            cmd->Bx = B;
            cmd->By = B;
            cmd->Kx = K;
            cmd->Ky = K;
        }

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }

    ROS_INFO("Leg Transform Finished\n");
    
    while (ros::ok()){
        ros::spinOnce();

        if (trigger){
            ROS_INFO("CSV Trajectory Starts\n");

            int seq = 0;
            while (ros::ok() && std::getline(csv_file, line)) {
                std::vector<double> columns;
                std::stringstream ss(line);
                std::string item;
                
                for (auto& cmd : imp_cmds){
                    std::getline(ss, item, ',');
                    cmd->theta = std::stod(item);

                    std::getline(ss, item, ',');
                    cmd->beta = std::stod(item);

                    cmd->Mx = M;
                    cmd->My = M;
                    cmd->Bx = B;
                    cmd->By = B;
                    cmd->Kx = K;
                    cmd->Ky = K;
                    cmd->Fy = -55;
                }

                imp_cmd.header.seq = seq;

                imp_cmd_pub.publish(imp_cmd);

                seq++;

                rate.sleep();
            }
            break;
        }
    }

    ROS_INFO("CSV Trajectory Finished\n");

    ros::shutdown();
    
    return 0;
}
