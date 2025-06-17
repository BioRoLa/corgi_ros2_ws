#include "hybrid_gen.hpp"

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
    ROS_INFO("Hybrid mode test\n");
    ros::init(argc, argv, "corgi_hybrid_test");
    ros::NodeHandle nh;

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*     Gait Selector Setting    */ 
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    auto gaitSelector = std::make_shared<GaitSelector>(nh, sim, CoM_bias, pub_rate);
    std::cout << "gaitSelector initialized" << std::endl;
    /*    Initialize of each mode   */ 
    // hybrid mode 
    Hybrid hybrid(gaitSelector); 

    /*  wlw initial pose  */
    std::cout << "hybrid" << std::endl;
    std::ofstream file("total_info.csv", std::ios::app);
    hybrid.Initialize(1, 1);
    // hybrid.csv_title(file); 
    // hybrid.save_to_csv(file, 0); // Pass file by reference
    // // /*  wlw real-time   */
    // // cout<< "-----wlw------"<<endl;

   for (int step = 0;step<100;step++) {
         // Send in swing_desired_height
        gaitSelector->motor_cmd.header.seq = step;
        gaitSelector->motor_cmd.header.stamp = ros::Time::now();
        hybrid.Step();
        hybrid.save_to_csv(file, step+1);
    }

  



    // cout<< "----wlw-variation----"<<endl;
    // cout<< "Height from 0.14 to 0.165"<<endl;
    // for (int step = 0;step<10000;step++) {
    //     // Test change_Height
    //     if(step <= 8000){
    //         hybrid.change_Height(0.14 + 0.025*(step)/8000);
    //     }
       
       
    //     // Test change_Step_length
    //     if (step >= 5000){
    //         if(step == 5000){
    //             cout<< "SL from 0.4 to 0.3"<<endl;
    //             cout<< "V from 0.08 to 0.05"<<endl;
    //         }
    //         hybrid.change_Step_length(0.3);
    //         hybrid.change_Velocity(0.05);
    //         gaitSelector.motor_cmd.header.seq = step;
    //         gaitSelector.motor_cmd.header.stamp = ros::Time::now();
    //         hybrid.Step(1, 1, -0.03);
    //     }
    //     else{
    //         gaitSelector.motor_cmd.header.seq = step;
    //         gaitSelector.motor_cmd.header.stamp = ros::Time::now();
    //         hybrid.Step(1, 1, -0.03);
    //     }
        
        
    // }
    file.close();
    return 0;
}


// static = share the variable
// but need to be set value outside the class 


// SwingType swing_type = SwingType::LINEAR;

// SwingType::LINEAR
// SwingType::CUBIC
// SwingType::QUINTIC
// SwingType::PARABOLIC
// SwingType::BEZIER
// SwingType::SINUSOIDAL