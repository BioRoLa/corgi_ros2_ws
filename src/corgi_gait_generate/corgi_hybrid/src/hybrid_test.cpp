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

    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    
    /*    Initialize of each mode   */ 
    // hybrid mode 
    Hybrid hybrid(nh);   

    /*  wlw initial pose  */
    std::cout << "hybrid" << std::endl;
    // hybrid.Initialize(2, 300, 0, 0, 5, 2, 0);
    hybrid.Initialize(2, 10, 1, 0, 5, 2, -0.03);
    /*  wlw real-time   */
    cout<< "-----wlw------"<<endl;
    cout<< "walking"<<endl;
    for (int step = 0;step<5000;step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        hybrid.Step(1, 1, -0.03);
    }

    cout<< "----wlw-variation----"<<endl;
    cout<< "Height from 0.14 to 0.165"<<endl;
    for (int step = 0;step<10000;step++) {
        // Test change_Height
        if(step <= 8000){
            hybrid.change_Height(0.14 + 0.025*(step)/8000);
        }
       
       
        // Test change_Step_length
        if (step >= 5000){
            if(step == 5000){
                cout<< "SL from 0.4 to 0.3"<<endl;
                cout<< "V from 0.08 to 0.05"<<endl;
            }
            hybrid.change_Step_length(0.3);
            hybrid.change_Velocity(0.05);
            gaitSelector.motor_cmd.header.seq = step;
            gaitSelector.motor_cmd.header.stamp = ros::Time::now();
            hybrid.Step(1, 1, -0.03);
        }
        else{
            gaitSelector.motor_cmd.header.seq = step;
            gaitSelector.motor_cmd.header.stamp = ros::Time::now();
            hybrid.Step(1, 1, -0.03);
        }
        
        
    }
    return 0;
}


// static = share the variable
// but need to be set value outside the class 