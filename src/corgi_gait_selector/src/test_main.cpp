#include "Simple_fsm.hpp"


int main(int argc, char **argv){
    ROS_INFO("Simple FSM Control\n");
    ros::init(argc, argv, "corgi_gait_selector");
    ros::NodeHandle nh;

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();    

    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;

    LegModel leg_model(sim);
    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    return 0;
}

/*    Initialize of each mode   */ 

/*     Gait Selector Setting    */ 
