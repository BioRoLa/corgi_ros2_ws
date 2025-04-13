#include "wheeled_gen.hpp"
// single execute wheeled mode

int main(int argc, char **argv){
    ROS_INFO("Wheeled mode test\n");
    ros::init(argc, argv, "corgi_wheeled_test");
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
    // Wheeled mode
    WheeledCmd WheeledCmd("joystick");
    Wheeled wheeled(nh);

    while (ros::ok()) {
        // ros::spinOnce();
    }
    return 0;
}

