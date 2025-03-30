#include "Simple_fsm.hpp"
#include "wheeled_gen.hpp"

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

    /*     Gait Selector Setting    */ 
    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    // Start the keyboard input thread
    std::thread input_thread(&GaitSelector::keyboardInputThread, &gaitSelector);
    
    /*    Initialize of each mode   */ 
    // Wheeled mode
    WheeledCmd WheeledCmd;
    Wheeled wheeled(nh);
    // add methos to avoid sending cmd when is not in wheel mode

    // Legged mode
    // HybridCmd mode
    // Transform mode

    /*    main loop   */ 
    while (ros::ok()) {
        // gaitSelector.Send(1);
        ros::spinOnce();
    }

    // Join the input thread before exiting
    if (input_thread.joinable()) {
        input_thread.join();
    }
    return 0;
}





