#include "Simple_fsm.hpp"

GaitSelector::GaitSelector( ros::NodeHandle& nh, 
                            bool sim, 
                            double CoM_bias, 
                            int pub_rate, 
                            double BL, 
                            double BW, 
                            double BH): 
    leg_model(sim), 
    CoM_bias(CoM_bias), 
    BL(BL), 
    BW(BW), 
    BH(BH), 
    pub_rate(pub_rate),
    rng(rd()), 
    dist(0, 359),
    currentGait(Gait::WHEEL)
{
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &GaitSelector::motor_state_cb, this);
    motor_cmd_pub_ = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", pub_rate);
    rate_ptr = new ros::Rate(pub_rate);
}

GaitSelector::~GaitSelector() {
    delete rate_ptr;
    rate_ptr = nullptr;
}

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


    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    return 0;
}

/*    Initialize of each mode   */ 

/*     Gait Selector Setting    */ 
