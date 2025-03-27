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

