#ifndef SIMPLE_FSM_HPP
#define SIMPLE_FSM_HPP


#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"

#include "corgi_msgs/FsmCmdStamped.h"
#include "corgi_msgs/FsmStateStamped.h"

#include "corgi_msgs/TriggerStamped.h"


enum class Gait {
    WHEEL, //1
    LEG,   //2
    WLW,   //3
    TRANSFORM
};

#endif