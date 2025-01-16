#ifndef WHEELED_HPP
#define WHEELED_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <std_msgs/String.h>


#include <corgi_msgs/WheelCmd.h>
#include <corgi_msgs/MotorState.h>
#include <corgi_msgs/MotorStateStamped.h>
#include <corgi_msgs/MotorCmd.h>
#include <corgi_msgs/MotorCmdStamped.h>

class Wheeled
{
public:
    Wheeled();
    ~Wheeled() = default;

private:
    void wheelCmdCallback(const corgi_msgs::WheelCmd::ConstPtr& msg);
    void motorsStateCallback(const corgi_msgs::MotorStateStamped::ConstPtr& msg);

    ros::NodeHandle wnh_;
    ros::Publisher  motor_cmd_pub_;
    ros::Subscriber wheel_cmd_sub_;
    ros::Subscriber motor_state_sub_;

    corgi_msgs::MotorStateStamped current_motor_state_;
};


#endif