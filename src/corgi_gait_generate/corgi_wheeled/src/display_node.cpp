#include <ros/ros.h>
#include <std_msgs/String.h>
#include <corgi_msgs/SteeringStateStamped.h>
#include <corgi_msgs/SteeringCmdStamped.h>
#include <corgi_msgs/WheelCmd.h>

const std::string reset("\033[1;0m");
const std::string black("\033[1;30m");
const std::string red("\033[1;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string blue("\033[1;34m");
const std::string magenta("\033[1;35m");
const std::string cyan("\033[1;36m");
const std::string white("\033[1;37m");


// 1) For general debug/log strings from joystick_control
void debugInfoCallback(const std_msgs::String::ConstPtr& msg)
{
  // Print debug/log messages
  ROS_INFO_STREAM("[DEBUG] " << msg->data << reset);
}

// 2) For SteeringCmd messages
void steeringCmdCallback(const corgi_msgs::SteeringCmdStamped::ConstPtr& msg)
{
  ROS_INFO_STREAM("SteeringCmd => angle=" << msg->angle
                  << ", voltage=" << msg->voltage << green);
}

// 3) For WheelCmd messages
void wheelCmdCallback(const corgi_msgs::WheelCmd::ConstPtr& msg)
{
  ROS_INFO_STREAM("WheelCmd => direction=" << (msg->direction ? "FWD" : "BWD")
                  << ", stop=" << (msg->stop ? "TRUE" : "FALSE")
                  << ", velocity=" << msg->velocity << blue);
}

// 4) For SteeringState
void steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg)
{
  ROS_INFO_STREAM("SteeringState => angle=" << msg->current_angle
                  << ", state=" << (msg->current_state ? "TRUE" : "FALSE")
                  << ", cmd_finish=" << (msg->cmd_finish ) << magenta);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "display_node");
  ros::NodeHandle nh;

  // Subscribing to all topics
  ros::Subscriber debug_sub = nh.subscribe("debug_info", 10, debugInfoCallback);
  ros::Subscriber steering_state_sub = nh.subscribe("steering_state", 10, steeringStateCallback);
  ros::Subscriber steering_cmd_sub   = nh.subscribe("steering_cmd", 10, steeringCmdCallback);
  ros::Subscriber wheel_cmd_sub      = nh.subscribe("wheel_cmd", 10, wheelCmdCallback);

  ros::spin();
  return 0;
}
