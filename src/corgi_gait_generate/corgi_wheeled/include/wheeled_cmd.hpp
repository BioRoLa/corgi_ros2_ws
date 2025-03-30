#ifndef WHEEL_CMD_HPP
#define WHEEL_CMD_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <corgi_msgs/WheelCmd.h>
#include <corgi_msgs/SteeringCmdStamped.h>
#include <corgi_msgs/SteeringStateStamped.h>
#include <algorithm> 

class WheeledCmd
{
  public:
    WheeledCmd();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void teleopCallback(const std_msgs::String::ConstPtr& key_msg);

    void steeringStateCallback(const corgi_msgs::SteeringStateStamped::ConstPtr& msg);
    // Timers for continuous publishing when commands are active
    void wheelCmdTimerCallback(const ros::TimerEvent&);
    void steerCmdTimerCallback(const ros::TimerEvent&);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber joy_sub_;
    ros::Subscriber teleop_sub_;
    ros::Subscriber steering_state_sub_;

    ros::Publisher wheel_cmd_pub_;
    ros::Publisher steering_cmd_pub_;
    ros::Publisher debug_pub_;

    ros::Timer wheel_cmd_timer_;
    ros::Timer steer_cmd_timer_;

    corgi_msgs::WheelCmd last_wheel_cmd_;
    corgi_msgs::SteeringCmdStamped steering_cmd_;

    int axis_steer_;         
    int axis_move_;          
    int axis_accel_;         
    int button_toggle_hold_; 
    int button_reset_;
    int button_ground_;

    // Control mode ("joystick" or "teleop")
    std::string control_mode_;

    // Internal state variables
    bool hold_active_;
    bool was_hold_pressed_;
    bool was_reset_pressed_;
    double current_velocity_;

    // Current steering state feedback (from /steer/state)
    corgi_msgs::SteeringStateStamped current_steering_state_;

    // Helper function: clamp a value between a minimum and maximum
    double clamp(double value, double min_val, double max_val);
};

#endif 
