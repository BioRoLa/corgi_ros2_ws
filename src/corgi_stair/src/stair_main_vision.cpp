#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "corgi_msgs/StairPlanes.h"
#include "leg_model.hpp"
#include "bezier.hpp"
#include "walk_gait.hpp"
#include "stair_climb.hpp"

#define INIT_THETA (M_PI*17.0/180.0)
#define INIT_BETA (0.0)

corgi_msgs::TriggerStamped trigger_msg;
corgi_msgs::StairPlanes plane_msg;

void trigger_cb(const corgi_msgs::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

void camera_cb(const corgi_msgs::StairPlaness::ConstPtr& msg) {
    plane_msg = *msg;
}//end camera_cb

double getPitchFromTransform(const geometry_msgs::TransformStamped& tf) {
    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::cout << "Pitch: " << pitch << " radians" << std::endl;
    return pitch;  // 這就是繞 Y 軸的旋轉角（弧度）
}//end getPitchFromTransform


int main(int argc, char** argv) {
    ros::init(argc, argv, "stair_climb");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1, trigger_cb);
    ros::Subscriber camera_sub = nh.subscribe<corgi_msgs::StairPlanes>("stair_planes", 1, camera_cb);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    // pid command
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 150.0;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150.0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    /* Setting variable */
    enum STATES {INIT, TRANSFORM, WAIT, WALK, STAIR, END};
    const std::array<double, 2> CoM_bias = {0.0, 0.0};
    const std::array<double, 2> camera_bias = {-0.01, 0.25, 0.032};   // initial camera position in map frame, (x, y, z)
    const std::array<double, 2> CoM2cemera = {0.32075, 0.099};  // translation from CoM to camera
    const int sampling_rate = 1000;
    const int transform_count = 2*sampling_rate; // 2 second
    double init_eta[8] = {1.857467698281913, 0.4791102940603915, 1.6046663223045279, 0.12914729012802004, 1.6046663223045279, -0.12914729012802004, 1.857467698281913, -0.4791102940603915};  // stand height 0.25, step length 0.3
    double velocity = 0.1; // velocity for walk gait
    double stand_height = 0.25; // stand height for walk gait
    double step_length = 0.3; // step length for walk gait
    double step_height = 0.04; // step height for walk gait

    /* Initial variable */
    ros::Rate rate(sampling_rate);
    WalkGait walk_gait(false, CoM_bias[0], sampling_rate);
    StairClimb stair_climb(false, CoM_bias, sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA , INIT_BETA , INIT_BETA , INIT_BETA }}};   // init eta (wheel mode)
    
    /* Other variable */
    STATES state = INIT, last_state = INIT;
    bool first_camera_pose;
    double transform_ratio;
    bool trigger;
    int stair_count;
    int command_count;
    double pitch;
    double max_cal_time = 0.0;
    std::array<int, 4> swing_phase;
    double min_keep_stair_d;
    double hip_x, to_stair_d;
    double max_step_length_last;
    std::array<bool, 4> if_contact_edge, last_if_contact_edge;
    geometry_msgs::TransformStamped initial_camera_transform, camera_transform;

    /* Behavior loop */
    auto start = std::chrono::high_resolution_clock::now();
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    while (ros::ok()) {
        auto one_loop_start = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        if (state == END) {
            break;
        }//end if
        /* State machine */
        switch (state) {
            case INIT:
                first_camera_pose = false;
                transform_ratio = 0.0;
                trigger = false;
                pitch = 0.0;
                stair_count = 0;
                command_count = 0;
                break;
            case TRANSFORM:
                transform_ratio += 1.0 / transform_count;
                for (int i=0; i<4; i++) {
                    eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i*2]   - INIT_THETA);
                    eta_list[1][i] = INIT_BETA  + transform_ratio * (init_eta[i*2+1] - INIT_BETA);
                    eta_list[1][i] = (i == 1 || i == 2)? eta_list[1][i] : -eta_list[1][i];  
                }//end for
                break;
            case WAIT:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    walk_gait.initialize(current_eta);
                }//end if

                break;
            case WALK:
                /* Get camera pose */
                if (tfBuffer.canTransform("map", "zedxm_camera_center", ros::Time(0), ros::Duration(0.0))) {
                    try {
                        camera_transform = tfBuffer.lookupTransform("map", "zedxm_camera_center", ros::Time(0));
                        if (!first_camera_pose) {
                            initial_camera_transform = camera_transform; // set initial camera pose
                            first_camera_pose = true;
                        }//end if
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN_THROTTLE(1.0, "TF lookup failed even after canTransform: %s", ex.what());
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "TF not available at this moment");
                }
                /* Vision feedback for real robot */
                hip_x = camera_transform.transform.translation.x - CoM2cemera[0] + 0.222; // front hip
                to_stair_d = hip_x + 100;  // set far from hip if not detect stair
                // Adjust last step length of walk gait, foothold of last walk step should not exceed min_keep_stair_d.
                if (plane_msg.vertical.size() > 0) {
                    to_stair_d = plane_msg.vertical[0] - camera_transform.transform.translation.x + CoM2cemera[0]; // distance from robot center to stair edge
                    max_step_length_last = (to_stair_d - 0.20 - hip_x - 0.3*step_length)*5; // step length if from current pos to min_keep_stair_d, step_length*(swing_phase + (1-swing_phase)/2) = foothold_x - hip_x
                    // std::cout << "max_step_length_last: " << max_step_length_last << std::endl;
                    // std::cout << "hip: " << hip_x << std::endl;
                    if ( max_step_length_last > 0 && step_length >= max_step_length_last ) {
                        walk_gait.set_step_length(max_step_length_last); 
                    }//end if
                }//end if
                eta_list = walk_gait.step();
                command_count ++;
                break;
            case STAIR:
                /* Initialize stair-climbing mode */
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    stair_climb.initialize(current_eta, velocity);
                }//end if
                /* Get camera pose */
                if (tfBuffer.canTransform("map", "zedxm_camera_center", ros::Time(0), ros::Duration(0.0))) {
                    try {
                        camera_transform = tfBuffer.lookupTransform("map", "zedxm_camera_center", ros::Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN_THROTTLE(1.0, "TF lookup failed even after canTransform: %s", ex.what());
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "TF not available at this moment");
                }
                /* Add stair edge */
                if (stair_climb.any_no_stair()) {
                    if (plane_msg.vertical.size() > stair_count+1 && plane_msg.horizontal.size() > stair_count+2) {
                        double next_edge_x = plane_msg.vertical[stair_count] + camera_bias[0];     // relative to camera frame
                        double next_edge_z = plane_msg.horizontal[stair_count+1] + camera_bias[2]; // relative to camera frame
                        
                        double real_pitch = - (getPitchFromTransform(camera_transform) - getPitchFromTransform(initial_camera_transform));
                        double CoM2camera_x = CoM2cemera[0] * std::cos(real_pitch) - CoM2cemera[1] * std::sin(real_pitch);
                        double CoM2camera_z = CoM2cemera[0] * std::sin(real_pitch) + CoM2cemera[1] * std::cos(real_pitch);
                        stair_climb.add_stair_edge_CoM(next_edge_x - camera_transform.transform.translation.x - CoM2camera_x, next_edge_z - camera_transform.transform.translation.z - CoM2camera_z);
                        stair_count++;
                    }//end if
                }//end if
                eta_list = stair_climb.step();
                pitch = stair_climb.get_pitch();
                command_count ++;
                break;
            default:
                break;
        }//end switch
        last_state = state;

        /* Next state */
        switch (state) {
            case INIT:
                state = TRANSFORM;
                break;
            case TRANSFORM:
                if (transform_ratio > 1.0) {
                    state = WAIT;
                }//end if
                break;
            case WAIT:
                if (trigger_msg.enable) {
                    state = WALK;
                }//end if
                break;
            case WALK:
                // Entering stair climbing phase
                swing_phase = walk_gait.get_swing_phase();
                if (walk_gait.if_touchdown() && (swing_phase[0]==1 || swing_phase[1]==1)) { // hind leg touched down (front leg start to swing)
                    if (hip_x + 0.15 >= to_stair_d - 0.10) {   // max next foothold >= keep_stair_d_front_max, to swing up stair
                        state = STAIR;
                        std::cout << "Enter stair climbing phase." << std::endl;
                    }//end if
                }//end if
                break;
            case STAIR:
                if (!stair_climb.if_any_stair()) {
                    state = END;
                }//end if  
                break;
            default:
                break;
        }//end switch

        /* Print when the leg contact stair edge */
        if_contact_edge = stair_climb.get_contact_edge_leg();
        for (int i=0; i<4; i++) {
            if (if_contact_edge[i] && !last_if_contact_edge[i]) {
                std::cout << "Command: " << command_count << ". Leg " << i << " is contacting the stair edge." << std::endl;
            }//end if
        }//end for
        last_if_contact_edge = if_contact_edge;

        /* Publish motor commands */
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Leg " << i << " exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI*160.0/180.0;
            }//end if 
            if (eta_list[0][i] < M_PI*16.9/180.0) {
                std::cout << "Leg " << i << " exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI*16.9/180.0;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2)? (eta_list[1][i]-pitch) : -(eta_list[1][i]-pitch);
        }//end for
        motor_pub.publish(motor_cmd);
        auto one_loop_end = std::chrono::high_resolution_clock::now();
        auto one_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(one_loop_end - one_loop_start);
        if (one_loop_duration.count() > max_cal_time) {
            max_cal_time = one_loop_duration.count();
            std::cout << "max time: " << max_cal_time << " us" << std::endl;
        }//end if
        rate.sleep();
    }//end while
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "max time: " << max_cal_time << " us" << std::endl;
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    std::cout << "total count: " << command_count << std::endl;

    
    ros::shutdown();
    return 0;
}//end main

