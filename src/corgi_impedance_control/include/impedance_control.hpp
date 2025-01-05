#ifndef IMPEDANCE_CONTROL_HPP
#define IMPEDANCE_CONTROL_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_model.hpp"

#include "ros/ros.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/ForceStateStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/MotorCmdStamped.h"

corgi_msgs::ImpedanceCmdStamped imp_cmd;
corgi_msgs::ForceStateStamped force_state;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::MotorCmdStamped motor_cmd;

Eigen::MatrixXd M(2, 2);
Eigen::MatrixXd K(2, 2);
Eigen::MatrixXd D(2, 2);
Eigen::MatrixXd eta_fb(2, 1);
Eigen::MatrixXd eta_ref(2, 1);
Eigen::MatrixXd force_fb(2, 1);
Eigen::MatrixXd force_ref(2, 1);


class AdmittanceController {
public:
    double dt;

    Eigen::MatrixXd M_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd D_;

    Eigen::MatrixXd pos_fb;
    Eigen::MatrixXd vel_fb;
    Eigen::MatrixXd acc_fb;
    Eigen::MatrixXd pos_fb_prev;
    Eigen::MatrixXd vel_fb_prev;
    Eigen::MatrixXd acc_fb_prev;

    Eigen::MatrixXd pos_ref;
    Eigen::MatrixXd vel_ref;
    Eigen::MatrixXd acc_ref;
    Eigen::MatrixXd pos_ref_prev;
    Eigen::MatrixXd vel_ref_prev;
    Eigen::MatrixXd acc_ref_prev;

    LegModel legmodel;


    AdmittanceController(const Eigen::MatrixXd& M, const Eigen::MatrixXd& K, const Eigen::MatrixXd& D);
    void update(const Eigen::MatrixXd& eta_fb, const Eigen::MatrixXd& eta_ref, const Eigen::MatrixXd& force_fb, const Eigen::MatrixXd& force_ref);
};

#endif