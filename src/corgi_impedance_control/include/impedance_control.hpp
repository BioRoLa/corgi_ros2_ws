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

Eigen::MatrixXd eta_fb(2, 1);
Eigen::MatrixXd eta_ref(2, 1);
Eigen::MatrixXd force_fb(2, 1);
Eigen::MatrixXd force_ref(2, 1);

std::vector<Eigen::MatrixXd> pos_err_prev_modules;
std::vector<Eigen::MatrixXd> force_err_prev_modules;


class AdmittanceController {
public:
    double dt;

    Eigen::MatrixXd M;
    Eigen::MatrixXd K;
    Eigen::MatrixXd D;

    Eigen::MatrixXd pos_fb;
    Eigen::MatrixXd pos_ref;

    Eigen::MatrixXd pos_cmd;
    Eigen::MatrixXd eta_cmd;

    LegModel legmodel;


    AdmittanceController();
    void update(const Eigen::MatrixXd& eta_ref, const Eigen::MatrixXd& pos_err_prev, const Eigen::MatrixXd& force_err_prev);
};

#endif