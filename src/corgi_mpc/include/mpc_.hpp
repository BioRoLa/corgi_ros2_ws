#ifndef MPC__HPP
#define MPC__HPP

#include <iostream>
#include <iomanip>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "ros/ros.h"

#include "corgi_msgs/SimDataStamped.h"
#include "corgi_msgs/ImpedanceCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "corgi_msgs/ForceStateStamped.h"
#include "corgi_msgs/MotorStateStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

#include "walk_gait.hpp"
#include "hybrid_gen.hpp"

#include <array>
#include <vector>

class ModelPredictiveController {
    public:
        int target_loop = 3000;

        double Mx = 0;
        double My = 0;
        double Bx_swing = 90;
        double By_swing = 90;
        double Bx_stance = 40;
        double By_stance = 30;
        double Kx_swing = 2000;
        double Ky_swing = 2000;
        double Kx_stance = 800;
        double Ky_stance = 200;


        const int freq = 100;
        const double dt = 1.0 / freq;

        const int m = 19.5;
        const double gravity = 9.81;

        const int N = 30;
        const int n_x = 13;
        const int n_u = 12;

        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        double target_pos_x = 0;
        double target_pos_z = 0;
        double target_vel_x = 0;
        double target_vel_z = 0;

        std::array<double, 3> robot_vel = {0, 0, 0};
        std::array<double, 3> robot_pos = {0, 0, 0};
        std::array<double, 3> robot_ang_vel = {0, 0, 0};
        Eigen::Quaterniond robot_ang = Eigen::Quaterniond::Identity();

        std::array<std::array<double, 4>, 2> eta_list = {{{0, 0, 0, 0}, {0, 0, 0, 0}}};

        void init_matrices(const double *ra, const double *rb, const double *rc, const double *rd);
        Eigen::VectorXd step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref, const bool *selection_matrix, std::vector<corgi_msgs::ForceState*> force_state_modules);

    private:
        Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(n_x, n_x);
        Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(n_x, n_u);
        Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(n_x, n_x);
        Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(n_x, n_u);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n_x, n_x);
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(n_u, n_u);

        int fx_upper_bound = 20;
        int fx_lower_bound = -20;
        int fz_upper_bound = 200;
        int fz_lower_bound = -100;

        double friction_coef = 1;

        double l = 0.62;
        double w = 0.33;
        double h = 0.17;
};

#endif