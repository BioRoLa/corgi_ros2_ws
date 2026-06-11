#ifndef VMC_HPP
#define VMC_HPP

#include <iostream>
#include <iomanip>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/force_state_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/contact_state_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"

#include "corgi_walk/walk_gait.hpp"
#include "corgi_walk/trot_gait.hpp"
#include "corgi_hybrid/hybrid_gen.hpp"

#include <array>
#include <vector>

class VirtualModelController {
    public:
        int target_loop = 2000;

        double Mx = 0;
        double My = 0;
        double Bx_swing = 80;
        double By_swing = 80;
        double Bx_stance = 80;
        double By_stance = 10;
        double Kx_swing = 2000;
        double Ky_swing = 2000;
        double Kx_stance = 2000;
        double Ky_stance = 100;

        const int freq = 500;
        const double dt = 1.0 / freq;

        const double m = 20.98;
        const double gravity = 9.81;

        const int n_u = 12;  // 3D force x 4 legs

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

        void load_config();
        Eigen::VectorXd step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref,
                             const bool *selection_matrix,
                             const double *ra, const double *rb, const double *rc, const double *rd);

    private:
        // PD gains: [roll, pitch, yaw, x, y, z]
        Eigen::VectorXd Kp = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd Kd = Eigen::VectorXd::Zero(6);
        // Wrench tracking weights: [tau_roll, tau_pitch, tau_yaw, Fx, Fy, Fz]
        Eigen::VectorXd wrench_weight = (Eigen::VectorXd(6) << 1e3, 1e3, 0, 0, 0, 1e3).finished();
        double force_weight = 1e-4;

        int fx_upper_bound = 30;
        int fx_lower_bound = -30;
        int fz_upper_bound = 150;
        int fz_lower_bound = -50;

        double friction_coef = 1;

        double l = 0.62;
        double w = 0.33;
        double h = 0.17;

        Eigen::VectorXd compute_wrench(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref);
        Eigen::MatrixXd build_contact_matrix(const double *ra, const double *rb, const double *rc, const double *rd);
        Eigen::VectorXd distribute_forces(const Eigen::VectorXd &F_virtual, const Eigen::MatrixXd &A, const bool *selection_matrix);
};

#endif
