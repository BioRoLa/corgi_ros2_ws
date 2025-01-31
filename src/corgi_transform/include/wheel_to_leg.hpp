#ifndef WHEEL_TO_LEG_HPP
#define WHEEL_TO_LEG_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_model.hpp"
#include "bezier.hpp"

class WheelToLegTransformer {
    public:
        int stage = 0;

        WheelToLegTransformer(double init_eta[8], bool sim=true);

        void initialize(double init_theta[4], double init_beta[4]);

        double round_3(double value);
        double round_6(double value);
        double find_closest_beta(double target_beta, double ref_beta);
        double find_smaller_closest_beta(double target_beta, double ref_beta);
        std::array<double, 2> find_hybrid_steps(double RH_beta, double LH_beta, double body_angle);
        std::array<std::array<double, 4>, 2> step();

    private:
        bool transform_finished = false;

        LegModel leg_model;
        double G_p[2] = {0, 0};

        std::array<double, 4> curr_theta;
        std::array<double, 4> curr_beta;

        const double BL = 0.444;
        const double BW = 0.4;
        const double BH = 0.2;

        double last_transform_step_x = 0.15;
        double body_vel = 0.1;
        double stance_height = 0.2;

        double dt = 0.001;

        double wheel_delta_beta;

        int step_count = 0;

        double delta_time_step;
        double body_move_dist;

        double RF_target_beta, RF_target_theta;
        double LF_target_beta, LF_target_theta;

        double RF_delta_beta, RF_delta_theta;
        double LF_delta_beta, LF_delta_theta;

        std::array<double, 2> hybrid_steps;
        double step_num, step_length;

        double body_angle;


        int delta_time_step_each;
        SwingProfile sp;

        std::vector<double> swing_target_theta_traj;
        std::vector<double> swing_target_beta_traj;
        std::array<double, 2> hip_pos;

        std::vector<double> stance_target_theta_traj;
        std::vector<double> stance_target_beta_traj;
        double stance_theta, stance_beta;
        std::array<double, 2> move_vector;

        int traj_idx;
        
};




#endif
