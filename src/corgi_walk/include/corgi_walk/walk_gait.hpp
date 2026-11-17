#ifndef WALKGAIT_HPP
#define WALKGAIT_HPP

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"
#include "corgi_utils/swing_planner.hpp"
#include "corgi_utils/robot_params.hpp"

// Walk gait for the ABAD Corgi. The gait is planned in the sagittal plane with
// the ABAD joints held at gamma = 0, and the leg touches the ground on its foot
// arc (centre O_r, radius LegModel::foot_radius), which rolls during stance.
//
// stand_height is the height of the hip (ABAD axis) above the ground.
// step_length is the hip travel per gait cycle.
// theta/beta use the same convention as before: all legs share one sagittal
// frame with +x forward, so module A and D need their beta negated when sent
// to the motors.
class WalkGait {
    public:
        // BL: front to hind hip distance, BW: left to right wheel plane distance.
        WalkGait(bool sim=true, double CoM_bias=0.0, int rate=1000,
                 double BL=corgi_robot::wheel_base,
                 double BW=corgi_robot::abad_track + 2.0 * corgi_robot::abad_axis_to_wheel_plane,
                 double BH=corgi_robot::chassis_height);

        void initialize(double init_eta[8], double step_length_=0.3);
        std::array<std::array<double, 4>, 2> step();
        void set_velocity(double new_value);
        void set_stand_height(double new_value);
        void set_step_length(double new_value);
        void set_step_height(double new_value);
        void set_curvature(double new_value);
        void set_eta(std::array<std::array<double, 4>, 2> eta_);
        void set_duty(std::array<double, 4> duty_);
        std::array<int, 4> get_step_count();
        std::array<int, 4> get_swing_phase();
        std::array<double, 4> get_duty();
        bool if_touchdown();
        // Standing pose (same layout as init_eta) for the current stand_height
        // and the given step length, with module A the first leg to swing.
        std::array<double, 8> get_init_eta(double step_length_);
        // Longest step_length the foot arc can cover at the given stand height.
        double max_step_length(double stand_height_);

        double velocity     = 0.1;
        double stand_height = 0.25;
        double step_length  = 0.3;
        double step_height  = 0.04;

    private:
        LegModel leg_model;

        // Constant value
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const double CoM_bias;
        const double swing_time = 0.2;
        const double max_beta = 40.0 * M_PI / 180.0;  // foot arc spans alpha in [-40, 40] deg
        double max_reach;  // largest hip to O_r distance the linkage can reach

        // Variable
        int rate;
        double dS;
        double incre_duty;
        double curvature    = 0.0;  // +: turn left, -:turn right, 0: straight

        // State
        std::array<double, 4> theta;
        std::array<double, 4> beta;
        std::array<std::array<double, 2>, 4> foothold;
        std::array<std::array<double, 2>, 4> hip;
        std::array<std::array<double, 2>, 4> next_hip;

        std::array<double, 4> duty;
        std::array<int, 4> swing_phase = {0, 0, 0, 0};
        std::array<int, 4> step_count  = {0, 0, 0, 0};
        std::array<double, 4> current_step_length = {step_length, step_length, step_length, step_length};
        std::array<double, 4> next_step_length    = {step_length, step_length, step_length, step_length};
        double new_step_length = step_length;
        int direction = 1;
        bool touchdown;

        // Intermediate variables
        double swing_phase_ratio;
        std::array<double, 2> result_eta;
        // Swing of each leg, planned as in LegWheel (corgi_utils/swing_planner.hpp).
        std::array<legwheel_swing::RimSwing, 4> sp;
        // Pose of each leg one control period earlier (for the lift-off velocity).
        std::array<std::array<double, 2>, 4> prev_eta;

        // For turning 
        double outer_radius;
        double inner_radius;
        double diff_step_length = 0.0;  // Differential step length 
        double new_diff_step_length = 0.0;  // New differential step length
        double diff_dS = 0.0;   // Differential dS
        int sign_diff[4] = {0, 0, 0, 0};   // Differential sign

        // Largest touchdown beta allowed at the current stand height.
        double beta_limit(double stand_height_);
        // Distance the contact point leads the hip at touchdown so that the
        // stance sweep is symmetric: stance_travel = 2*H_O*tan(b) + 2*R_arc*b.
        double touchdown_lead(double stance_travel);
        // Touchdown beta b of that symmetric stance sweep.
        double stance_beta(double stance_travel);
};//end class WalkGait

#endif // WALKGAIT_HPP
