#ifndef WALKGAIT_HPP
#define WALKGAIT_HPP

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>

#include "leg_model.hpp"
#include "bezier.hpp"
class WalkGait {
    public:
        WalkGait(double init_eta[8], bool sim=true, double CoM_bias=0.0, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        void initialize(double init_theta[4], double init_beta[4]);

        std::array<std::array<double, 4>, 2> step();

        void set_velocity(double new_value);
        void set_stand_height(double new_value);
        void set_step_length(double new_value);
        void set_step_height(double new_value);

    private:
        LegModel leg_model;

        // constant value
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const double CoM_bias;
        const double swing_time = 0.2;

        // variable
        int rate;
        double dS;
        double incre_duty;
        double velocity     = 0.1;
        double stand_height = 0.2;
        double step_length  = 0.3;
        double step_height  = 0.05;

        // state
        std::array<double, 4> theta;
        std::array<double, 4> beta;
        std::array<std::array<double, 2>, 4> foothold;
        std::array<std::array<double, 2>, 4> hip;

        std::array<SwingProfile, 4> sp;
        std::array<double, 4> duty;
        std::array<int, 4> swing_phase = {0, 0, 0, 0};

        // Intermediate variables
        int current_rim;
        std::string touch_rim_list[3] = {"G", "L_l", "U_l"};
        int touch_rim_idx[3] = {3, 2, 1};
        double swing_phase_ratio;
        std::array<double, 2> curve_point_temp;
        std::array<double, 2> result_eta;
        std::array<double, 2> p_lo;
        std::array<double, 2> p_td;
        double stand_height_diff = 0.0;
};//end class WalkGait

#endif // WALKGAIT_HPP
