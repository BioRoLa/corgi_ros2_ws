#ifndef WALKGAIT_WITH_CONTACT_HPP
#define WALKGAIT_WITH_CONTACT_HPP

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

class WalkGaitWithContact {
    public:
        struct DebugState {
            std::array<std::array<double, 2>, 4> foothold;
            std::array<std::array<double, 2>, 4> touchdown_point;
            std::array<std::array<double, 2>, 4> foot_point;
            std::array<std::array<double, 2>, 4> hip;
            std::array<std::array<double, 2>, 4> next_hip;
            std::array<int, 4> swing_phase;
            std::array<bool, 4> touchdown_leg;
            std::array<bool, 4> contact_state;
            std::array<bool, 4> early_contact;
            std::array<bool, 4> late_probing;
        };

        WalkGaitWithContact(bool sim=true, double CoM_bias=0.0, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        void initialize(double init_eta[8], double step_length_=0.3);
        std::array<std::array<double, 4>, 2> step();
        void set_velocity(double new_value);
        void set_stand_height(double new_value);
        void set_ground_offset(const std::array<double, 4>& new_value);
        void set_step_length(double new_value);
        void set_step_height(double new_value);
        void set_curvature(double new_value);
        void set_eta(std::array<std::array<double, 4>, 2> eta_);
        void set_duty(std::array<double, 4> duty_);
        std::array<int, 4> get_step_count();
        std::array<int, 4> get_swing_phase();
        std::array<double, 4> get_duty();
        bool if_touchdown();
        DebugState get_debug_state() const;

        void set_probe_speed(double new_value);
        void set_contact_state(const std::array<bool, 4>& contact);
        void set_contact_filter(bool enabled, double swing_accept_ratio, int on_count, int off_count);

        double velocity     = 0.1;
        double stand_height = 0.2;
        double step_length  = 0.2;
        double step_height  = 0.08;
        std::array<double, 4> ground_offset = {0.0, 0.0, 0.0, 0.0};
        double probe_speed  = 0.5; // Late contact probing speed (m/s)

    private:
        LegModel leg_model;

        // Constant value
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const double CoM_bias;
        const double swing_time = 0.2;

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
        std::array<bool, 4> touchdown_leg = {false, false, false, false};

        std::array<bool, 4> raw_contact_state = {true, true, true, true};
        std::array<bool, 4> contact_state = {true, true, true, true};
        std::array<bool, 4> early_contact = {false, false, false, false};
        std::array<bool, 4> late_probing  = {false, false, false, false};
        bool contact_filter_enabled = false;
        double contact_swing_accept_ratio = 0.5;
        int contact_on_count_threshold = 15;
        int contact_off_count_threshold = 3;
        std::array<int, 4> contact_on_count = {0, 0, 0, 0};
        std::array<int, 4> contact_off_count = {0, 0, 0, 0};

        // Intermediate variables
        int current_rim;
        std::string touch_rim_list[5] = {"G", "L_l", "L_r", "U_l", "U_r"};
        int touch_rim_idx[5] = {3, 2, 4, 1, 5};
        double swing_phase_ratio;
        std::array<double, 2> curve_point_temp;
        std::array<double, 2> result_eta;
        std::array<double, 2> p_lo;
        std::array<double, 2> p_td;
        std::array<std::array<double, 2>, 4> touchdown_point;
        std::array<std::array<double, 2>, 4> foot_point;
        std::array<SwingProfile, 4> sp;

        bool update_contact_filter(int leg_idx, double current_swing_phase_ratio);

        // For turning 
        double outer_radius;
        double inner_radius;
        double diff_step_length = 0.0;  // Differential step length 
        double new_diff_step_length = 0.0;  // New differential step length
        double diff_dS = 0.0;   // Differential dS
        int sign_diff[4] = {0, 0, 0, 0};   // Differential sign
};//end class WalkGaitWithContact

#endif // WALKGAIT_WITH_CONTACT_HPP
