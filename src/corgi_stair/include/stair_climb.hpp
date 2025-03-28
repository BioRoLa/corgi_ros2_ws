#ifndef STAIRCLIMB_HPP
#define STAIRCLIMB_HPP

#include <array>
#include <complex>

#include "leg_model.hpp"
#include "leg_info.hpp"

class StairClimb {
    public:
        // Constructor
        StairClimb(bool sim=true, double CoM_bias=0.0, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        std::array<std::array<double, 4>, 2> step();

    private:
        bool move_CoM_stable();
        void move_CoM_stable_fixed_leg_length(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch);
        void swing_next_step_smooth(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch, double front_height, double hind_height);

        // Constant
        std::array<std::array<int, 2>, 4> other_side_leg = {{{3, 1},    // front-hind, left-right
                                                             {2, 0}, 
                                                             {1, 3}, 
                                                             {0, 2}}};
        LegModel leg_model;
        LegInfo leg_info[4] = {LegInfo(0), LegInfo(1), LegInfo(2), LegInfo(3)};
        int rate;
        double BL, BH, BW;
        double min_margin;
        std::array<double, 2> CoM_bias;
        double max_velocity;
        double stability_margin;
        double acc;
        double min_swing_time_cw, min_swing_time_ccw, min_swing_time_step;
        // State
        std::array<double, 4> theta;
        std::array<double, 4> beta;
        std::array<double, 2> CoM;
        std::array<double, 2> velocity;
        std::array<std::array<double, 4>, 2> last_hip;
        double pitch;
        int swing_leg;

        std::array<double, 2> result_eta;
        std::vector<std::array<double, 2>> stairs_edge[4];



};//end class StairClimb

#endif // STAIRCLIMB_HPP
