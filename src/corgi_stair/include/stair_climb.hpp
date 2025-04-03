#ifndef STAIRCLIMB_HPP
#define STAIRCLIMB_HPP

#include <array>
#include <vector>
#include <complex>

#include "bezier.hpp"
#include "leg_model.hpp"
#include "leg_info.hpp"

struct StairEdge {
    std::array<double, 2> edge;
    int count;
};

class StairClimb {
    public:
        // Constructor
        StairClimb(bool sim=true, std::array<double, 2> CoM_bias={0.0, 0.0}, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        void initialize(double init_eta[8]);
        std::array<std::array<double, 4>, 2> step();
        void add_stair_edge(double x, double y);
        double get_pitch();
        bool if_any_stair();

    private:
        /* Private function */
        void init_move_CoM_stable(int swing_leg);
        bool move_CoM_stable();
        // void move_CoM_stable_fixed_leg_length();
        void init_swing_same_step(int swing_leg, double front_height, double hind_height);
        bool swing_same_step();
        void init_swing_next_step(int swing_leg, double front_height, double hind_height); 
        bool swing_next_step();
        std::array<double, 2> move_consider_edge(int leg_ID, std::array<double, 2> move_vec);
        std::array<double, 2> move_edge(int leg_ID, std::array<double, 2> contact_p, double contact_alpha, double tol = 1e-14, size_t max_iter = 100);
        double objective_edge(double d_x, std::array<double, 2> init_U, std::array<double, 2> contact_p, double contact_alpha);
        bool determine_next_foothold();
        std::array<double, 2> get_foothold(double theta, double beta, int contact_rim = -1);
        void update_hip();

        /* Constant value */
        LegModel leg_model;
        LegInfo leg_info[4] = {LegInfo(0), LegInfo(1), LegInfo(2), LegInfo(3)};
        const std::array<std::array<int, 2>, 4> other_side_leg = {{{3, 1},    // front-hind, left-right
                                                             {2, 0}, 
                                                             {1, 3}, 
                                                             {0, 2}}};
        const double BL;  // body length
        const double BW;  // body width
        const double BH;  // body height
        const std::array<double, 2> CoM_bias;
        const double swing_time = 0.2;                                                
        const double min_margin = 0.01;
        const double max_velocity = 0.1; // m/s, max velocity of CoM
        const double acc = max_velocity / 0.5;
        const double stability_margin = 0.03;
        const std::array<int, 4> swing_sequence = {0, 2, 1, 3}; // sequence of swing leg 
        const double keep_edge_distance = 0.03;
        const double stand_height_on_stair_front = 0.3;
        const double stand_height_on_stair_hind  = 0.3;
        const double keep_stair_distance_all = 0.22;
        const double keep_stair_distance_hind = 0.22;
        const double keep_stair_distance_front = 0.05;
        const double step_length_up_stair = 0.3;
        const double min_swing_time_cw   = 1.5, 
                     min_swing_time_ccw  = 1.5, 
                     min_swing_time_step = 0.5;

        /* Variable */
        int rate;
        double dS;
        double incre_duty;
        std::array<double, 2> velocity = {0.1, 0.0};    // v_x, v_y of CoM 
        double stand_height = 0.25;
        double step_length  = 0.3;
        double step_height  = 0.04; // step height for swing on same step
        double max_theta = 2.7348759382405214;  // rad, corresponding to leg length 0.34
        double max_length = 0.34;  // rad, corresponding to leg length 0.34
        std::array<SwingProfile, 4> sp;
        int swing_count;

        // State
        std::array<double, 4> theta;
        std::array<double, 4> beta;
        std::array<double, 2> CoM;
        std::array<std::array<double, 2>, 4> hip;
        std::array<std::array<double, 2>, 4> last_hip;
        double pitch;
        int swing_leg;
        bool achieve_max_length;

        std::vector<StairEdge> stair_edge[4];
        int stair_count = 0;

        enum STATES {MOVE_STABLE, SWING_SAME, SWING_NEXT, END};
        STATES state;
        STATES last_state;

        // Intermediate variables
        std::array<double, 2> result_eta;
        std::string touch_rim_list[3] = {"G", "L_l", "L_r"};
        std::vector<std::vector<int>> touch_rim_idx = {{3}, {1, 2}, {4, 5}};

        /* Variable for move_CoM_stable */
        std::array<double, 2> CoM_offset;
        int move_dir;
        /* Variable for swing_same_step */
        double front_height, hind_height;
        double t_f_x, t_f_y, t_f;
        double coeff_a, coeff_b;
        double local_max_velocity;
        double margin_d;
        double vel_incre;
        int total_steps, step_count;
        /* Variable for swing_next_step */
        bool is_clockwise;
        double coeff_a_x, coeff_b_x, coeff_a_y, coeff_b_y;
        bool first_in, second_in, third_in;
        std::array<double, 2> final_hip;
        double final_theta, final_beta;
};//end class StairClimb

#endif // STAIRCLIMB_HPP
