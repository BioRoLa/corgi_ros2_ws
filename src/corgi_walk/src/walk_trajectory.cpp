#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "walk_trajectory.hpp"

#include <array>
#include <string>
#include <fstream>

// Main function
int main() {
    bool sim = true;
    LegModel leg_model(sim);

    // User-defined parameters
    bool animate = true;
    std::string output_file_name = "walk_trajectory";
    bool transform = false;
    double BL = 0.444;
    double BH = 0.2;
    double CoM_bias = 0.0;
    double velocity = 0.1;
    int sampling = 1000;
    double stand_height = 0.2 + leg_model.r;
    double step_length = 0.4;
    double step_height = 0.06;
    double forward_distance = 1.0;

    bool use_init_phi = true;
    std::array<double, 8> init_eta = {1.7908786895256839,0.7368824288764617,1.1794001564068406,-0.07401410141135822,1.1744876957173913,-1.8344700758454735e-15,1.790992783013031,5.5466991499313485};
    std::array<double, 4> init_theta = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};;
    std::array<double, 4> init_beta = {init_eta[1], init_eta[3], init_eta[5], init_eta[7]};
    init_beta[0] *= -1;
    init_beta[3] *= -1;

    double swing_time = 0.2;

    // Relative foothold calculation
    std::array<std::array<double, 2>, 4> relative_foothold = {};
    for (int i = 0; i < 4; ++i) {
        leg_model.forward(init_theta[i], init_beta[i]);
        relative_foothold[i][0] = leg_model.G[0];
        relative_foothold[i][1] = -stand_height;
    }

    int first_swing_leg = 0;
    for (int i = 1; i < 4; ++i) {
        if (relative_foothold[i][0] < relative_foothold[first_swing_leg][0]) {
            first_swing_leg = i;
        }
    }

    std::array<double, 4> duty;
    if (!use_init_phi || first_swing_leg == 0) {
        duty = {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
    } else if (first_swing_leg == 1) {
        duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
    } else if (first_swing_leg == 2) {
        duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
    } else {
        duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
    }

    std::array<int, 4> swing_phase = {0, 0, 0, 0};

    // More logic as per the Python code structure, ensuring placeholders for external function calls

    return 0;
}
