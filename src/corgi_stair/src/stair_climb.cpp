#include <iostream>
#include <cmath>
#include <stdexcept>
#include <array>
#include <vector>
#include <algorithm>

#include "bezier.hpp"
#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "trajectory_plan.hpp"
#include "stair_climb.hpp"

// #define BEZIER_CURVE
#define CW_SWING

StairClimb::StairClimb(bool sim, std::array<double, 2> CoM_bias, int rate, double BL, double BW, double BH) : 
    /* Initializer List */
    leg_model(sim),
    CoM_bias(CoM_bias),
    rate(rate), 
    BL(BL),
    BW(BW),
    BH(BH)
{
    // Initialize
    state = MOVE_STABLE;
    last_state = MOVE_STABLE;
    vel_incre = acc / rate;
    stair_count = 0;
}//end StairClimb

void StairClimb::initialize(double init_eta[8]) {
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4]  = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    int current_rim = 0;
    for (int i=0; i<4; i++) {
        leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = leg_model.rim;
        leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1) {
            relative_foothold[i][0] = leg_model.U_l[0];
        } else if (current_rim == 2) {
            relative_foothold[i][0] = leg_model.L_l[0];
        } else if (current_rim == 3) {
            relative_foothold[i][0] = leg_model.G[0];
        } else if (current_rim == 4) {
            relative_foothold[i][0] = leg_model.L_r[0];
        } else if (current_rim == 5) {
            relative_foothold[i][0] = leg_model.U_r[0];
        } else {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        }//end if else
        relative_foothold[i][1] = -stand_height;
    }//end for
    // Get first swing leg  
    int first_swing_leg = 0;
    for (int i=1; i<4; i++) {
        if (relative_foothold[i][0] < relative_foothold[first_swing_leg][0]) {
            first_swing_leg = i;
        }//end if
    }//end for 
    switch (first_swing_leg) {
        case 0: swing_count = 0; break;    
        case 1: swing_count = 2; break;
        case 2: swing_count = 1; break;
        case 3: swing_count = 3; break;
        default: std::cout << "Error in determining first swing leg." << std::endl; break;
    }//end switch
    // Get foothold in world coordinate
    CoM = {0, stand_height};
    pitch = 0;
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    front_height = stand_height;
    hind_height  = stand_height;
    // Initial theta/beta
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i]  = init_beta[i];
    }//end for
    init_move_CoM_stable(swing_sequence[swing_count % 4]);
}//end initialize

std::array<std::array<double, 4>, 2> StairClimb::step() {
    // state machine
    bool finish_move = false;
    switch (this->state) {
        case MOVE_STABLE:
            if (last_state != state) {
                init_move_CoM_stable(swing_sequence[swing_count % 4]);
            }//end if
            finish_move = move_CoM_stable();
            break;
        case SWING_SAME:
            if (last_state != state) {
                swing_leg = swing_sequence[swing_count % 4];
                // front_height = hip[0][1];
                // hind_height  = hip[3][1];
                if (!stair_edge[swing_leg].empty()) {
                    if (stair_edge[swing_leg].front().count == 1 && leg_info[swing_leg].next_up) {
                        if (swing_leg == 0 || swing_leg == 1) {
                            front_height = stand_height_on_stair_front;
                        } else {
                            hind_height  = stand_height_on_stair_hind;
                        }//end if else
                    }//end if
                }//end if
                init_swing_same_step(swing_sequence[swing_count % 4], front_height, hind_height);
            }//end if
            finish_move = swing_same_step();
            break;
        case SWING_NEXT:
            if (last_state != state) {
                swing_leg = swing_sequence[swing_count % 4];
                // front_height = hip[0][1];
                // hind_height  = hip[3][1];
                this->is_clockwise = true;
                if (swing_leg == 0 || swing_leg == 1) {
                    if (!stair_edge[0].empty() && !stair_edge[1].empty()) { // at most only one will be empty
                        if (stair_edge[0].front().count != stair_edge[1].front().count) {   // second swing leg
                            double stand_height_on_stair = stair_edge[swing_leg].size() >= 2? stand_height_on_stair_front : stand_height;
                            front_height = stair_edge[swing_leg].front().edge[1] + stand_height_on_stair;
                            this->is_clockwise = false;
                        } else {
                            // front_height = stair_edge[swing_leg].front().edge[1] + stand_height;
                        }//end if else
                    } else {    // first swing leg
                        this->is_clockwise = false;
                        front_height = stair_edge[swing_leg].front().edge[1] + stand_height;
                    }//end if else
                } else {
                    if (!stair_edge[2].empty() && !stair_edge[3].empty()) { // at most only one will be empty
                        if (stair_edge[2].front().count != stair_edge[3].front().count) {   // second swing leg
                            double stand_height_on_stair = stair_edge[swing_leg].size() >= 2? stand_height_on_stair_hind : stand_height;
                            hind_height = stair_edge[swing_leg].front().edge[1] + stand_height_on_stair;
                            this->is_clockwise = false;
                        } else {
                            // hind_height = stair_edge[swing_leg].front().edge[1] + stand_height;
                        }//end if else
                    } else {    // first swing leg
                        this->is_clockwise = false;
                        hind_height = stair_edge[swing_leg].front().edge[1] + stand_height;
                    }//end if else  
                }//end if else
                init_swing_next_step(swing_sequence[swing_count % 4], front_height, hind_height);
            }//end if
            finish_move = swing_next_step();
            break;
        default:
            break;
    }//end switch
    if (last_state != state) {
        std::cout << "State:" << this->state << std::endl;
        std::cout << "Swing leg:" << swing_leg << std::endl;
        std::cout << "next_foothold:" << leg_info[swing_leg].next_foothold[0] << ", " << leg_info[swing_leg].next_foothold[1] << std::endl;
        std::cout << "front_height:" << this->front_height << std::endl;
        std::cout << "hind_height:" << this->hind_height << std::endl;
    }
    last_state = state;

    // next state
    switch (this->state) {
        case MOVE_STABLE:
            if (finish_move) {
                bool up_stair = determine_next_foothold();
                state = up_stair? SWING_NEXT : SWING_SAME;
                if (!this->if_any_stair()) {
                    state = END;
                }//end if
            }//end if
            break;
        case SWING_SAME:
            if (finish_move) {
                state = MOVE_STABLE;
                leg_info[swing_leg].foothold = leg_info[swing_leg].next_foothold;
                leg_info[swing_leg].contact_edge = false;
                swing_count ++;
            }//end if
            break;
        case SWING_NEXT:
            if (finish_move) {
                state = MOVE_STABLE;
                leg_info[swing_leg].stair_count = stair_edge[swing_leg].front().count;
                leg_info[swing_leg].foothold = leg_info[swing_leg].next_foothold;
                leg_info[swing_leg].contact_edge = false;
                stair_edge[swing_leg].erase(stair_edge[swing_leg].begin());
                swing_count ++;
            }//end if
            break;
        case END:
            std::cout << "End of stair climbing." << std::endl;
            break;
        default:
            break;
    }//end switch

    return {theta, beta};
}//end step

void StairClimb::add_stair_edge(double x, double y) {
    stair_count ++;
    stair_edge[0].push_back({{x, y}, stair_count});
    stair_edge[1].push_back({{x, y}, stair_count});
    stair_edge[2].push_back({{x, y}, stair_count});
    stair_edge[3].push_back({{x, y}, stair_count});
}//end add_stair_edge

double StairClimb::get_pitch() {
    return this->pitch;
}//end get_pitch

bool StairClimb::if_any_stair() {
    for (int i=0; i<4; i++) {
        if (!stair_edge[i].empty()) {
            return true;
        }//end if
    }//end for
    return false;
}//end if_any_stair

std::array<bool, 4> StairClimb::get_contact_edge_leg() {
    std::array<bool, 4> if_contact_edge = {false, false, false, false};
    for (int i=0; i<4; i++) {
        if (leg_info[i].contact_edge) {
            if_contact_edge[i] = true;
        }//end if
    }//end for
    return if_contact_edge;
}//end get_stair_count


/* Private function */
void StairClimb::init_move_CoM_stable(int swing_leg) { 
    this->swing_leg = swing_leg;
    this->move_dir = (swing_leg == 0 || swing_leg == 1) ? -1 : 1;
    this->CoM_offset = { std::cos(pitch) * CoM_bias[0] - std::sin(pitch) * CoM_bias[1],
                         std::sin(pitch) * CoM_bias[0] + std::cos(pitch) * CoM_bias[1] };
    this->achieve_max_length = false;
}//end init_move_CoM_stable

bool StairClimb::move_CoM_stable() {    // return true if stable, false if not
    this->update_hip();
    /* Change velocity */
    if (move_dir * velocity[0] < max_velocity) {
        velocity[0] += move_dir * vel_incre;
    } else if (move_dir * velocity[0] > max_velocity + vel_incre) {
        velocity[0] -= move_dir * vel_incre;
    }//end if else
    CoM[0] += velocity[0] / rate;
    /* Calculate leg command */
    if (achieve_max_length) {   // if achieve max leg length, let the leg's length to be fixed
        if (!leg_info[swing_leg].contact_edge) {
            leg_model.forward(theta[swing_leg], beta[swing_leg]);
            CoM[1] += leg_model.G[1] + std::sqrt(max_length*max_length - std::pow(velocity[0]/rate - leg_model.G[0], 2));    // hip_y = last_hip_y + leg_model.G[1] + std::sqrt( max_length**2 - (hip_x - (last_hip_x + leg_model.G[0]))**2 ), hip_x - last_hip_x = velocity[0] / rate
        }//end if
    }//end if
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    /* Check if achieve max leg length */
    if (theta[swing_leg] >= max_theta) {
        achieve_max_length = true;
    }//end if
    /* Return if stable (entering support triangle) */
    // if (move_dir * (get_foothold(theta[(swing_leg+1)%4], beta[(swing_leg+1)%4])[0] + get_foothold(theta[(swing_leg+3)%4], beta[(swing_leg+3)%4])[0]) / 2 < move_dir * CoM_offset[0] - stability_margin) {
    //     return true;
    // } else {
    //     return false;
    // }//end if else
    if (move_dir * (CoM[0] + CoM_offset[0]) > move_dir * ((leg_info[(swing_leg+1)%4].foothold[0] + leg_info[(swing_leg+3)%4].foothold[0]) / 2) + stability_margin) {
        return true;
    } else {
        return false;
    }//end if else
}//end move_CoM_stable

// void StairClimb::move_CoM_stable_smooth_fixed_leg_length(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch) {
//     Eigen::Matrix2d rotation;
//     rotation << std::cos(pitch), -std::sin(pitch),
//                 std::sin(pitch),  std::cos(pitch);
//     Eigen::Vector2d CoM_offset = rotation * CoM_bias;

//     int move_dir = (leg_info[swing_leg].ID == 0 || leg_info[swing_leg].ID == 1) ? -1 : 1;
//     LegModel leg_model;
//     leg_model.forward(0.0, 0.0); // 初始化腿部模型
//     double leg_length = leg_model.G.norm();
    
//     while (move_dir * (CoM[0] + CoM_offset[0]) < move_dir * ((leg_info[(swing_leg + 1) % 4].foothold[0] + leg_info[(swing_leg - 1) % 4].foothold[0]) / 2) + stability_margin) {
//         if (move_dir * velocity[0] < max_velocity) {
//             velocity[0] += move_dir * acc / sampling;
//         }//end if
//         CoM += velocity / sampling;
        
//         double hip_x = CoM[0];
//         double hip_y;
//         leg_model.forward(0.0, 0.0);
//         if (leg_info[swing_leg].contact_edge) {
//             hip_y = CoM[1];
//         } else {
//             hip_y = CoM[1] + leg_model.G[1] + std::sqrt(leg_length * leg_length - std::pow(hip_x - (CoM[0] + leg_model.G[0]), 2));
//         }//end if else
        
//         Eigen::Vector2d hip = Eigen::Vector2d(hip_x, hip_y);
//         std::cout << "Hip position for swing leg " << swing_leg << ": " << hip.transpose() << std::endl;
//     }//end while
//     CoM = Eigen::Vector2d(CoM[0], CoM[1]);
//     pitch = std::asin((CoM[1] - CoM[1]) / BL);
// }//end move_CoM_stable_fixed_leg_length

void StairClimb::init_swing_same_step(int swing_leg, double front_height, double hind_height) { 
    this->swing_leg = swing_leg;
    this->margin_d = std::abs(CoM[0] - (leg_info[(swing_leg+1)%4].foothold[0]+leg_info[(swing_leg+3)%4].foothold[0])/2) - min_margin;
    leg_model.forward(theta[swing_leg], beta[swing_leg]);
    std::array<double, 2> p_lo = {hip[swing_leg][0] + leg_model.G[0], hip[swing_leg][1] + leg_model.G[1]}; 
    std::array<double, 2> p_td = {leg_info[swing_leg].next_foothold[0], leg_info[swing_leg].next_foothold[1]+leg_model.r};
    this->sp[swing_leg] = SwingProfile(p_lo, p_td, step_height, 1);

    this->final_CoM_height = (front_height + hind_height) / 2;
    this->coeff_b = acc;
    this->coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    this->t_f_x = margin_d / max_velocity;
    this->t_f_y = - coeff_b / coeff_a;
    this->t_f = std::max({min_swing_time_step, t_f_x, t_f_y});
    this->local_max_velocity = margin_d / t_f;
    this->total_steps = static_cast<int>(t_f * rate); // total steps for swinging
    this->step_count = 0;
}//end init_swing_same_step

bool StairClimb::swing_same_step() {  // return true if finish swinging, false if not
    this->update_hip();
    /* Change velocity */
    if (velocity[0] < local_max_velocity) {
        velocity[0] += vel_incre;
    } else if (velocity[0] > local_max_velocity + vel_incre) {
        velocity[0] -= vel_incre;
    }//end if else
    double t_ = (step_count+1.0) / rate;
    velocity[1] = (t_ <= t_f_y)? coeff_a * (t_*t_) + coeff_b * t_ : 0.0;
    CoM[0] += velocity[0] / rate;
    CoM[1] += velocity[1] / rate;
    /* Calculate pitch */
    if (swing_leg == 0 || swing_leg == 1) {
        pitch = std::asin((CoM[1]-hind_height) / (BL/2));
    } else {
        pitch = std::asin((front_height-CoM[1]) / (BL/2));
    }//end if else
    /* Calculate leg command */
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        if (i == swing_leg) {
            double swing_phase_ratio = (step_count+1.0) / total_steps;
            std::array<double, 2> curve_point = sp[i].getFootendPoint(swing_phase_ratio);
            std::array<double, 2> pos = {curve_point[0] - hip[i][0], curve_point[1] - hip[i][1]};
            result_eta = leg_model.inverse(pos, "G");
        } else {
            result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        }//end if else
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    step_count ++;
    /* Return if finish swinging */
    if (step_count==total_steps) {
        velocity[1] = 0.0;
        return true;
    } else {
        return false;
    }//end if else
}//end swing_same_step

void StairClimb::init_swing_next_step(int swing_leg, double front_height, double hind_height) { 
    this->swing_leg = swing_leg;
    // this->is_clockwise = (swing_leg == 0 || swing_leg == 1)? (stair_edge[0].front().count == stair_edge[1].front().count) : (stair_edge[2].front().count == stair_edge[3].front().count);

    this->final_CoM_height = (front_height + hind_height) / 2;
    this->coeff_b = acc;
    this->coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    this->t_f_x = velocity[0] / acc;
    this->t_f_y = - coeff_b / coeff_a;
    this->t_f = is_clockwise? min_swing_time_cw : min_swing_time_ccw;
    this->total_steps = static_cast<int>(t_f * rate); // total steps for swinging

    int sign_vel = velocity[0]>=0.0? 1 : -1;
    this->init_CoM = CoM;
    this->final_CoM = {CoM[0] + velocity[0]*t_f_x - sign_vel*acc*(t_f_x*t_f_x)/2, final_CoM_height};
    this->init_pitch = pitch;
    this->final_pitch = std::asin((front_height-hind_height)/BL);
    this->init_front_height = hip[0][1];
    this->init_hind_height  = hip[3][1];
    this->final_hip = leg_info[swing_leg].get_hip_position(final_CoM, final_pitch);

    for (int i=0; i<3; i++) {
        double rim_radius = i==0? leg_model.r : leg_model.radius;
        std::array<double, 2> pos = { 
            leg_info[swing_leg].next_foothold[0] - final_hip[0],
            leg_info[swing_leg].next_foothold[1] - final_hip[1] + rim_radius
        };
        result_eta = leg_model.inverse(pos, touch_rim_list[i]);
        leg_model.contact_map(result_eta[0], result_eta[1]);
        if (std::find(touch_rim_idx[i].begin(), touch_rim_idx[i].end(), leg_model.rim) != touch_rim_idx[i].end()) {
            break;
        }//end if
    }//end for
    final_theta = result_eta[0];
    final_beta  = result_eta[1];

    this->first_ratio  = is_clockwise? 0.1 : 0.2;  // 0 ~ first_ratio by G vertical up
    this->second_ratio = is_clockwise? 0.7 : 0.8;    // first_ratio ~ mid_ratio by theta and beta
    this->first_in  = true;
    this->second_in = true;
    this->third_in  = true;
    this->step_count = 0;
}//end init_swing_same_step

bool StairClimb::swing_next_step() {  // return true if finish swinging, false if not
    this->update_hip();
    /* Change velocity */
    if (velocity[0] > vel_incre) {
        velocity[0] -= vel_incre;
    } else if (velocity[0] < -vel_incre) {
        velocity[0] += vel_incre;
    } else {
        velocity[0] = 0.0;
    }//end if else
    double t_ = (step_count+1.0) / rate;
    velocity[1] = (t_<=t_f_y) ? coeff_a * (t_*t_) + coeff_b * t_ : 0.0;
    CoM[0] += velocity[0] / rate;
    CoM[1] += velocity[1] / rate;
    /* Calculate pitch */
    // if (swing_leg == 0 || swing_leg == 1) {
    //     pitch = std::asin((CoM[1]-hind_height) / (BL/2));
    // } else {
    //     pitch = std::asin((front_height-CoM[1]) / (BL/2));
    // }//end if else
    double CoM_up_ratio = (CoM[1] - init_CoM[1]) / (final_CoM[1] - init_CoM[1]);
    double current_front_height = init_front_height + CoM_up_ratio*(front_height - init_front_height);
    double current_hind_height  = init_hind_height  + CoM_up_ratio*(hind_height - init_hind_height);
    pitch = std::asin((current_front_height-current_hind_height) / BL);
    /* Calculate leg command */
    for (int i=0; i<4; i++) {
        hip[i] = leg_info[i].get_hip_position(CoM, pitch);
        if (i == swing_leg) {
            double swing_phase_ratio = (step_count+1.0) / total_steps;
            /* Bezier curve */
            #ifdef BEZIER_CURVE
            if (first_in) {
                first_in = false;
                leg_model.forward(theta[i], beta[i]);
                std::array<double, 2> current_G = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
                leg_model.forward(final_theta, final_beta);
                std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                this->sp[i] = SwingProfile(current_G, final_G, final_G[1]-current_G[1]+step_height, 1);
            }//end if
            std::array<double, 2> curve_point = sp[i].getFootendPoint(swing_phase_ratio);
            std::array<double, 2> pos = {curve_point[0] - hip[i][0], curve_point[1] - hip[i][1]};
            result_eta = leg_model.inverse(pos, "G");
            #endif
            /* Clockwise swing (first swing leg) */
            #ifdef CW_SWING
            if (swing_phase_ratio < first_ratio) {  // first phase
                if (first_in) {
                    first_in = false;
                    leg_model.forward(theta[i], beta[i]);
                    std::array<double, 2> current_G = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
                    para_traj[0] = LinearParaBlend({current_G[0], current_G[0]}            , {0.0, 1.0}, 0.3, true, 0.0, false, 0.0);
                    para_traj[1] = LinearParaBlend({current_G[1], current_G[1]+step_height}, {0.0, 1.0}, 0.3, true, 0.0, false, 0.0);
                }//end if
                if (is_clockwise) {
                    swing_phase_ratio = swing_phase_ratio / first_ratio;
                    double x_p = para_traj[0].get_point(swing_phase_ratio);
                    double y_p = para_traj[1].get_point(swing_phase_ratio);
                    std::array<double, 2> pos = {x_p-hip[i][0], y_p-hip[i][1]};
                    result_eta = leg_model.inverse(pos, "G");
                } else {
                    result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], 0});
                }//end if else
                last_theta[i] = theta[i];
                last_beta[i]  = beta[i];
            } else if (swing_phase_ratio < second_ratio) {  // second phase
                if (second_in) {
                    second_in = false;
                    // last point & velocity of first phase
                    double v_theta = (theta[i] - last_theta[i]) * rate * ((second_ratio-first_ratio)*total_steps/rate);
                    double v_beta  = (beta[i]  - last_beta[i])  * rate * ((second_ratio-first_ratio)*total_steps/rate);
                    leg_model.forward(final_theta, final_beta);
                    std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                    std::array<double, 2> C1 = hip[i];
                    std::array<double, 2> P1 = {final_G[0], final_G[1] + step_height};
                    std::array<double, 2> C1_P1 = {P1[0]-C1[0], P1[1]-C1[1]};
                    double C1_P1_d = std::hypot(C1_P1[0], C1_P1[1]);
                    double tan_line_angle;
                    if (C1_P1_d < leg_model.R) {
                        std::cerr << "Error in StairClimb::swing_next_step: P1 is inside the wheel." << std::endl;
                    } else {
                        tan_line_angle = std::acos(leg_model.R / C1_P1_d);
                    }//end if else
                    double second_theta = is_clockwise? 17.0/180.0*M_PI : (final_theta + 17.0/180.0*M_PI) / 2.0;
                    double second_beta  = is_clockwise? std::atan2(C1_P1[1], C1_P1[0]) + tan_line_angle + M_PI/2 - 2*M_PI : final_beta;
                    para_traj[0] = LinearParaBlend({theta[i], second_theta             , second_theta}, {0.0, 0.5, 1.0}, 0.1, true, v_theta, false, 0.0);
                    para_traj[1] = LinearParaBlend({beta[i] , (beta[i]+second_beta)/2.0, second_beta }, {0.0, 0.5, 1.0}, 0.1, true, v_beta , false, 0.0);
                    if (!is_clockwise) {
                        leg_model.forward(theta[i], beta[i]);
                        std::array<double, 2> current_G = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
                        leg_model.forward(final_theta, final_beta);
                        std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                        this->sp[i] = SwingProfile(current_G, final_G, final_G[1]-current_G[1]+2*step_height, 1);
                    }//end if
                }//end if
                if (is_clockwise) {
                    swing_phase_ratio = (swing_phase_ratio - first_ratio) / (second_ratio - first_ratio);
                    result_eta[0] = para_traj[0].get_point(swing_phase_ratio);
                    result_eta[1] = para_traj[1].get_point(swing_phase_ratio);
                } else {
                    swing_phase_ratio = (swing_phase_ratio - first_ratio) / (1.0 - first_ratio);
                    std::array<double, 2> curve_point = sp[i].getFootendPoint(swing_phase_ratio);
                    std::array<double, 2> pos = {curve_point[0] - hip[i][0], curve_point[1] - hip[i][1]};
                    result_eta = leg_model.inverse(pos, "G");
                }//end if else
                last_theta[i] = theta[i];
                last_beta[i]  = beta[i];
                for (int j=0; j<4; j++) {
                    last2_hip[j][0] = last_hip[j][0];
                    last2_hip[j][1] = last_hip[j][1];
                }//end for
            } else {    // third phase
                if (third_in) {
                    third_in = false;
                    // last point & velocity of second phase
                    leg_model.forward(theta[i], beta[i]);
                    std::array<double, 2> last_G = {last_hip[i][0] + leg_model.G[0], last_hip[i][1] + leg_model.G[1]};
                    leg_model.forward(last_theta[i], last_beta[i]);
                    std::array<double, 2> last2_G = {last2_hip[i][0] + leg_model.G[0], last2_hip[i][1] + leg_model.G[1]};
                    double v_x = (last_G[0] - last2_G[0]) * rate * ((1.0-second_ratio)*total_steps/rate);
                    double v_y = (last_G[1] - last2_G[1]) * rate * ((1.0-second_ratio)*total_steps/rate);
                    leg_model.forward(final_theta, final_beta);
                    std::array<double, 2> final_G = {final_hip[0] + leg_model.G[0], final_hip[1] + leg_model.G[1]};
                    para_traj[0] = LinearParaBlend({last_G[0], final_G[0]            , final_G[0]}, {0.0, 0.5, 1.0}, 0.3, true, v_x, true, 0.0);
                    para_traj[1] = LinearParaBlend({last_G[1], final_G[1]+step_height, final_G[1]}, {0.0, 0.5, 1.0}, 0.3, true, v_y, true, 0.0);
                }//end if
                swing_phase_ratio = (swing_phase_ratio - second_ratio) / (1.0 - second_ratio);
                double x_p = para_traj[0].get_point(swing_phase_ratio);
                double y_p = para_traj[1].get_point(swing_phase_ratio);
                std::array<double, 2> pos = {x_p-hip[i][0], y_p-hip[i][1]};
                result_eta = leg_model.inverse(pos, "G");
            }//end if else
            #endif
        } else {
            result_eta = move_consider_edge(i, {hip[i][0]-last_hip[i][0], hip[i][1]-last_hip[i][1]});
        }//end if else
        theta[i] = result_eta[0];
        beta[i]  = result_eta[1];
    }//end for
    step_count ++;
    /* Return if finish swinging */
    if (step_count==total_steps) {
        velocity[1] = 0.0;
        return true;
    } else {
        return false;
    }//end if else
}//end swing_next_step

std::array<double, 2> StairClimb::move_consider_edge(int leg_ID, std::array<double, 2> move_vec) {
    std::array<double, 2> current_stair_edge = {INFINITY, 0}; // [0] set very large to ensure execute correctly when the vector is empty.
    leg_model.forward(theta[leg_ID], beta[leg_ID]);
    if (!stair_edge[leg_ID].empty()) {
        current_stair_edge = stair_edge[leg_ID].front().edge;
        double err = 0.01;
        
        std::array<double, 2> edge_U_vec = {hip[leg_ID][0]+leg_model.U_r[0]-current_stair_edge[0], hip[leg_ID][1]+leg_model.U_r[1]-current_stair_edge[1]};
        if (!leg_info[leg_ID].contact_edge && (hip[leg_ID][0]+leg_model.U_r[0] <= current_stair_edge[0]) && (std::hypot(edge_U_vec[0], edge_U_vec[1]) <= leg_model.radius)) {
            leg_info[leg_ID].contact_edge = true;
            leg_model.forward(theta[leg_ID], beta[leg_ID], false);
            std::complex<double> current_stair_edge_c(current_stair_edge[0], current_stair_edge[1]);
            std::complex<double> hip_c(hip[leg_ID][0], hip[leg_ID][1]);
            leg_info[leg_ID].contact_alpha = std::arg((current_stair_edge_c - hip_c - leg_model.U_r_c) / (leg_model.F_r_c-leg_model.U_r_c));
        } else if (leg_info[leg_ID].contact_edge && (hip[leg_ID][0]+leg_model.U_r[0]>current_stair_edge[0] || std::hypot(edge_U_vec[0], edge_U_vec[1]) > leg_model.radius + err)) {
            leg_info[leg_ID].contact_edge = false;
        }//end if else
    }//end if
    
    if (leg_info[leg_ID].contact_edge) {
        result_eta = move_edge(leg_ID, {current_stair_edge[0]-hip[leg_ID][0], current_stair_edge[1]-hip[leg_ID][1]}, leg_info[leg_ID].contact_alpha);
        leg_info[leg_ID].foothold = {current_stair_edge[0], current_stair_edge[1]};
    } else {
        std::array<double, 2> relative_foothold;
        if (hip[leg_ID][0] + leg_model.U_r[0] > current_stair_edge[0]) {
            result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], move_vec, 0.0, true, false);
            relative_foothold = get_foothold(theta[leg_ID], beta[leg_ID], 5);
        } else {
            result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], move_vec, 0.0, false);
            relative_foothold = get_foothold(theta[leg_ID], beta[leg_ID]);
        }//end if else
        leg_info[leg_ID].foothold = {hip[leg_ID][0] + relative_foothold[0], hip[leg_ID][1] + relative_foothold[1]};
    }//end if else

    return result_eta;
}//end move_consider_edge

std::array<double, 2> StairClimb::move_edge(int leg_ID, std::array<double, 2> contact_p, double contact_alpha, double tol, size_t max_iter) {
    leg_model.forward(theta[leg_ID], beta[leg_ID]);
    std::array<double, 2> init_U = leg_model.U_r;
    
    // Use optimization solver to find d_x and d_y of init_U (analogous to fsolve)
    double guess_dx = 0.0;    // initial guess = 0
    for (size_t iter = 0; iter < max_iter; ++iter) {
        double cost = this->objective_edge(guess_dx, init_U, contact_p, contact_alpha);     // 计算当前函数值
        if (std::abs(cost) < tol) {                // 判断收敛
            // std::cout << "cost converged after " << iter << " iterations.\n";
            break;
        }//end if

        // computeJacobian, 数值计算雅可比矩阵
        double epsilon = 1e-6;
        double dx_eps = guess_dx + epsilon;
        double cost_eps = this->objective_edge(dx_eps, init_U, contact_p, contact_alpha);
        double cost_d = (cost_eps - cost) / epsilon;  // 数值差分计算导数

        double dx = -cost / cost_d;   // 解线性方程 cost_d * dx = -cost
        if (std::abs(dx) < tol) {             // 判断步长是否足够小
            // std::cout << "dx converged after " << iter << " iterations.\n";
            break;
        }//end if

        // 更新解
        guess_dx += dx;

        if (iter == max_iter-1) {
            std::cout << "Last cost: " << cost << std::endl;
            throw std::runtime_error("Move_edge: Newton solver did not converge.");
        }//end if
    }//end for

    double d_y = std::sqrt(std::pow(leg_model.radius, 2) - std::pow(contact_p[0]-(init_U[0]+guess_dx), 2)) - (init_U[1] - contact_p[1]);
    std::array<double, 2> new_U = {init_U[0]+guess_dx, init_U[1]+d_y};
    result_eta = leg_model.inverse(new_U, "U_r");
    return result_eta;
}//end move_edge

double StairClimb::objective_edge(double d_x, std::array<double, 2> init_U, std::array<double, 2> contact_p, double contact_alpha) {
    double d_y = std::sqrt(std::pow(leg_model.radius, 2) - std::pow(contact_p[0]-(init_U[0]+d_x), 2)) - (init_U[1] - contact_p[1]);
    std::array<double, 2> new_U = {init_U[0]+d_x, init_U[1]+d_y};
    std::array<double, 2> new_result_eta = leg_model.inverse(new_U, "U_r");
    leg_model.forward(new_result_eta[0], new_result_eta[1], false);

    std::complex<double> new_U_c(new_U[0], new_U[1]);
    std::complex<double> contact_p_c(contact_p[0], contact_p[1]);
    double new_alpha = std::arg((contact_p_c - leg_model.U_r_c) / (leg_model.F_r_c - leg_model.U_r_c));
    // double new_alpha = std::arg((contact_p_c - new_U_c) / (leg_model.F_r_c - new_U_c));
    return new_alpha - contact_alpha;
}//end objective_edge

bool StairClimb::determine_next_foothold() {
    bool up_stair = false;
    std::array<double, 2> current_stair_edge;
    int current_stair_count = leg_info[swing_leg].stair_count;
    int next_stair = 0;
    double keep_stair_d_max;
    double keep_stair_d_min;
    if (swing_leg < 2) {
        keep_stair_d_max = keep_stair_d_front_max;
        keep_stair_d_min = keep_stair_d_front_min;
    } else {
        keep_stair_d_max = keep_stair_d_hind_max;
        keep_stair_d_min = keep_stair_d_hind_min;
    }//end if else

    if (!stair_edge[swing_leg].empty()) {
        current_stair_edge  = stair_edge[swing_leg].front().edge;
        if ((leg_info[swing_leg].next_up || leg_info[ other_side_leg[swing_leg][1] ].next_up) &&
        (swing_leg < 2 || leg_info[other_side_leg[swing_leg][0]].one_step  || current_stair_count + 1 < leg_info[other_side_leg[swing_leg][0]].stair_count)) {
            leg_info[swing_leg].next_up = false;
            if (current_stair_count == leg_info[other_side_leg[swing_leg][1]].stair_count) {    //first swing leg
                leg_info[swing_leg].one_step = false;
                leg_info[swing_leg].next_foothold = {current_stair_edge[0] + keep_edge_d, current_stair_edge[1]};
            } else {    //second swing leg
                leg_info[swing_leg].one_step = true;
                double deepest_x;
                if (stair_edge[swing_leg].size() >= 2) {
                    deepest_x = stair_edge[swing_leg][1].edge[0] - keep_stair_d_min;
                } else {
                    deepest_x = INFINITY;
                }//end if else
                double next_max_foothold_x = leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length_up_stair / 2;
                if (next_max_foothold_x >= deepest_x) {
                    leg_info[swing_leg].next_foothold = {deepest_x, current_stair_edge[1]};
                } else {
                    leg_info[swing_leg].next_foothold = {next_max_foothold_x, current_stair_edge[1]};
                }//end if else
            }//end if else
            up_stair = true;
            next_stair = 1;
        } else {    // move on the same stair step
            leg_info[swing_leg].one_step = true;
            double deepest_x = current_stair_edge[0] - keep_stair_d_min;
            double next_max_foothold_x = leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length / 2;
            if (next_max_foothold_x >= deepest_x) {
                leg_info[swing_leg].next_foothold = {deepest_x, leg_info[swing_leg].foothold[1]};
            } else {
                leg_info[swing_leg].next_foothold = {next_max_foothold_x, leg_info[swing_leg].foothold[1]};
            }//end if else
        }//end if
        // determine if next swing leg will swing up to next stair step
        if (stair_edge[swing_leg].size() > next_stair && leg_info[swing_leg].next_foothold[0] >= stair_edge[swing_leg][next_stair].edge[0] - keep_stair_d_max) {
            leg_info[swing_leg].next_up = true;
        }//end if
    } else {    // move on the upper ground
        leg_info[swing_leg].one_step = true;
        leg_info[swing_leg].next_foothold = {leg_info[swing_leg].get_hip_position(CoM, pitch)[0] + step_length / 2, leg_info[swing_leg].foothold[1]};
        std::cout << "Leg " << swing_leg << " is on the upper ground." << std::endl;
    }//end if else

    return up_stair;
}//end determine_next_foothold

std::array<double, 2> StairClimb::get_foothold(double theta, double beta, int contact_rim) {
    leg_model.contact_map(theta, beta);
    if (contact_rim == -1) {
        return leg_model.contact_p;
    }//end if

    double radius = leg_model.radius;
    std::complex<double> center_beta0;
    switch (contact_rim) {
        case 1: // left upper rim
            center_beta0 = std::complex<double>(U_l_poly[0](theta), U_l_poly[1](theta));
            break;
        case 2: // left lower rim
            center_beta0 = std::complex<double>(L_l_poly[0](theta), L_l_poly[1](theta));
            break;
        case 3: // G
            center_beta0 = std::complex<double>(0.0, G_poly[1](theta));
            radius = leg_model.r;
            break;
        case 4: // right lower rim
            center_beta0 = std::complex<double>(L_r_poly[0](theta), L_r_poly[1](theta));
            break;
        case 5: // right upper rim
            center_beta0 = std::complex<double>(U_r_poly[0](theta), U_r_poly[1](theta));
            break;
        default:
            std::cerr << "ERROR IN get_foothold, contact_rim should be 1~5." << std::endl;
            return {0, 0};
    }//end switch

    std::complex<double> center_exp = center_beta0 * std::exp(std::complex<double>(0, beta));
    return {center_exp.real(), center_exp.imag() - radius};
}//end get_foothold

void StairClimb::update_hip() { // set last hip position as current hip position
    for (int i=0; i<4; i++) {
        last_hip[i] = hip[i];
    }//end for
}//end update_hip
