#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "leg_model.hpp"
#include "stair_climb.hpp"

StairClimb::StairClimb(bool sim, double CoM_bias, int rate, double BL, double BW, double BH) : 
    /* Initializer List */
    leg_model(sim),
    CoM_bias(CoM_bias),
    rate(rate), 
    BL(BL),
    BW(BW),
    BH(BH)
{
    // Initialize positions
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
    int first_swing_leg;
    if (relative_foothold[0][0] < relative_foothold[1][0]) {
        first_swing_leg = 0;
    } else {
        first_swing_leg = 1;
    }//end if else
    // Get foothold in world coordinate
    CoM = {0, stand_height};
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;
    // Initial leg configuration
    for (int i=0; i<4; i++) {
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0], next_hip[i][1] + relative_foothold[i][1]};
    }//end for
    // Initial theta/beta
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i]  = init_beta[i];
    }//end for
}//end initialize

std::array<std::array<double, 4>, 2> StairClimb::step() {
    // state machine
    switch (this->state) {
        case MOVE_STABLE:
            if (!move_CoM_stable()) {
                this->state = SWING_SAME;
            }//end if
            break;
        case SWING_SAME:
            swing_same_step(leg_info, swing_leg, CoM, pitch, stairs_edge[swing_leg][1], stairs_edge[swing_leg][0]);
            this->state = SWING_NEXT;
            break;
        case SWING_NEXT:
            if (determine_next_foothold(leg_info, swing_leg, stairs_edge.size(), stairs_edge, step_length, step_length_up_stair, keep_edge_distance, keep_stair_distance_front, keep_stair_distance_hind, keep_stair_distance_all, CoM, pitch)) {
                swing_next_step(leg_info, swing_leg, CoM, pitch, stairs_edge[swing_leg][1], stairs_edge[swing_leg][0]);
            } else {
                swing_same_step(leg_info, swing_leg, CoM, pitch, stairs_edge[swing_leg][1], stairs_edge[swing_leg][0]);
            }//end if else
            break;
        default:
            break;
    }//end switch

    // next state
    switch (this->state) {
        case MOVE_STABLE:
            break;
        case SWING_SAME:
            break;
        case SWING_NEXT:
            break;
        default:
            break;
    }//end switch

    return {theta, beta};
}//end step


bool StairClimb::move_CoM_stable() {
    int move_dir = (leg_info[swing_leg].ID == 0 || leg_info[swing_leg].ID == 1) ? -1 : 1;
    std::array<double, 2> CoM_offset = { std::cos(pitch) * CoM_bias[0] - std::sin(pitch) * CoM_bias[1],
                                         std::sin(pitch) * CoM_bias[0] + std::cos(pitch) * CoM_bias[1] };
    
    // change velocity 
    if (move_dir * velocity[0] < max_velocity) {
        velocity[0] += move_dir * acc / rate;
    }//end if
    CoM[0] += velocity[0] / rate;
    // leg command
    for (int i=0; i<4; i++) {
        std::array<double, 2> next_hip = leg_info[i].get_hip_position(CoM, pitch);
        result_eta = move_consider_edge(i);
    }//end for

    // return if stable (entering support triangle)
    if (move_dir * (CoM[0] + CoM_offset[0]) < move_dir * ((leg_info[(swing_leg+1)%4].foothold[0] + leg_info[(swing_leg-1)%4].foothold[0]) / 2) + stability_margin) {
        return false;
    } else {
        return true;
    }//end if else
}//end move_CoM_stable

void StairClimb::move_CoM_stable_smooth_fixed_leg_length(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch) {
    Eigen::Matrix2d rotation;
    rotation << std::cos(pitch), -std::sin(pitch),
                std::sin(pitch),  std::cos(pitch);
    Eigen::Vector2d CoM_offset = rotation * CoM_bias;

    int move_dir = (leg_info[swing_leg].ID == 0 || leg_info[swing_leg].ID == 1) ? -1 : 1;
    LegModel leg_model;
    leg_model.forward(0.0, 0.0); // 初始化腿部模型
    double leg_length = leg_model.G.norm();
    
    while (move_dir * (CoM[0] + CoM_offset[0]) < move_dir * ((leg_info[(swing_leg + 1) % 4].foothold[0] + leg_info[(swing_leg - 1) % 4].foothold[0]) / 2) + stability_margin) {
        if (move_dir * velocity[0] < max_velocity) {
            velocity[0] += move_dir * acc / sampling;
        }//end if
        CoM += velocity / sampling;
        
        double hip_x = CoM[0];
        double hip_y;
        leg_model.forward(0.0, 0.0);
        if (leg_info[swing_leg].contact_edge) {
            hip_y = CoM[1];
        } else {
            hip_y = CoM[1] + leg_model.G[1] + std::sqrt(leg_length * leg_length - std::pow(hip_x - (CoM[0] + leg_model.G[0]), 2));
        }//end if else
        
        Eigen::Vector2d hip = Eigen::Vector2d(hip_x, hip_y);
        std::cout << "Hip position for swing leg " << swing_leg << ": " << hip.transpose() << std::endl;
    }//end while
    CoM = Eigen::Vector2d(CoM[0], CoM[1]);
    pitch = std::asin((CoM[1] - CoM[1]) / BL);
}//end move_CoM_stable_fixed_leg_length

void StairClimb::swing_same_step(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch, double front_height, double hind_height) {
    double final_CoM_height = (front_height + hind_height) / 2;
    double margin_d = std::abs(CoM[0] - (leg_info[(swing_leg+1)%4].foothold[0] + leg_info[(swing_leg-1)%4].foothold[0]) / 2) - min_margin;
    
    Eigen::Vector2d p_lo = leg_info[swing_leg].hip_position(CoM, pitch); 
    Eigen::Vector2d p_td = leg_info[swing_leg].next_foothold;
    SwingProfile sp(p_td[0] - p_lo[0], 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, p_lo[0], p_lo[1], p_td[1] - p_lo[1]);
    
    double t_f_x = margin_d / max_velocity;
    double coeff_b = acc;
    double coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    double t_f_y = -coeff_b / coeff_a;
    double t_f = std::max({min_swing_time_step, t_f_x, t_f_y});
    double local_max_velocity = margin_d / t_f;
    int samples = static_cast<int>(t_f * sampling);
    
    for (int sample = 0; sample < samples; ++sample) {
        if (velocity[0] < local_max_velocity) {
            velocity[0] += acc / sampling;
        } else if (velocity[0] > local_max_velocity + acc / sampling) {
            velocity[0] -= acc / sampling;
        }
        
        double t_ = static_cast<double>(sample+1) / sampling;
        velocity[1] = (t_ <= t_f_y) ? coeff_a * (t_ * t_) + coeff_b * t_ : 0.0;
        CoM += velocity / sampling;
        
        if (swing_leg == 0 || swing_leg == 1) {
            pitch = std::asin((CoM[1] - hind_height) / (BL / 2));
        } else {
            pitch = std::asin((front_height - CoM[1]) / (BL / 2));
        }
        
        Eigen::Vector2d hip = leg_info[swing_leg].hip_position(CoM, pitch);
        double swing_phase_ratio = static_cast<double>(sample+1) / samples;
        Eigen::Vector2d curve_point = sp.getFootendPoint(swing_phase_ratio);
        
        std::cout << "Swing leg " << swing_leg << " moving to: " << curve_point.transpose() << std::endl;
    }
    velocity[1] = 0.0;
}//end swing_same_step

void StairClimb::swing_next_step(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch, double front_height, double hind_height) {
    double lift_height = 0.04;
    bool is_clockwise = (swing_leg == 0 || swing_leg == 1) ? (leg_info[0].ID == leg_info[1].ID) : (leg_info[2].ID == leg_info[3].ID);
    double final_CoM_height = (front_height + hind_height) / 2;
    
    double t_f = is_clockwise ? min_swing_time_cw : min_swing_time_ccw;
    int samples = static_cast<int>(t_f * sampling);
    
    double t_f_x = std::abs(velocity[0] / acc);
    double coeff_b = acc;
    double coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    double t_f_y = -coeff_b / coeff_a;
    
    LegModel leg_model;
    leg_model.forward(0.0, 0.0);
    Eigen::Vector2d final_CoM = Eigen::Vector2d(CoM[0] + velocity[0] * t_f_x - std::copysign(acc * (t_f_x * t_f_x) / 2, velocity[0]), final_CoM_height);
    double final_pitch = std::asin((front_height - hind_height) / BL);
    Eigen::Vector2d final_hip = leg_info[swing_leg].hip_position(final_CoM, final_pitch);
    
    for (int sample = 0; sample < samples; ++sample) {
        if (velocity[0] > acc / sampling) {
            velocity[0] -= acc / sampling;
        } else if (velocity[0] < -acc / sampling) {
            velocity[0] += acc / sampling;
        } else {
            velocity[0] = 0.0;
        }
        
        double t_ = static_cast<double>(sample + 1) / sampling;
        velocity[1] = (t_ <= t_f_y) ? coeff_a * (t_ * t_) + coeff_b * t_ : 0.0;
        CoM += velocity / sampling;
        
        if (swing_leg == 0 || swing_leg == 1) {
            pitch = std::asin((CoM[1] - hind_height) / (BL / 2));
        } else {
            pitch = std::asin((front_height - CoM[1]) / (BL / 2));
        }
        
        Eigen::Vector2d hip = leg_info[swing_leg].hip_position(CoM, pitch);
        leg_model.inverse(final_hip - hip, "G");
        std::cout << "Swing leg " << swing_leg << " moving to: " << hip.transpose() << std::endl;
    }
    velocity[1] = 0.0;
}//end swing_next_step

std::array<double, 2> StairClimb::move_consider_edge(int leg_ID) {
    std::array<double, 2> current_stair_edge = stairs_edge[leg_ID].back();
    leg_model.forward(theta[leg_ID], beta[leg_ID]);
    double err = 0.01;
    
    Eigen::Vector2d edge_U_vec(hip[0]+leg_model.U_r[0]-current_stair_edge[0], hip[1]+leg_model.U_r[1]-current_stair_edge[1]);
    if (!leg_info[leg_ID].contact_edge && (hip[0]+leg_model.U_r[0] <= current_stair_edge[0]) && (edge_U_vec.norm() <= leg_model.radius)) {
        leg_info[leg_ID].contact_edge = true;
        leg_model.forward(theta[leg_ID], beta[leg_ID], false);
        std::complex<double> current_stair_edge_c(current_stair_edge[0], current_stair_edge[1]);
        std::complex<double> hip_c(hip[0], hip[1]);
        leg_info[leg_ID].contact_alpha = std::arg((current_stair_edge_c - hip_c - leg_model.U_r_c) / (leg_model.F_r_c-leg_model.U_r_c));
    } else if (leg_info[leg_ID].contact_edge && (hip[0]+leg_model.U_r[0]>current_stair_edge[0] || edge_U_vec.norm() > leg_model.radius + err)) {
        leg_info[leg_ID].contact_edge = false;
    }//end if else
    
    if (leg_info[leg_ID].contact_edge) {
        result_eta = move_edge(theta[leg_ID], beta[leg_ID], {current_stair_edge[0]-hip[0], current_stair_edge[1]-hip[1]}, leg_info[leg_ID].contact_alpha);
        leg_info[leg_ID].foothold = {current_stair_edge[0], current_stair_edge[1]};
    } else {
        if (hip[0] + leg_model.U_r[0] > current_stair_edge[0]) {
            result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], {hip[0]-last_hip[leg_ID][0], hip[1]-last_hip[leg_ID][1]}, true, false);
            std::array<double, 2> relative_foothold = get_foothold(theta, beta, 5);
        } else {
            result_eta = leg_model.move(theta[leg_ID], beta[leg_ID], {hip[0]-last_hip[leg_ID][0], hip[1]-last_hip[leg_ID][1]}, false);
            std::array<double, 2> relative_foothold = get_foothold(theta, beta);
        }//end if else
        leg_info[swing_leg].foothold = {hip[0] + relative_foothold[0], hip[1] + relative_foothold[1]};
    }//end if else

    return result_eta;
}//end move_consider_edge

std::array<double, 2> StairClimb::move_edge(double theta, double beta, std::array<double, 2> contact_p, double contact_alpha, double tol, size_t max_iter) {
    leg_model.forward(theta, beta);
    std::array<double, 2> init_U = leg_model.U_r;
    
    // Use optimization solver to find d_x and d_y of init_U (analogous to fsolve)
    double guess_dx = 0.0;    // initial guess = 0
    for (size_t iter = 0; iter < max_iter; ++iter) {
        double cost = this->objective_edge(guess_dx, init_U, contact_p, contact_alpha);     // 计算当前函数值
        if (cost < tol) {                // 判断收敛
            //std::cout << "Converged after " << iter << " iterations.\n";
            break;
        }//end if

        // computeJacobian, 数值计算雅可比矩阵
        double epsilon = 1e-6;
        double dx_eps = guess_dx + epsilon;
        double cost_eps = this->objective_edge(dx_eps, init_U, contact_p, contact_alpha);
        double cost_d = (cost_eps - cost) / epsilon;  // 数值差分计算导数

        double dx = -cost / cost_d;   // 解线性方程 cost_d * dx = -cost
        if (dx < tol) {             // 判断步长是否足够小
            //std::cout << "Converged after " << iter << " iterations.\n";
            break;
        }//end if

        // 更新解
        guess_dx += dx;

        if (iter == max_iter-1) {
            throw std::runtime_error("Newton solver did not converge.");
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

    std::complex<double> contact_p_c(contact_p[0], contact_p[1]);
    double new_alpha = std::arg((contact_p_c - leg_model.U_r_c) / (leg_model.F_r_c - leg_model.U_r_c));
    return new_alpha - contact_alpha;
}//end objective_edge

bool determine_next_foothold(std::vector<LegInfo>& leg_info, int swing_leg, int n_stairs, const std::vector<Vec2>& stairs_edge,
    double step_length, double step_length_up_stair, double keep_edge_distance, double keep_stair_distance_front, 
    double keep_stair_distance_hind, double keep_stair_distance_all, Vec2& CoM, double pitch) {

    bool up_stair = false;
    int current_stair = leg_info[swing_leg].stair;
    int next_stair = current_stair;
    int other_side_leg = (swing_leg % 2 == 0) ? swing_leg + 1 : swing_leg - 1;
    int front_hind_side_leg = (swing_leg < 2) ? 2 + swing_leg : swing_leg - 2;
    double keep_stair_d = (swing_leg < 2) ? keep_stair_distance_front : keep_stair_distance_hind;

    if (current_stair <= n_stairs) {
    if ((leg_info[swing_leg].next_up || leg_info[other_side_leg].next_up) &&
    (current_stair + 1 < leg_info[front_hind_side_leg].stair || leg_info[front_hind_side_leg].one_step || swing_leg < 2)) {
    leg_info[swing_leg].next_up = false;
    if (current_stair == leg_info[other_side_leg].stair) {
    leg_info[swing_leg].one_step = false;
    leg_info[swing_leg].next_foothold = {stairs_edge[current_stair][0] + keep_edge_distance, stairs_edge[current_stair][1]};
    } else {
    leg_info[swing_leg].one_step = true;
    double deepest_x = stairs_edge[current_stair + 1][0] - keep_stair_d;
    if (leg_info[swing_leg].hip_position(CoM, pitch)[0] + step_length_up_stair / 2 >= deepest_x) {
    leg_info[swing_leg].next_foothold = {deepest_x, stairs_edge[current_stair][1]};
    } else {
    leg_info[swing_leg].next_foothold = {leg_info[swing_leg].hip_position(CoM, pitch)[0] + step_length_up_stair / 2, stairs_edge[current_stair][1]};
    }
    }
    up_stair = true;
    next_stair = current_stair + 1;
    } else {
    leg_info[swing_leg].one_step = true;
    double deepest_x = stairs_edge[current_stair][0] - keep_stair_d;
    if (leg_info[swing_leg].hip_position(CoM, pitch)[0] + step_length / 2 >= deepest_x) {
    leg_info[swing_leg].next_foothold = {deepest_x, stairs_edge[current_stair][1]};
    } else {
    leg_info[swing_leg].next_foothold = {leg_info[swing_leg].hip_position(CoM, pitch)[0] + step_length / 2, stairs_edge[current_stair][1]};
    }
    }
    } else {
    leg_info[swing_leg].one_step = true;
    leg_info[swing_leg].next_foothold = {leg_info[swing_leg].hip_position(CoM, pitch)[0] + step_length / 2, stairs_edge.back()[1]};
    }

    if (leg_info[swing_leg].next_foothold[0] >= stairs_edge[next_stair][0] - keep_stair_distance_all) {
    leg_info[swing_leg].next_up = true;
    }
    return up_stair;
}//end determine_next_foothold

std::array<double, 2> StairClimb::get_foothold(double theta, double beta, int contact_rim = -1) {
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


void StairClimb::add_stair_edge(double x, double y) {
    stair_edge[0].push_back({x, y});
    stair_edge[1].push_back({x, y});
    stair_edge[2].push_back({x, y});
    stair_edge[3].push_back({x, y});
}//end add_stair_edge