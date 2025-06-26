#include "leg_model.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <stdexcept>
#include <complex>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>
#include <utility>


// -------------------- Utility --------------------
Eigen::Vector2d pointOnRimByAlpha(
    LegModel &legmodel,
    double theta, double beta,
    int rim, double alpha)
{
    legmodel.forward(theta, beta, false);
    std::complex<double> O, V0;
    switch (rim) {
        case 1: O = legmodel.U_l_c; V0 = legmodel.H_l_c - O; break;
        case 2: O = legmodel.L_l_c; V0 = legmodel.F_l_c - O; break;
        case 3: O = legmodel.L_l_c; V0 = legmodel.G_c   - O; break;
        case 4: O = legmodel.L_r_c; V0 = legmodel.F_r_c - O; break;
        case 5: O = legmodel.U_r_c; V0 = legmodel.H_r_c - O; break;
        default: throw std::runtime_error("invalid rim id");
    }
    const double deg50 = 50.0 * M_PI / 180.0;
    std::complex<double> rotated;
    if      (alpha >= -M_PI   && alpha < -deg50) rotated = std::polar(1.0, alpha + M_PI)  * V0;
    else if (alpha >= -deg50  && alpha <  0.0)   rotated = std::polar(1.0, alpha + deg50) * V0;
    else if (alpha >= 0.0     && alpha <  deg50) rotated = std::polar(1.0, alpha)        * V0;
    else if (alpha >= deg50   && alpha <  M_PI)  rotated = std::polar(1.0, alpha - deg50) * V0;
    else {
        auto VG = legmodel.G_c - legmodel.L_l_c;
        auto tip = legmodel.L_l_c + (legmodel.r / legmodel.R) * std::polar(1.0, alpha) * VG;
        return { tip.real(), tip.imag() };
    }
    auto P = O + (legmodel.radius / legmodel.R) * rotated;
    return { P.real(), P.imag() };
}

std::array<double,2> find_pose(
    LegModel& leg_model,
    double height,
    float shift,
    float steplength,
    double slope)
{
    auto pose = leg_model.inverse({0.0, -height + leg_model.r}, "G");
    double ds = (steplength >= 0 ? +0.001 : -0.001);
    double target = shift + steplength;
    for (double s = 0; (ds>0 ? s < target : s > target); s += ds) {
        pose = leg_model.move(pose[0], pose[1], {ds,0}, slope);
    }
    return pose;
}

void find_real_slope(LegModel& leg_model, double theta, double beta, double slope) {
    leg_model.forward(theta, beta, true);
    leg_model.contact_map(theta, beta, slope);
    auto end_point = leg_model.contact_p;
    // 1   -> 2   -> 3 -> 4 -> 5 -> 0: 
    // U_l -> L_l -> G -> L_r -> U_r -> None
    std::complex<double> center;
    switch (leg_model.rim) {
        case 1:
            leg_model.contact_map(theta, beta,0); 
            center = leg_model.U_l_c; 
            break;
        case 2: 
            leg_model.contact_map(theta, beta,0);
            center = leg_model.L_l_c; 
            break;
        case 3: 
            leg_model.contact_map(theta, beta,0);
            center = leg_model.G_c;   
            break;
        case 4: 
            leg_model.contact_map(theta, beta,0);
            center = leg_model.L_r_c; 
            break;
        case 5: 
            leg_model.contact_map(theta, beta,0);
            center = leg_model.U_r_c; 
            break;
        default:
            throw std::runtime_error("Invalid rim ID: " +
                                     std::to_string(leg_model.rim));
    }

    double slope_line = (end_point[1] - center.imag()) / (end_point[0] - center.real());
    double angle_perp = std::atan((-1/slope_line)) * 180.0 / M_PI;
    std::cout << "slope_line = " << angle_perp << std::endl;
}

double quintic(double t) { return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t; }

void generate_and_save_swing_trajectory(
    LegModel& leg,
    double theta_start, double beta_start,
    double theta_end,   double beta_end,
    int N,
    int rim, double alpha,
    const Eigen::Vector2d &body_vel,
    const Eigen::Vector2d &ground_tangent,
    std::ofstream& out)
{
    Eigen::Vector2d P_end = pointOnRimByAlpha(leg, theta_end, beta_end, rim, alpha);
    double raw_prev = double(N-1)/N;
    double mid_theta = 17.0 * M_PI / 180.0;
    auto compute_theta_raw = [&](double t){
        if (t < 0.5)
            return theta_start + (mid_theta - theta_start) * (2*t);
        else
            return mid_theta + (theta_end - mid_theta) * (2*(t - 0.5));
    };
    auto compute_beta_raw = [&](double t){ return beta_start + (beta_end - beta_start) * quintic(t); };
    Eigen::Vector2d P_prev = pointOnRimByAlpha(
        leg,
        compute_theta_raw(raw_prev),
        compute_beta_raw(raw_prev),
        rim, alpha
    );
     Eigen::Vector2d dPdr = (P_end - P_prev) * N;
    double denom = dPdr.dot(ground_tangent);
    double vb_t  = body_vel.dot(ground_tangent);
    double eps   = (std::abs(denom) > 1e-6 ? 1.0 - vb_t/denom : 0.0);
    eps = std::clamp(eps, -0.5, 0.5);

    // 2) warp for theta only
    auto warp_theta = [&](double r){ return std::clamp(r + eps*r*(1-r), 0.0, 1.0); };

    // constraints
    const double max_dtheta = 1.0 * M_PI/180.0; // max 1° per step
    const double lambda     = 0.3;              // beta mix weight

    double prev_theta = theta_start;

    // 3) generate trajectory
    for (int i = 0; i <= N; ++i) {
        double raw = double(i) / N;
        // theta: warped two-segment + clamp delta
        double t_th    = warp_theta(raw);
        double theta_uns = compute_theta_raw(t_th);
        double dtheta_val = theta_uns - prev_theta;
        double theta_cur  = prev_theta + std::copysign(std::min(std::abs(dtheta_val), max_dtheta), dtheta_val);

        // beta: linear+quintic on raw (no warp)
        double s_lin   = raw;
        double s_quint = quintic(raw);
        double s_beta  = lambda*s_lin + (1.0 - lambda)*s_quint;
        double beta_cur = beta_start + (beta_end - beta_start) * s_beta;

        Eigen::Vector2d p = pointOnRimByAlpha(leg, theta_cur, beta_cur, rim, alpha);
        out << std::fixed << std::setprecision(6) 
            << theta_cur << "," 
            << beta_cur << "," 
            << p.x()<<"," 
            << p.y()<<","
            << "swing"<<"\n";
        
        prev_theta = theta_cur;
    }
}
// -------------------- Main --------------------------------------
int main(){
    const double stand_h   = 0.159;
    const double step_len  = 0.3;
    const double swing_t   = 0.2;
    const double vel       = 0.2;
    const double slope     = 10.0 * M_PI / 180.0;
    LegModel leg(true);

    // pure check of slope calculation
    std::array<double, 2> new_theta_beta;
    std::array<double, 2> new_theta_beta2;
    double pos[2] = {0,-stand_h+leg.r};
    new_theta_beta = leg.inverse(pos, "G");
    find_real_slope(leg, new_theta_beta[0], new_theta_beta[1], slope);

    // test of pose (if the pose is correct the swing traj will be correct)
    // without clearence
    // 1- -ss-sl = TDF ....
    std::ofstream file2("test9.csv");
    double shift = 0.05;
    new_theta_beta = leg.inverse(pos, "G");
    
    for (double i=0;i<shift;i=i+0.001){
        new_theta_beta = leg.move(new_theta_beta[0], new_theta_beta[1], {-0.001,0}, 0);
        // store theta, beta in "test.csv"
        if(new_theta_beta[0]>170* M_PI / 180.0 || new_theta_beta[0]<16* M_PI / 180.0){
            std::cerr << "theta out of range: " << new_theta_beta[0] << std::endl;
            break;
        }     
        new_theta_beta2 = new_theta_beta;
        
    }
    for (double i=0;i<0.5*step_len;i=i+0.001){
        new_theta_beta = leg.move(new_theta_beta[0], new_theta_beta[1], {-0.001,0}, slope);
        // store theta, beta in "test.csv"
        if(new_theta_beta[0]>170* M_PI / 180.0 || new_theta_beta[0]<16* M_PI / 180.0){
            std::cerr << "theta out of range: " << new_theta_beta[0] << std::endl;
            break;
        }     
        new_theta_beta2 = new_theta_beta;
    }
    // touchdown
    for (double i=0;i<=(1-swing_t);i=i+((vel/1000)/step_len)){
            new_theta_beta = leg.move(new_theta_beta[0], new_theta_beta[1], {0.0002,0}, slope);
            if(new_theta_beta[0]>170* M_PI / 180.0 || new_theta_beta[0]<16* M_PI / 180.0){
                std::cerr << "theta out of range: " << new_theta_beta[0] << std::endl;
                break;
            }
            else{
                new_theta_beta2 = new_theta_beta;
                leg.contact_map(new_theta_beta[0], new_theta_beta[1], slope);
                if (file2.is_open()) {
                file2 << std::fixed << std::setprecision(6) 
                    << new_theta_beta[0] << "," 
                    << new_theta_beta[1] << "," 
                    << leg.contact_p[0]<<"," 
                    << leg.contact_p[1]<<","
                    << "touchdown"<<"\n";
                } else {
                    std::cerr << "Unable to open file for writing." << std::endl;
                }

            }
    }
    // swing
    auto start = new_theta_beta2;
    double th0 = start[0], bt0 = start[1];
    new_theta_beta = leg.inverse(pos, "G");
    shift = 0.1;
    for (double i=0;i<shift;i=i+0.001){
        // other side
        new_theta_beta = leg.move(new_theta_beta[0], new_theta_beta[1], {0.001,0}, 0);
        // store theta, beta in "test.csv"
        if(new_theta_beta[0]>170* M_PI / 180.0 || new_theta_beta[0]<16* M_PI / 180.0){
            std::cerr << "theta out of range: " << new_theta_beta[0] << std::endl;
            break;
        }     
        new_theta_beta2 = new_theta_beta;
    }
    for (double i=0;i<0.5*step_len;i=i+0.001){
        new_theta_beta = leg.move(new_theta_beta[0], new_theta_beta[1], {-0.001,0}, -slope);
        if(new_theta_beta[0]>170* M_PI / 180.0 || new_theta_beta[0]<16* M_PI / 180.0){
            std::cerr << "theta out of range: " << new_theta_beta[0] << std::endl;
            break;
        }     
        new_theta_beta2 = new_theta_beta;
    }
    auto end = new_theta_beta2;
    double th5 = end[0], bt5 = end[1];
    while (bt0 < bt5) bt5 -= 2*M_PI;

    leg.forward(th5, bt5, false);
    leg.contact_map(th5, bt5, -slope);
    find_real_slope(leg, th5, bt5, -slope);
    int rim_id = leg.rim;
    double alpha0 = leg.alpha;

    generate_and_save_swing_trajectory(
        leg, th0, bt0, th5, bt5,
        200, rim_id, alpha0,
        Eigen::Vector2d(vel,0), Eigen::Vector2d(1,0),
        file2
    );
    // touchdown2
    for (double i=0;i<=(1-swing_t);i=i+((vel/1000)/step_len)){
        new_theta_beta = leg.move(new_theta_beta[0], new_theta_beta[1], {0.0002,0}, -slope);
        if(new_theta_beta[0]>170* M_PI / 180.0 || new_theta_beta[0]<16* M_PI / 180.0){
            std::cerr << "theta out of range: " << new_theta_beta[0] << std::endl;
            break;
        }
        else{
            new_theta_beta2 = new_theta_beta;
            leg.contact_map(new_theta_beta[0], new_theta_beta[1], -slope);
            if (file2.is_open()) {
            file2 << std::fixed << std::setprecision(6) 
                << new_theta_beta[0] << "," 
                << new_theta_beta[1] << "," 
                << leg.contact_p[0]<<"," 
                << leg.contact_p[1]<<","
                << "touchdown2"<<"\n";
            } else {
                std::cerr << "Unable to open file for writing." << std::endl;
            }

        }
    }
    file2.close();

    

    
    // 試試看如果先用對稱找到forward -> -beta -> leg.move() -> csv -> ani
    // try mapping the limitation of each height?   
    // check velocity 相對靜止
    // add for ss
    // add clearence? (aroud  1 degree)

    return 0;
}
