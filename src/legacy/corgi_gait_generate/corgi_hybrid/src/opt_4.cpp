#include "corgi_utils/leg_model.hpp"
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

// -------------------- Data Structure --------------------
struct DataPoint {
    double theta, beta, x, y;
    std::string phase;  // "swing"
};

// -------------------- Utility: find_pose --------------------
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

// -------------------- Utility: pointOnRimByAlpha --------------------
Eigen::Vector2d pointOnRimByAlpha(
    LegModel &leg,
    double theta,
    double beta,
    int rim,
    double alpha)
{
    // std::cout << "point receive rim = " << rim << std::endl;
    // std::cout << "point receive alpha = " << alpha << std::endl;

    leg.forward(theta, beta, false);
    const double deg50 = 50.0 * M_PI / 180.0;

    std::complex<double> P;

    if (rim == 3) {
        // G
        P = leg.L_l_c + (leg.r / leg.R) * std::polar(1.0, alpha) * (leg.G_c - leg.L_l_c);
    } else {
        if (-M_PI <= alpha && alpha < -deg50) {
            P = leg.U_l_c + (leg.radius / leg.R)  * std::polar(1.0, alpha + M_PI) * (leg.H_l_c - leg.U_l_c);
        } else if (-deg50 <= alpha && alpha < 0) {
            P = leg.L_l_c + (leg.radius / leg.R)  * std::polar(1.0, alpha + deg50) * (leg.F_l_c - leg.L_l_c);
        } else if (0 <= alpha && alpha < deg50) {
            P =  leg.L_r_c +  (leg.radius / leg.R) * std::polar(1.0, alpha) * (leg.G_c - leg.L_r_c);
        } else if (deg50 <= alpha && alpha < M_PI) {
            P = leg.U_r_c + (leg.radius / leg.R)  * std::polar(1.0, alpha - deg50) * (leg.F_r_c - leg.U_r_c);
        } else {
            throw std::runtime_error("invalid alpha value for given rim");
        }
    }

    return { P.real(), P.imag() };
}


// -------------------- Quintic time scaling --------------------
double quintic(double t) {
    return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t;
}

// -------------------- Cost: position error → θ,β (with height penalty) ----------------
double costFunctionPos(
    LegModel& leg,
    double theta, double beta,
    const Eigen::Vector2d& target,
    double theta_prev, double beta_prev,
    int rim, double alpha,
    double standHeight,
    double w_reg   = 1e-2,
    double w_theta = 1e3,
    double w_h     = 1e2
) {
    Eigen::Vector2d modelP = pointOnRimByAlpha(leg, theta, beta, rim, alpha);
    double cost = (modelP - target).squaredNorm();
    cost += w_reg * (std::pow(theta - theta_prev,2) + std::pow(beta - beta_prev,2));
    double tmin = 17*M_PI/180.0, tmax = 160*M_PI/180.0;
    if      (theta < tmin) cost += w_theta * std::pow(tmin - theta,2);
    else if (theta > tmax) cost += w_theta * std::pow(theta - tmax,2);
    if (modelP.y() < standHeight) {
        double dh = standHeight - modelP.y();
        cost += w_h * dh * dh;
    }
    return cost;
}

// -------------------- Solver: position → θ,β (with height constraint) ----------------
std::pair<double,double> solveThetaBetaForPoint(
    LegModel& leg,
    const Eigen::Vector2d& target,
    double theta_init, double beta_init,
    int rim, double alpha,
    double standHeight,
    int max_iters = 50,
    double lr = 1e-2,
    double tol = 1e-6
) {
    double theta = theta_init, beta = beta_init;
    for (int it = 0; it < max_iters; ++it) {
        double J = costFunctionPos(leg, theta, beta,
                                   target, theta_init, beta_init,
                                   rim, alpha, standHeight);
        double eps = 1e-6;
        double dtheta = (costFunctionPos(leg, theta+eps, beta,
                           target, theta_init, beta_init,
                           rim, alpha, standHeight) - J) / eps;
        double dbeta  = (costFunctionPos(leg, theta, beta+eps,
                           target, theta_init, beta_init,
                           rim, alpha, standHeight) - J) / eps;
        theta -= lr * dtheta;
        beta  -= lr * dbeta;
        if (std::abs(dtheta) + std::abs(dbeta) < tol) break;
    }
    return {theta, beta};
}

// -------------------- Cost: α error → θ,β ----------------
double costFunctionAlpha(
    LegModel& leg,
    double theta, double beta,
    double alpha_target,
    double theta_prev, double beta_prev,
    double slope,
    double w_reg   = 1e-2,
    double w_theta = 1e3
) {
    leg.forward(theta, beta, false);
    leg.contact_map(theta, beta, slope);
    double a_model = leg.alpha;
    double cost = std::pow(a_model - alpha_target,2)
                + w_reg * (std::pow(theta - theta_prev,2) + std::pow(beta - beta_prev,2));
    double tmin = 17*M_PI/180.0, tmax = 160*M_PI/180.0;
    if      (theta < tmin) cost += w_theta * std::pow(tmin - theta,2);
    else if (theta > tmax) cost += w_theta * std::pow(theta - tmax,2);
    return cost;
}

// -------------------- Solver: α → θ,β ----------------
std::pair<double,double> solveThetaBetaForAlpha(
    LegModel& leg,
    double alpha_target,
    double theta_init, double beta_init,
    double slope,
    int max_iters = 50,
    double lr = 1e-2,
    double tol = 1e-6
) {
    double theta = theta_init, beta = beta_init;
    for (int it = 0; it < max_iters; ++it) {
        double J = costFunctionAlpha(leg, theta, beta,
                                     alpha_target, theta_init, beta_init, slope);
        double eps = 1e-6;
        double dtheta = (costFunctionAlpha(leg, theta+eps, beta,
                           alpha_target, theta_init, beta_init, slope) - J) / eps;
        double dbeta  = (costFunctionAlpha(leg, theta, beta+eps,
                           alpha_target, theta_init, beta_init, slope) - J) / eps;
        theta -= lr * dtheta;
        beta  -= lr * dbeta;
        if (std::abs(dtheta) + std::abs(dbeta) < tol) break;
    }
    return {theta, beta};
}

// -------------------- CSV Output --------------------------------
void save_data_to_csv(
    const std::string& filename,
    const std::vector<DataPoint>& data)
{
    std::ofstream file(filename);
    file << "theta,beta,x,y,phase\n"
         << std::fixed << std::setprecision(6);
    for (auto &d : data) {
        file << d.theta << "," << d.beta << ","
             << d.x     << "," << d.y   << ","
             << d.phase << "\n";
    }
}

// -------------------- checkHeight ----------------
std::pair<double,double> checkHeight(
    LegModel& leg,
    double theta_in,
    double beta_in,
    double standHeight,
    int    rim,
    double alpha,
    double theta_prev,
    double beta_prev)
{
    leg.forward(theta_in, beta_in, false);
    leg.contact_map(theta_in, beta_in);
    double h = leg.contact_p[1];
    if (h >= standHeight) {
        return {theta_in, beta_in};
    }
    Eigen::Vector2d target{ leg.contact_p[0], standHeight };
    return solveThetaBetaForPoint(
        leg, target,
        theta_prev, beta_prev,
        rim, alpha,
        standHeight
    );
}


void generate_and_save_swing_trajectory(
    LegModel& leg,
    double theta_start, double beta_start,
    double theta_end,   double beta_end,
    int    N,
    int    rim,
    double alpha,
    const Eigen::Vector2d &body_vel,
    const Eigen::Vector2d &ground_tangent,
    double standHeight,
    double slope,
    double step_len,
    double swing_fraction,
    double vel,
    const std::string &filename)
{
    // 1) Compute actual swing duration and dr/dt
    double swing_dur = (step_len * swing_fraction) / vel;  // seconds
    double drdt      = 1.0 / swing_dur;                    // dr per second

    // 2) Setup quintic interpolators for theta and beta
    const double mid_theta   = 17.0 * M_PI / 180.0;
    const double max_dtheta  = 0.5  * M_PI / 180.0;
    const double lambda      = 0.3;
    auto theta_raw = [&](double r) {
        if (r < 0.5) {
            double u = r / 0.5;
            return theta_start + (mid_theta - theta_start) * quintic(u);
        } else {
            double u = (r - 0.5) / 0.5;
            return mid_theta + (theta_end - mid_theta) * quintic(u);
        }
    };
    auto beta_raw = [&](double r) {
        return beta_start + (beta_end - beta_start) * quintic(r);
    };

    // // 3) Compute displacement per r-step and true foot speed
    // double r4 = double(N - 1) / N;
    // Eigen::Vector2d P4 = pointOnRimByAlpha(leg, theta_raw(r4), beta_raw(r4), rim, alpha);
    // Eigen::Vector2d P5 = pointOnRimByAlpha(leg, theta_end,        beta_end,        rim, alpha);
    // double delta_p    = (P5 - P4).dot(ground_tangent);    // m per r-step
    // double vt         = body_vel.dot(ground_tangent);      // m/s along tangent
    // // foot speed = delta_p / (swing_dur/N) = delta_p * N / swing_dur = delta_p * N * drdt
    // double denom_time = delta_p * N * drdt;               // m/s foot speed

    // // 4) Compute time-warp factor eps
    // double raw_eps = (std::abs(denom_time) > 1e-6 ? (1.0 - vt / denom_time) : 0.0);
    // double eps     = std::clamp(raw_eps, -1.0, 1.0);
    // std::cout << std::fixed
    //           << "swing_dur="   << swing_dur   << "s, "
    //           << "delta_p="     << delta_p     << "m, "
    //           << "denom_time="  << denom_time  << "m/s, "
    //           << "vt="          << vt          << "m/s, "
    //           << "eps="         << eps         << "\n";

    // 5) Generate swing trajectory with warp
    std::vector<DataPoint> data;
    data.reserve(N+1);
    double prev_th = theta_start;
    double prev_b  = beta_start;

    for (int i = 0; i <= N; ++i) {
        double r_raw  = double(i) / N;
        // double r_warp = std::clamp(r_raw + eps * r_raw * (1.0 - r_raw), 0.0, 1.0);

        // Theta interpolation with rate limit
        // double tgt_th = theta_raw(r_warp);
        double tgt_th = theta_raw(r_raw);
        double dth    = tgt_th - prev_th;
        double theta_cur = prev_th + std::copysign(
            std::min(std::abs(dth), max_dtheta), dth);

        // Beta interpolation (warped)
        // double s_beta   = lambda * r_warp + (1.0 - lambda) * quintic(r_warp);
        // double beta_cur = beta_start + (beta_end - beta_start) * s_beta;
        double beta_cur = beta_raw(r_raw);

        // Height check on first 20%
        if (i < N / 5) {
            std::tie(theta_cur, beta_cur) = checkHeight(
                leg, theta_cur, beta_cur,
                -standHeight, rim, alpha,
                prev_th, prev_b
            );
        }

        Eigen::Vector2d p = pointOnRimByAlpha(
            leg, theta_cur, beta_cur, rim, alpha
        );
        data.push_back({theta_cur, beta_cur, p.x(), p.y(), "swing"});
        prev_th = theta_cur;
        prev_b  = beta_cur;
    }

    // // 6) Final alpha refinement for precise landing
    // auto [theta_f, beta_f] = solveThetaBetaForAlpha(
    //     leg, alpha,
    //     data.back().theta, data.back().beta,
    //     slope
    // );
    // Eigen::Vector2d pf = pointOnRimByAlpha(leg, theta_f, beta_f, rim, alpha);
    // data.back() = {theta_f, beta_f, pf.x(), pf.y(), "swing"};


    //     // 7) Velocity compensation on last M points (method one: time window)
    // double dt = swing_dur / N;            // actual sample interval
    // double T_ref = 0.05;                  // 50 ms refinement window
    // int M = std::min(N, (int)std::ceil(T_ref / dt));
    // std::cout << "Refinement window M=" << M << " points";
    // for (int j = 1; j <= M; ++j) {
    //     int idx = N - M + j;
    //     // shift along ground tangent proportionally to body velocity
    //     double offset = vt * dt * j;
    //     data[idx].x += ground_tangent.x() * offset;
    //     data[idx].y += ground_tangent.y() * offset;
    // }

    // // 8) Check final velocities along ground tangent
    // {
    //     Eigen::Vector2d p_prev(data[N-1].x, data[N-1].y);
    //     Eigen::Vector2d p_last(data[N].x,   data[N].y);
    //     double foot_vel = (p_last - p_prev).dot(ground_tangent) / dt;
    //     double body_vel_t = vt;
    //     std::cout << std::fixed
    //               << "Final foot speed=" << foot_vel << " m/s, "
    //               << "Body speed=" << body_vel_t << " m/s";
    // }

    // 9) Save trajectory to CSV
    save_data_to_csv(filename, data);
}

// // -------------------- Trajectory Generator ----------------
// // Generates a full swing trajectory, applying time warp to compensate body motion
// void generate_and_save_swing_trajectory(
//     LegModel& leg,
//     double theta_start, double beta_start,
//     double theta_end,   double beta_end,
//     int    N,
//     int    rim,
//     double alpha,
//     const Eigen::Vector2d &body_vel,
//     const Eigen::Vector2d &ground_tangent,
//     double standHeight,
//     double slope,
//     double step_len,
//     double swing_fraction,
//     double vel,
//     const std::string &filename)
// {
//     // 1) Compute actual swing duration [s] and dr/dt (unit-r per sec)
//     double swing_dur = (step_len * swing_fraction) / vel;  // [s]
//     double drdt      = 1.0 / swing_dur;                    // [1/s]

//     // 2) Precompute P4 (r=(N-1)/N) and P5 (r=1)
//     double r4 = double(N - 1) / N;
//     auto theta_raw = [&](double r) {
//         const double mid = 17.0 * M_PI / 180.0;
//         if (r < 0.5) return theta_start + (mid - theta_start) * (2.0 * r);
//         return mid + (theta_end - mid) * (2.0 * (r - 0.5));
//     };
//     auto beta_raw = [&](double r) {
//         return beta_start + (beta_end - beta_start) * quintic(r);
//     };
//     Eigen::Vector2d P4 = pointOnRimByAlpha(leg, theta_raw(r4), beta_raw(r4), rim, alpha);
//     Eigen::Vector2d P5 = pointOnRimByAlpha(leg, theta_end, beta_end, rim, alpha);

//     // 3) Compute foot displacement per r-step [m] and true foot speed [m/s]
//     double delta_p    = (P5 - P4).dot(ground_tangent);    // [m per r-step]
//     double vt         = body_vel.dot(ground_tangent);      // [m/s]
//     // Each r-step corresponds to Δt = swing_dur/N, so foot speed = delta_p / (swing_dur/N) = delta_p * N / swing_dur
//     double denom_time = delta_p * N * drdt;               // [m/s]

//     // 4) Compute time-warp factor eps so that final foot speed = vt
//     double raw_eps = (std::abs(denom_time) > 1e-6) ? (1.0 - vt / denom_time) : 0.0;
//     double eps     = std::clamp(raw_eps, -1.0, 1.0);
//     std::cout << std::fixed
//               << "swing_dur="   << swing_dur   << "s, "
//               << "delta_p="     << delta_p     << "m, "
//               << "denom_time="  << denom_time  << "m/s, "
//               << "vt="          << vt          << "m/s, "
//               << "eps="         << eps         << "\n";

//     // 5) Generate trajectory with warp applied over full swing
//     std::vector<DataPoint> data;
//     data.reserve(N + 1);
//     double prev_th = theta_start;
//     double prev_b  = beta_start;
//     const double max_dth = 1.0 * M_PI / 180.0;
//     const double lambda  = 0.3;

//     for (int i = 0; i <= N; ++i) {
//         double r_raw  = double(i) / N;
//         double r_warp = std::clamp(r_raw + eps * r_raw * (1.0 - r_raw), 0.0, 1.0);

//         // Theta interpolation with rate limit
//         double th_tgt   = theta_raw(r_warp);
//         double dth      = th_tgt - prev_th;
//         double theta_cur = prev_th + std::copysign(std::min(std::abs(dth), max_dth), dth);

//         // Beta interpolation (warped)
//         double s_beta   = lambda * r_warp + (1.0 - lambda) * quintic(r_warp);
//         double beta_cur = beta_start + (beta_end - beta_start) * s_beta;

//         // Early height check for first 20% points
//         if (i < N / 5) {
//             std::tie(theta_cur, beta_cur) = checkHeight(
//                 leg, theta_cur, beta_cur,
//                 -standHeight, rim, alpha,
//                 prev_th, prev_b
//             );
//         }

//         Eigen::Vector2d p = pointOnRimByAlpha(leg, theta_cur, beta_cur, rim, alpha);
//         data.push_back({theta_cur, beta_cur, p.x(), p.y(), "swing"});
//         prev_th = theta_cur;
//         prev_b  = beta_cur;
//     }

//     // 6) Final alpha refinement: precise landing
//     auto [theta_f, beta_f] = solveThetaBetaForAlpha(
//         leg, alpha,
//         data.back().theta, data.back().beta,
//         slope
//     );
//     Eigen::Vector2d pf = pointOnRimByAlpha(leg, theta_f, beta_f, rim, alpha);
//     data.back() = {theta_f, beta_f, pf.x(), pf.y(), "swing"};

//     // 7) Save to CSV
//     save_data_to_csv(filename, data);
// }



// -------------------- Main --------------------------------------
int main(){
    const double stand_h   = 0.129;
    const double step_len  = 0.3;
    const double swing_t   = 0.2;
    const double vel       = 0.2;
    
    const double slope     = -20 * M_PI / 180.0;
    const double dS  = vel / 1000;
    const double incre_duty =  dS / step_len;
    const int    N         = (swing_t)/incre_duty;
    double temp_height_terrain = -stand_h +(0.6*step_len)*tan(slope);
    double temp_height =  -temp_height_terrain;
    double clearance = 0.01; // 0.0015 m clearance


    LegModel leg(true);
    double duty = 1.0 - swing_t;
    auto start = find_pose(
        leg, stand_h, 0,
        (0.8*step_len/2) - (duty/(1-swing_t))*0.8*step_len,
        0
    );
    double theta_start = start[0], beta_start = -start[1];

    std::array<double, 2> new_theta_beta;
    double pos[2] = {0,temp_height_terrain+leg.r};
    new_theta_beta = leg.inverse(pos, "G");
    leg.forward(new_theta_beta[0], new_theta_beta[1],true);
    leg.contact_map(new_theta_beta[0], new_theta_beta[1],slope);
    double slope_line = (leg.contact_p[1] + stand_h) / (leg.contact_p[0] + 0.6 * step_len);
    std::cout << "slope_line = " << slope_line << std::endl;
    // for(int i=0;i<0.4*step_len*1000;i++){
    //     std::array<double, 2> result_eta;
    //     result_eta = leg.move(new_theta_beta[0], new_theta_beta[1], {0.001, 0}, -slope);
    //     new_theta_beta[0] = result_eta[0];
    //     new_theta_beta[1] = result_eta[1];
    // }

    for(int i=0;i<clearance*10000;i++){
        std::cout<<i<<std::endl;
        std::array<double, 2> result_eta;
        result_eta = leg.move(new_theta_beta[0], new_theta_beta[1], {-0.0001*sin(slope), -0.0001*cos(slope)}, 0);
        new_theta_beta[0] = result_eta[0];
        new_theta_beta[1] = result_eta[1];
    }

    // 1   -> 2   -> 3 -> 4 -> 5 -> 0: 
    // U_l -> L_l -> G -> L_r -> U_r -> None
    leg.forward(new_theta_beta[0], new_theta_beta[1],true);
    leg.contact_map(new_theta_beta[0], new_theta_beta[1],slope);
    auto end = new_theta_beta;
    double theta_end = end[0], beta_end = -end[1];
    // cout theta and beta
    while (beta_start < beta_end) beta_end -= 2.0*M_PI;
    end[1] = beta_end;
    leg.contact_map(theta_end, beta_end,slope);
    int rim_id = leg.rim;
    double alpha0 = leg.alpha;

    Eigen::Vector2d ground_tan(std::cos(slope), std::sin(slope));
    Eigen::Vector2d body_vel(vel*std::cos(slope), vel*std::sin(slope));

    generate_and_save_swing_trajectory(
        leg,
        theta_start, beta_start,
        theta_end,   beta_end,
        N,
        rim_id, alpha0,
        body_vel, ground_tan,
        stand_h, slope, step_len, swing_t, vel,
        "0523-1.csv"
    );

    // check velocity 相對靜止
    // add for ss
    // add clearence? (aroud  1 degree)

    return 0;
}
