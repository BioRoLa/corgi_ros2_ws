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
#include <Eigen/Dense>
#include <utility>

// -------------------- Data Structure --------------------
struct DataPoint {
    double theta;
    double beta;
    double x;
    double y;
    std::string phase; // "swing"
};

// -------------------- Utility: find_pose --------------------
std::array<double,2> find_pose(LegModel& leg_model,
    double height, float shift, float steplength, double slope)
{
    auto pose = leg_model.inverse({0.0, -height + leg_model.r}, "G");
    double ds = (steplength >= 0 ? +0.001 : -0.001);
    double target = shift + steplength;
    for (double s = 0; (ds>0 ? s < target : s > target); s += ds) {
        pose = leg_model.move(pose[0], pose[1], {ds,0}, slope);
    }
    return pose;
}

// -------------------- Utility: rim point by alpha ---------------
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

// -------------------- Time Scaling (quintic) --------------------
double quintic(double t) { return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t; }

// -------------------- Cost: position error → θ,β ----------------
double costFunctionPos(
    LegModel& leg,
    double theta, double beta,
    const Eigen::Vector2d& target,
    double theta_prev, double beta_prev,
    int rim, double alpha,
    double w_reg = 1e-2,
    double w_theta = 1e3
) {
    Eigen::Vector2d modelP = pointOnRimByAlpha(leg, theta, beta, rim, alpha);
    Eigen::Vector2d diff = modelP - target;
    double cost = diff.squaredNorm();
    cost += w_reg * (std::pow(theta - theta_prev,2) + std::pow(beta - beta_prev,2));
    double tmin = 17*M_PI/180.0, tmax = 160*M_PI/180.0;
    if (theta < tmin) cost += w_theta * std::pow(tmin - theta,2);
    if (theta > tmax) cost += w_theta * std::pow(theta - tmax,2);
    return cost;
}

// -------------------- Solver: position → θ,β -----------------
std::pair<double,double> solveThetaBetaForPoint(
    LegModel& leg,
    const Eigen::Vector2d& target,
    double theta_init, double beta_init,
    int rim, double alpha,
    int max_iters = 50,
    double lr = 1e-2,
    double tol = 1e-6
) {
    double theta = theta_init, beta = beta_init;
    for (int i = 0; i < max_iters; ++i) {
        double J = costFunctionPos(leg, theta, beta, target, theta_init, beta_init, rim, alpha);
        double eps = 1e-6;
        double dtheta = (costFunctionPos(leg, theta+eps, beta, target, theta_init, beta_init, rim, alpha) - J) / eps;
        double dbeta  = (costFunctionPos(leg, theta, beta+eps, target, theta_init, beta_init, rim, alpha) - J) / eps;
        theta -= lr * dtheta;
        beta  -= lr * dbeta;
        if (std::abs(dtheta)+std::abs(dbeta) < tol) break;
    }
    return {theta, beta};
}

// -------------------- Cost: α error → θ,β ----------------
double costFunctionAlpha(
    LegModel& leg,
    double theta, double beta,
    double alpha_target,
    double theta_prev, double beta_prev,
    double w_reg = 1e-2,
    double w_theta = 1e3
) {
    leg.forward(theta, beta, false);
    leg.contact_map(theta, beta);
    double alpha_model = leg.alpha;
    double e_alpha = alpha_model - alpha_target;
    double e_theta = theta - theta_prev;
    double e_beta  = beta  - beta_prev;
    double cost = e_alpha*e_alpha + w_reg * (e_theta*e_theta + e_beta*e_beta);
    double tmin = 17*M_PI/180.0, tmax = 160*M_PI/180.0;
    if (theta < tmin) cost += w_theta * std::pow(tmin - theta,2);
    if (theta > tmax) cost += w_theta * std::pow(theta - tmax,2);
    return cost;
}

// -------------------- Solver: α → θ,β -----------------
std::pair<double,double> solveThetaBetaForAlpha(
    LegModel& leg,
    double alpha_target,
    double theta_init, double beta_init,
    int max_iters = 50,
    double lr = 1e-2,
    double tol = 1e-6
) {
    double theta = theta_init, beta = beta_init;
    for (int i = 0; i < max_iters; ++i) {
        double J = costFunctionAlpha(leg, theta, beta, alpha_target, theta_init, beta_init);
        double eps = 1e-6;
        double dtheta = (costFunctionAlpha(leg, theta+eps, beta, alpha_target, theta_init, beta_init) - J) / eps;
        double dbeta  = (costFunctionAlpha(leg, theta, beta+eps, alpha_target, theta_init, beta_init) - J) / eps;
        theta -= lr * dtheta;
        beta  -= lr * dbeta;
        if (std::abs(dtheta)+std::abs(dbeta) < tol) break;
    }
    return {theta, beta};
}

// -------------------- CSV Output --------------------------------
void save_data_to_csv(const std::string& filename, const std::vector<DataPoint>& data) {
    std::ofstream file(filename);
    file << "theta,beta,x,y,phase\n";
    file << std::fixed << std::setprecision(6);
    for (auto &d: data) file << d.theta << "," << d.beta
                           << "," << d.x << "," << d.y
                           << "," << d.phase << "\n";
}

// ----- Trajectory Generator: Theta & Beta planning + Velocity + Final alpha refine -----
void generate_and_save_swing_trajectory(
    LegModel& leg,
    double theta_start, double beta_start,
    double theta_end,   double beta_end,
    int N,
    int rim, double alpha,
    const Eigen::Vector2d &body_vel,
    const Eigen::Vector2d &ground_tangent,
    const std::string &filename)
{
    std::vector<DataPoint> data;
    data.reserve(N+1);

    // 1) Precompute landing compensation epsilon
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
        DataPoint dp{theta_cur, beta_cur, p.x(), p.y(), "swing"};
        data.push_back(dp);
        prev_theta = theta_cur;
    }

    // 4) final refine by alpha optimization
    auto pair_ref = solveThetaBetaForAlpha(leg, alpha,
        data[N-1].theta, data[N-1].beta);
    double theta_ref = pair_ref.first;
    double beta_ref  = pair_ref.second;
    Eigen::Vector2d p_ref = pointOnRimByAlpha(leg, theta_ref, beta_ref, rim, alpha);
    data.back() = DataPoint{theta_ref, beta_ref, p_ref.x(), p_ref.y(), "swing"};

    // 5) save
    save_data_to_csv(filename, data);
}

// -------------------- Main -------------------------------------- --------------------------------------
int main(){
    const double stand_h=0.159, step_len=0.4, swing_t=0.2, vel=0.2;
    LegModel leg(true);

    auto start = find_pose(leg, stand_h, -0.03,
        (step_len/2) - ((1-swing_t)>0 ? vel/(1-swing_t) : 0)*step_len, 0);
    double th0 = start[0], bt0 = -start[1];

    auto end = find_pose(leg, stand_h, -0.05, (step_len/2), 0);
    double th5 = end[0], bt5 = -end[1];
    while (bt0 < bt5) bt5 -= 2*M_PI;

    leg.forward(th5, bt5, false);
    leg.contact_map(th5, bt5);
    int rim_id = leg.rim;
    double alpha0 = leg.alpha;

    generate_and_save_swing_trajectory(
        leg, th0, bt0, th5, bt5,
        200, rim_id, alpha0,
        Eigen::Vector2d(vel,0), Eigen::Vector2d(1,0),
        "trajectory3.csv"
    );
    std::cout<<"Saved swing trajectory to trajectory.csv\n";
    return 0;
}
