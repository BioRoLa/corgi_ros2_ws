#include "hybrid_swing.hpp"

double HybridSwing::quintic(double t) { return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t; }
double HybridSwing::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}
// -------------------- Utility: rim point by alpha ---------------
Eigen::Vector2d HybridSwing::pointOnRimByAlpha(
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

// -------------------- Cost: α error → θ,β ----------------
double HybridSwing::costFunctionAlpha(
    LegModel& leg,
    double theta, double beta,
    double alpha_target,
    double theta_prev, double beta_prev,
    double w_reg,
    double w_theta
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
std::pair<double,double> HybridSwing::solveThetaBetaForAlpha(
    LegModel& leg,
    double alpha_target,
    double theta_init, double beta_init,
    int max_iters,
    double lr,
    double tol
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


std::vector<SwingPoint> HybridSwing::generate(LegModel& leg, SwingType type, double theta_start, double theta_end, double beta_start, double beta_end, int rim, double alpha, Eigen::Vector2d body_vel, Eigen::Vector2d ground_tangent, int points) {
    std::vector<SwingPoint> traj;
    for (int i = 0; i <= points; ++i) {
        double t = static_cast<double>(i) / points;
        double theta = 0.0, beta = 0.0;


        double mid_theta = 30.0 * M_PI / 180.0;
        if (t < 0.3) {
            theta = theta_start + (mid_theta - theta_start) * (2 * t);
        } else if(t>0.6) {
            theta = mid_theta + (theta_end - mid_theta) * (2 * (t - 0.6));
        }
        if (t > 0.3) {
            beta  = beta_start  + (beta_end - beta_start) * (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t);
        }
        else{
            beta  = beta_start;
        }
    
        // switch (type) {
        //     case SwingType::LINEAR:
        //         theta = theta_start + (theta_end - theta_start) * t;
        //         beta  = beta_start  + (beta_end  - beta_start)  * t;
        //         break;
        //     case SwingType::CUBIC: {
        //         double mid_theta = 17.0 * M_PI / 180.0;
        //         if (t < 0.5) {
        //             theta = theta_start + (mid_theta - theta_start) * (2 * t);
        //         } else {
        //             theta = mid_theta + (theta_end - mid_theta) * (2 * (t - 0.5));
        //         }
        //         beta  = beta_start + (beta_end - beta_start) * (3*t*t - 2*t*t*t);
        //         break;
        //     }
        //     case SwingType::FIVETIMES: {
        //         double mid_theta = 17.0 * M_PI / 180.0;
        //         if (t < 0.5) {
        //             theta = theta_start + (mid_theta - theta_start) * (2 * t);
        //         } else {
        //             theta = mid_theta + (theta_end - mid_theta) * (2 * (t - 0.5));
        //         }
        //         beta  = beta_start + (beta_end - beta_start) * (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t);
        //         break;
        //     }
        //     case SwingType::OPTIMIZE: {
        //         // 1) Precompute landing compensation epsilon
        //         Eigen::Vector2d P_end = pointOnRimByAlpha(leg, theta_end, beta_end, rim, alpha);
        //         double raw_prev = double(points - 1) / points;
        //         // θ 原始 RAW 兩段式（與 FIVETIMES 相同）
        //         auto compute_theta_raw = [&](double r){
        //             double mid = 17.0 * M_PI / 180.0;
        //             if (r < 0.5)
        //                 return theta_start + (mid - theta_start)*(2*r);
        //             else
        //                 return mid + (theta_end - mid)*(2*(r - 0.5));
        //         };
        //         // β 原始 RAW 五次多項
        //         auto compute_beta_raw = [&](double r){
        //             return beta_start + (beta_end - beta_start)*quintic(r);
        //         };
        //         Eigen::Vector2d P_prev = pointOnRimByAlpha(
        //             leg,
        //             compute_theta_raw(raw_prev),
        //             compute_beta_raw(raw_prev),
        //             rim, alpha
        //         );
        //         // 2) 計算 ε（預補償 warp 參數）
        //         Eigen::Vector2d dPdr = (P_end - P_prev) * points;
        //         double denom = dPdr.dot(ground_tangent);
        //         double vb_t  = body_vel.dot(ground_tangent);
        //         double eps   = (std::abs(denom) > 1e-6 ? 1.0 - vb_t/denom : 0.0);
        //         eps = clamp(eps, -0.5, 0.5);

        //         // 2) warp θ 用 lambda 曲線：r → r + ε·r(1−r)
        //         auto warp_theta = [&](double r){ return clamp(r + eps*r*(1-r), 0.0, 1.0); };

        //         // constraints
        //         const double max_dtheta = 1.0 * M_PI/180.0; // max 1° per step
        //         const double lambda     = 0.3;              // beta mix weight

        //         double prev_theta = theta_start;
        //         // 3) generate trajectory
        //         for (int i = 0; i <= points; ++i) {
        //             double raw = double(i) / points;
        //             // theta: warped two-segment + clamp delta
        //             double t_th    = warp_theta(raw);
        //             double theta_uns = compute_theta_raw(t_th);
        //             double dtheta_val = theta_uns - prev_theta;
        //             double theta_cur  = prev_theta + std::copysign(std::min(std::abs(dtheta_val), max_dtheta), dtheta_val);
            
        //             // beta: linear+quintic on raw (no warp)
        //             double s_lin   = raw;
        //             double s_quint = quintic(raw);
        //             double s_beta  = lambda*s_lin + (1.0 - lambda)*s_quint;
        //             double beta_cur = beta_start + (beta_end - beta_start) * s_beta;
        //             traj.push_back({ raw, theta_cur, beta_cur});
        //             prev_theta = theta_cur;
        //         }

        //         // 4) final refine by alpha optimization
        //         auto pair_ref = solveThetaBetaForAlpha(leg, alpha,
        //             traj.back().theta, traj.back().beta);
        //         traj.back().theta = pair_ref.first;
        //         traj.back().beta  = pair_ref.second;

            
        //         break;
               
        //     }
        // }

        traj.push_back({t, theta, beta});
    }
    return traj;
}
