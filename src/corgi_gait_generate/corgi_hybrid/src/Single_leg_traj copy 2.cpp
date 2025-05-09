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

// -------------------- Data Structure --------------------
struct DataPoint {
    double theta;
    double beta;
    double x;
    double y;
    std::string phase; // "swing"
};

// find_pose: find (theta,beta) so that G-contact starts at given shift+steplength
std::array<double,2> find_pose(LegModel& leg_model,
    double height,
    float shift,
    float steplength,
    double slope)
{
    std::array<double,2> pose;
    double pos[2] = {0, -height + leg_model.r};
    pose = leg_model.inverse(pos, "G");
    double ds = (steplength >= 0 ? +0.001 : -0.001);
    double target = shift + steplength;
    for (double s = 0; (ds>0 ? s < target : s > target); s += ds) {
        pose = leg_model.move(pose[0], pose[1], {ds,0}, slope);
    }
    return pose;
}

/// Compute (x,y) of the point on rim `rim` at parameter `alpha`
Eigen::Vector2d pointOnRimByAlpha(
    LegModel &legmodel,
    double theta,
    double beta,
    int rim,
    double alpha)
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
    if      (alpha >= -M_PI    && alpha < -deg50) rotated = std::polar(1.0, alpha + M_PI) * V0;
    else if (alpha >= -deg50   && alpha <  0.0  ) rotated = std::polar(1.0, alpha + deg50) * V0;
    else if (alpha >=  0.0     && alpha <  deg50) rotated = std::polar(1.0, alpha)        * V0;
    else if (alpha >=  deg50   && alpha <  M_PI ) rotated = std::polar(1.0, alpha - deg50) * V0;
    else {
        // tip (Eq.2.28)
        std::complex<double> VG = legmodel.G_c - legmodel.L_l_c;
        std::complex<double> tip = legmodel.L_l_c
            + (legmodel.r / legmodel.R) * std::polar(1.0, alpha) * VG;
        return { tip.real(), tip.imag() };
    }
    // rim point (Eq.2.27)
    std::complex<double> P = O + (legmodel.radius / legmodel.R) * rotated;
    return { P.real(), P.imag() };
}

// -------------------- Bézier: spatial interpolation --------------
Eigen::Vector2d bezierPoint(const std::vector<Eigen::Vector2d>& ctrl, double s) {
    static const int C[6] = {1,5,10,10,5,1};
    Eigen::Vector2d p(0,0);
    for (int i = 0; i < 6; ++i) {
        double b = C[i] * std::pow(s, i) * std::pow(1 - s, 5 - i);
        p += b * ctrl[i];
    }
    return p;
}

// -------------------- Cost: position error → θ,β ----------------
double costFunctionPos(
    LegModel& leg,
    double theta, double beta,
    const Eigen::Vector2d& target,
    double theta_prev, 
    double beta_prev,
    int rim,
    double alpha,
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


// -------------------- Solver: target pos → θ,β -----------------
std::pair<double,double> solveThetaBetaForPoint(
    LegModel& leg,
    const Eigen::Vector2d& target,
    double theta_init,
    double beta_init,
    int rim,
    double alpha,
    int max_iters = 50,
    double lr = 1e-2,
    double tol = 1e-6
) {
    double theta = theta_init, beta = beta_init;
    for (int i=0; i<max_iters; ++i) {
        double J = costFunctionPos(leg, theta, beta, target, theta_init, beta_init, rim, alpha);
        double eps = 1e-6;
        double dtheta = ( costFunctionPos(leg, theta+eps, beta, target, theta_init, beta_init, rim, alpha) - J ) / eps;
        double dbeta  = ( costFunctionPos(leg, theta, beta+eps, target, theta_init, beta_init, rim, alpha) - J ) / eps;
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
    for (auto& d : data) {
        file << d.theta << "," << d.beta << "," << d.x << "," << d.y << "," << d.phase << "\n";
    }
}

// -------------------- Save pure Bézier trajectory ----------------
void save_bezier_traj(const std::string& filename,
    const std::vector<Eigen::Vector2d>& ctrl_pts,
    int M) {
        std::ofstream file(filename);
        file << "x,y\n";
        file << std::fixed << std::setprecision(6);
        for (int i = 0; i <= M; ++i) {
            double s = double(i) / M;
            Eigen::Vector2d p = bezierPoint(ctrl_pts, s);
            file << p.x() << "," << p.y() << "\n";
        }
}

// -------------------- Trajectory Generator --------------------
// ctrl_pts: 6 world-space control points P0..P5
void generate_and_save_swing_trajectory(
    LegModel& leg,
    const std::vector<Eigen::Vector2d>& ctrl_pts,
    double theta0,
    double beta0,
    double theta_end,
    double beta_end,
    int N,
    const Eigen::Vector2d& body_vel,
    const Eigen::Vector2d& ground_tangent,
    const std::string& filename,
    int rim,
    double alpha
) {
    // compute required time-scaling epsilon
    Eigen::Vector2d dBds = 5.0 * (ctrl_pts[5] - ctrl_pts[4]);
    double vb_t = body_vel.dot(ground_tangent);
    double denom = dBds.dot(ground_tangent);
    double eps = 0.0;
    if (std::abs(denom) > 1e-6) eps = - vb_t / denom;

    // time-scaling function
    auto s_t = [&](double t){
        return 10*std::pow(t,3) - 15*std::pow(t,4) + 6*std::pow(t,5) + eps*t*(1-t);
    };

    std::vector<DataPoint> data;
    data.reserve(N+1);
    double theta_prev = theta0, beta_prev = beta0;

    // intermediate points
    for (int i = 0; i < N; ++i) {
        double t = static_cast<double>(i) / N;
        double s = s_t(t);
        Eigen::Vector2d target = bezierPoint(ctrl_pts, s);
        auto [theta_k, beta_k] = solveThetaBetaForPoint(
            leg, target, theta_prev, beta_prev,
            rim, alpha
        );
        Eigen::Vector2d p = pointOnRimByAlpha(leg, theta_k, beta_k, rim, alpha);
        data.push_back({theta_k, beta_k, p.x(), p.y(), "swing"});
        theta_prev = theta_k;
        beta_prev  = beta_k;
    }
    // enforce final pose
    leg.forward(theta_end, beta_end, false);
    Eigen::Vector2d p_final = pointOnRimByAlpha(leg, theta_end, beta_end, rim, alpha);
    data.push_back({theta_end, beta_end, p_final.x(), p_final.y(), "swing"});

    // 4) smooth theta/beta (moving average window=5, keep endpoints)
    int sz = data.size();
    std::vector<double> tbuf(sz), bbuf(sz);
    for(int i=0;i<sz;++i){ tbuf[i]=data[i].theta; bbuf[i]=data[i].beta; }
    int W=20;
    
    // --- 做两遍移动平均 ---
    std::vector<double> tbuf2 = tbuf, bbuf2 = bbuf;
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 1; i < sz-1; ++i) {
            int start = std::max(1, i-W/2), end = std::min(sz-2, i+W/2);
            double ts=0, bs=0; int cnt=0;
            for (int j = start; j <= end; ++j) {
                ts += tbuf2[j];
                bs += bbuf2[j];
                cnt++;
            }
            data[i].theta = ts/cnt;
            data[i].beta  = bs/cnt;
        }
        // 为下一遍，把刚平滑的结果作为输入
        for (int i = 1; i < sz-1; ++i) {
            tbuf2[i] = data[i].theta;
            bbuf2[i] = data[i].beta;
        }
    }
    // for(int i=1;i<sz-1;++i){
    //     int start = std::max(1, i-W/2), end = std::min(sz-2, i+W/2);
    //     double ts=0, bs=0; int cnt=0;
    //     for(int j=start;j<=end;++j){ ts+=tbuf[j]; bs+=bbuf[j]; cnt++; }
    //     data[i].theta = ts/cnt;
    //     data[i].beta  = bs/cnt;
    // }

    save_data_to_csv(filename, data);
}


int main() {
    const double stand_height = 0.149;
    const double step_length  = 0.4;
    const double swing_time   = 0.2;
    const double velocity     = 0.2;
    const int    pub_rate     = 1000;
    const double dS           = velocity / pub_rate;
    const double incre_duty   = dS / step_length;

    LegModel leg_model(true);
    std::vector<DataPoint> all_data;

    // compute start/end poses as before
    double duty = 1.0 - swing_time;
    auto start_pose = find_pose(leg_model, stand_height, 0.0,
                                (step_length/2) - (duty/(1-swing_time))*step_length, 0.0);
    double theta_start = start_pose[0];
    double beta_start  = -start_pose[1];
    duty = 0.0;
    auto end_pose = find_pose(leg_model, stand_height, 0.0,
                              (step_length/2), 0.0);
    double theta_end = end_pose[0];
    double beta_end  = -end_pose[1];
    while (beta_start < beta_end) beta_end -= 2.0 * M_PI;
    end_pose[1] = beta_end;

    // 1) contact map for end
    leg_model.contact_map(theta_end, beta_end);
    int    rim_id = leg_model.rim;
    double alpha0 = leg_model.alpha;
    Eigen::Vector2d pt_end(leg_model.contact_p[0], leg_model.contact_p[1]);

    // 2) calc start rim point 
    Eigen::Vector2d pt_start = pointOnRimByAlpha(leg_model, theta_start, beta_start, rim_id, alpha0);
    
    // 3) Swing trajectory
    Eigen::Vector2d p0 = pt_start;
    Eigen::Vector2d p5 = pt_end;

    Eigen::Vector2d hip = {0,0};
    Eigen::Vector2d dir = (p5 - p0).normalized();
    Eigen::Vector2d ortho(-dir.y(), dir.x());
    double SL = (p5 - p0).norm();
    double h1 = 0.3*SL, h2 = 0.6*SL;
    // Eigen::Vector2d p1 = p0 - 0.15*SL*dir + h1*ortho;
    // Eigen::Vector2d p2 = hip + h2*ortho;
    // Eigen::Vector2d p3 = p5 - 0.3*SL*dir + 0.1*h1*ortho;
    // Eigen::Vector2d p4 = p5 - 0.1*SL*dir;
    Eigen::Vector2d p1 = {p0.x()+0.5*leg_model.radius, stand_height};
    Eigen::Vector2d p2 = {p1.x()+1.5*leg_model.radius, p1.y()};
    Eigen::Vector2d p3 = {p2.x()+1.0*leg_model.radius, p2.y()/2};
    Eigen::Vector2d p4 = {p3.x(), -leg_model.radius};
    std::vector<Eigen::Vector2d> ctrl = {p0,p1,p2,p3,p4,p5};

    // cout all points
    std::cout << "p0: " << ctrl[0].transpose() << std::endl;
    std::cout << "p1: " << ctrl[1].transpose() << std::endl;
    std::cout << "p2: " << ctrl[2].transpose() << std::endl;
    std::cout << "p3: " << ctrl[3].transpose() << std::endl;
    std::cout << "p4: " << ctrl[4].transpose() << std::endl;
    std::cout << "p5: " << ctrl[5].transpose() << std::endl;
    std::cout << "theta_start: " << theta_start*180/M_PI << std::endl;
    std::cout << "beta_start: " << beta_start*180/M_PI << std::endl;
    std::cout << "theta_end: " << theta_end*180/M_PI << std::endl;
    std::cout << "beta_end: " << beta_end*180/M_PI << std::endl;

    std::cout << "theta_start: " << theta_start << std::endl;
    std::cout << "beta_start: " << beta_start<< std::endl;
    std::cout << "theta_end: " << theta_end << std::endl;
    std::cout << "beta_end: " << beta_end<< std::endl;

    
    // generate_and_save_swing_trajectory(leg_model, ctrl, theta_start, beta_start, 1000, "trajectory2.csv", rim_id, alpha0);
    // save ideal Bézier path
    save_bezier_traj("bezier_traj.csv", ctrl, 200);
    // body velocity and ground tangent
    Eigen::Vector2d body_vel(velocity, 0);
    Eigen::Vector2d ground_tan(1, 0);
    generate_and_save_swing_trajectory(leg_model, ctrl,  theta_start, beta_start, theta_end, beta_end, 1000, body_vel, ground_tan, "trajectory2.csv", rim_id, alpha0);
    std::cout << "Saved swing trajectory to trajectory.csv" << std::endl;
    
    return 0;
}