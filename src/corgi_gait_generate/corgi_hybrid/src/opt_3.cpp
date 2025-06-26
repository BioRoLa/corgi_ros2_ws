#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <vector>
#include <array>
#include <cmath>
#include "leg_model.hpp"

// ———— Terrain 类：支持多段“平面/斜坡”，且 length 为“水平长度” ————
class Terrain {
public:
    double terrain_start_height;
    // 构造：传入 x=0 处的初始高度
    Terrain(double init_height) {
        // segments.push_back({ 0.0, 0.0, 0.0, init_height });
        terrain_start_height = init_height;
    }

    // 增加一段水平 (plain)：在目前地形末端追加一段水平平台，水平长度 = length
    void addPlain(double length) {
        if (segments.empty()) {
            segments.push_back({ 0.0, length, 0.0, terrain_start_height });
        }
        else{
            Segment &prev = segments.back();
            double new_x0 = prev.x0 + prev.length;
            double new_h0 = prev.h0 + prev.slope * prev.length; 
            segments.push_back({ new_x0, length, 0.0, new_h0 });
        }
        // assert(!segments.empty()); 
    }

    // 增加一段斜坡 (slope)：slope = tan(theta)，length 表示此段的“水平长度”
    void addSlope(double slope, double length) {
        if (segments.empty()) {
            segments.push_back({ 0.0, length, slope, terrain_start_height });
        }
        else{
            Segment &prev = segments.back();
            double new_x0 = prev.x0 + prev.length;
            double new_h0 = prev.h0 + prev.slope * prev.length; 
            segments.push_back({ new_x0, length, slope, new_h0 });
        }
    }

    double height(double x) const {
        if (x < segments.front().x0) {
            return terrain_start_height;
        }
        for (const auto &seg : segments) {
            if (x >= seg.x0 && x <= (seg.x0 + seg.length)) {
                return seg.h0 + seg.slope * (x - seg.x0);
            }
        }
        const Segment &last = segments.back();
        return last.h0 + last.slope * last.length;
    }

    double check_position(double x) const{
        for (const auto &seg : segments) {
            if (x >= seg.x0 && x <= (seg.x0 + seg.length)) {
                if(seg.slope == 0.0){
                    return true; // 平面段
                }
                return false;
            }
        }
    }

    double compare_slope(double x,double y) const{
        double slope_x;
        double slope_y;
        for (const auto &seg : segments) {
            if (x >= seg.x0 && x <= (seg.x0 + seg.length)) {
               slope_x = seg.slope;
            }
            if (y >= seg.x0 && y <= (seg.x0 + seg.length)) {
               slope_y = seg.slope;
            }
        }
        if (slope_x == slope_y){
            return true; // 相同斜率
        }
        else{
            return false; // 不同斜率
        }
    }

   double slopeAt(double hip_x, bool next) const {
        // 1) 先找出 hip_x 落在哪个 segment 中（按 “x0 ≤ hip_x ≤ x0+length”）
        int idx = -1;
        int N   = segments.size();
        for (int i = 0; i < N; ++i) {
            const Segment &seg = segments[i];
            if (hip_x >= seg.x0 && hip_x <= seg.x0 + seg.length) {
                idx = i;
                break;
            }
        }
        // 2) 如果没找到（hip_x > 最后一个 segment 的 x0+length），那就把 idx 定为最后一个 segment
        if (idx == -1) {
            idx = N - 1;
        }

        if (!next) {
            // —— next == false：直接返回当前段的 slope
            return segments[idx].slope;
        } else {
            // —— next == true：想要返回“下一个 segment”的 slope
            int next_idx = idx + 1;
            if (next_idx >= N) {
                // 如果已经是最后一个 segment，后面没有下一个，就退回返回当前这段的 slope
                return segments[idx].slope;
            } else {
                // 否则返回下一个 segment 的 slope
                return segments[next_idx].slope;
            }
        }
    }

    // 打印所有 segment 信息，便于调试
    void printAll() const {
        std::cout << "Terrain segments (x0, length, slope, h0):\n";
        for (const auto &seg : segments) {
            std::cout << "  x0=" << seg.x0
                      << ", length=" << seg.length
                      << ", slope=" << seg.slope 
                      << ", h0=" << seg.h0 << "\n";
        }
    }

private:
    struct Segment {
        double x0;      // 段的起点 x 坐标（world-frame）
        double length;  // “水平”延申长度
        double slope;   // 斜率 = tan(θ)，垂直升降 / 水平距离
        double h0;      // 在 x0 处的高度
    };
    std::vector<Segment> segments;
};

// ———— 返回狀態碼 ————
enum TouchdownStatus {
    LEGAL = 0,
    THETA_VIOLATION,
    COLLISION,
    PENETRATION,
    FURTHER,
    UN_CONVERGED
};

// ———— 碰撞檢查 —— 取樣扇形 A–D、G-sector 與 E 線段 ————
bool checkCollision(LegModel &leg, 
                    double theta, double beta, 
                    const Terrain &terrain,
                    double hip_x, double stand_h) 
{
    // 1) 前向運算更新所有關節點
    leg.forward(theta, beta, true);

    const int N = 20;   // 每條弧線/線段取 N+1 個點檢查
    auto below = [&](double wx, double wy){
        // 僅檢查 hip_x ± stand_h 範圍內的地形
        if (wx < hip_x - stand_h || wx > hip_x + stand_h)
            return false;
        return wy < terrain.height(wx);
    };

    // 2) 扇形取樣檢查函式
    auto sampleSector = [&](std::complex<double> C,
                            std::complex<double> P0,
                            std::complex<double> P1,
                            double R) {
        double a0 = std::atan2((P0 - C).imag(), (P0 - C).real());
        double a1 = std::atan2((P1 - C).imag(), (P1 - C).real());
        if (a1 < a0) a1 += 2*M_PI;
        for (int i = 0; i <= N; ++i) {
            double a = a0 + (a1-a0)*i/N;
            double xr = C.real() + R * std::cos(a);
            double yr = C.imag() + R * std::sin(a);
            double wx = hip_x + xr;
            double H = terrain.height(wx);
            if (below(wx, yr)) return true;
        }
        return false;
    };

    // A: L_l_c → [F_l_c, G_c]
    {
        auto C = leg.L_l_c;
        double R = std::abs(leg.F_l_c - C) + leg.r ;
        if (sampleSector(C, leg.F_l_c, leg.G_c, R)) return true;
    }
    // B: L_r_c → [G_c, F_r_c]
    {
        auto C = leg.L_r_c;
        double R = std::abs(leg.F_r_c - C) + leg.r ;
        if (sampleSector(C, leg.G_c, leg.F_r_c, R)) return true;
    }
    // C: U_r_c → [H_r_c, F_r_c]
    {
        auto C = leg.U_r_c;
        double R = std::abs(leg.H_r_c - C) + leg.r ;
        if (sampleSector(C, leg.H_r_c, leg.F_r_c, R)) return true;
    }
    // D: U_l_c → [H_l_c, F_l_c]
    {
        auto C = leg.U_l_c;
        double R = std::abs(leg.H_l_c - C) + leg.r ;
        if (sampleSector(C, leg.H_l_c, leg.F_l_c, R)) return true;
    }

    // G-sector: G_c → [extend toward L_l_c, extend toward L_r_c]
    {
        auto C = leg.G_c;
        // 往 L_*_c 方向的反向延伸 leg.r
        auto dirL = (leg.L_l_c - C) / std::abs(leg.L_l_c - C);
        auto dirR = (leg.L_r_c - C) / std::abs(leg.L_r_c - C);
        std::complex<double> P0 = C - dirL * leg.r;
        std::complex<double> P1 = C - dirR * leg.r;
        if (sampleSector(C, P0, P1, leg.r)) return true;
    }

    // E 段：U_l_c↔H_l_c、U_r_c↔H_r_c 延伸 leg.r 後取樣直線
    {
        // 延伸端點
        auto dirL = (leg.H_l_c - leg.U_l_c) / std::abs(leg.H_l_c - leg.U_l_c);
        auto dirR = (leg.H_r_c - leg.U_r_c) / std::abs(leg.H_r_c - leg.U_r_c);
        auto extL = leg.U_l_c + dirL * leg.r;
        auto extR = leg.U_r_c + dirR * leg.r;
        // 直線取樣
        for (int i = 0; i <= N; ++i) {
            double t = double(i)/N;
            double xr = (1-t)*extL.real() + t*extR.real();
            double yr = (1-t)*extL.imag() + t*extR.imag();
            double wx = hip_x + xr;
            if (below(wx, yr)) return true;
        }
    }
    
    return false;
}

// ———— 全域參數 ————
static const double clearence      = 0.001;    // min安全距離 (m)
static const double clearence_M    = 0.005;    // MAX安全距離 (m)
static const double step_len       = 0.15;      // 步長 (m)
static const double swing_t        = 0.2;      // 擺動階段比例
static const double vel            = 0.2;      // 速度 (m/s)
static const double dS             = vel / 1000.0;  // 每步腳移動量 (m)

double clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}

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

double quintic(double t) { 
    return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t; 
}

// -------------------- Data Structure --------------------
struct DataPoint {
    double theta, beta, x, y;
    std::string phase;  // "swing"
};

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

void generate_swing_trajectory(
    LegModel& leg, double stand_h,
    double theta_start, double beta_start,
    double theta_end,   double beta_end,
    int N,
    int rim, double alpha,
    const Eigen::Vector2d &body_vel,
    const Eigen::Vector2d &ground_tangent,
    std::vector<DataPoint> &data)
{
    // 中間節點（固定用 17° → 17°*π/180）
    const double mid_theta   = 17.0 * M_PI / 180.0;
    // 每次迭代 θ 最大只能改變 0.5° → 0.5°*π/180
    const double max_dtheta  = 10  * M_PI / 180.0;

    // 1) θ 插值函式： r ∈ [0,1]，如果 r<0.5 用前段從 theta_start → mid_theta，
    //    否則用後段從 mid_theta → theta_end，兩段都用 quintic(u) 做 ease‐in‐out
    auto theta_raw = [&](double r) {
        if (r < 0.5) {
            double u = r / 0.5;  // 前半段正規化到 [0,1]
            return theta_start + (mid_theta - theta_start) * quintic(u);
        } else {
            double u = (r - 0.5) / 0.5;  // 後半段正規化到 [0,1]
            return mid_theta    + (theta_end   - mid_theta)    * quintic(u);
        }
    };

    // 2) β 插值函式： r ∈ [0,1]，直接用 quintic(r) 從 beta_start → beta_end
    auto beta_raw = [&](double r) {
        return beta_start + (beta_end - beta_start) * quintic(r);
    };

    // 清空並預留空間
    data.clear();
    data.reserve(N + 1);

    // prev_th 紀錄上一次迭代真正使用的 θ
    double prev_th = theta_start;

    // 主迴圈： i=0..N 共 N+1 個點
    for (int i = 0; i <= N; ++i) {
        double r = double(i) / N;            // 從 0 到 1 等份
        // 從插值函式取理想上的目標 θ
        double target_th = theta_raw(r);
        // 計算與前一次 θ 差距
        double diff = target_th - prev_th;
        // 如果差距太大，就以 max_dtheta 做限制
        double limited_dth = std::copysign(
            std::min(std::abs(diff), max_dtheta),
            diff
        );
        // 真正這一筆用的 θ
        double theta_cur = prev_th + limited_dth;

        // β 沒做限速，直接用五次插值
        double beta_cur = beta_raw(r);

        // 計算當前 (theta_cur, beta_cur) 對應到輪緣上 alpha 的 (x,y)
        Eigen::Vector2d p = pointOnRimByAlpha(
            leg, theta_cur, beta_cur, rim, alpha
        );

        // 存入 data (phase 一律設定為 "swing")
        data.push_back({theta_cur, beta_cur, p.x(), p.y(), "swing"});

    }
}

// ———— 模擬一次 touchdown 回傳狀態碼 ————
TouchdownStatus simulate_td_status(LegModel &leg, Terrain &terrain,double stand_h, double shift, double O_start_x) 
{   
    double hip_x = O_start_x;
    auto eta = leg.inverse({0.0, -stand_h + leg.r}, "G");
    
    // 1) 後退 shift
    for (double s1 = 0; s1 < shift; s1 += 0.001) {
        eta = leg.move(eta[0], eta[1], { -0.001, 0.0 }, 0.0);
        if (eta[0] < 17*M_PI/180.0 || eta[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
    }
    // 2) 前進半步長
    for (double s = 0; s < 0.5 * step_len; s += 0.001) {
        try {
             eta = leg.move(eta[0], eta[1], {-0.001, 0.0}, 0.0);
        } catch (std::runtime_error &e) {
            return UN_CONVERGED;
        }
       
        if (eta[0] < 17*M_PI/180.0 || eta[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
    }
    
    // 3) 初始碰撞檢查
    leg.forward(eta[0], eta[1], true);
    if (checkCollision(leg,eta[0],eta[1],terrain,hip_x,stand_h)){
        return COLLISION;
    }
    
    // 4) 初始接觸深度檢查
    // use leg.contact_map(eta[0], eta[1], 0.0); and leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
    // 先測試 hip 所在的地形
    leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, false)));
    if ((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, false))) < clearence){
        return PENETRATION;
    }
    else if((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, false))) > clearence_M ){
        // 測試下一個地形
        leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
        if(!terrain.check_position(hip_x + leg.contact_p[0])){
            if ((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, true)))< clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, true))) > clearence_M ){
                return FURTHER;
            }
        }
        else{
            return FURTHER;
        }
    }

    // else{
    //     // std::cout << "Initial pose Touching differ terrain at x = " << hip_x << std::endl;
    // }
    
    // 5) touchdown phase
    for (double t = 0.0; t <= (1.0 - swing_t); t += dS / step_len) {
        hip_x += dS;
        double check = false;
        try{
            // if touching the same slope
            double contact_a;
            double contact_b;
            leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, false)));
            contact_a = leg.contact_p[0];
            leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
            contact_b = leg.contact_p[0];
            check = terrain.compare_slope(contact_a, contact_b);
            if( check ){
                // 當前所在斜率
                eta = leg.move(eta[0], eta[1], {dS, 0.0}, atan(terrain.slopeAt(hip_x, false)));
            }
            else{
                // 下一個地形斜率
                eta = leg.move(eta[0], eta[1], {dS, 0.0}, atan(terrain.slopeAt(hip_x, true)));
            }
        }
        catch(const std::exception& e){
            return UN_CONVERGED;
        }
       
        if (eta[0] < 17*M_PI/180.0 || eta[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
        
        leg.forward(eta[0], eta[1], true);
        if (checkCollision(leg,eta[0],eta[1],terrain,hip_x,stand_h)){
            return COLLISION;
        }
        
        if(check){
            leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, false)));
            if ((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, false)))< clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0])) > clearence_M ){
                return FURTHER;
            }
        }
        else{
            leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
            if ((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, true)))< clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - terrain.height(hip_x + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x, true))) > clearence_M ){
                return FURTHER;
            }
                
        }
    } 
    return LEGAL;
}

// ———— 模擬一次 swing + touchdown 回傳狀態碼 ————
TouchdownStatus simulate_status(LegModel &leg, Terrain &terrain,double stand_h, double shift, double O_start_x, std::array<double,2> eta_current, std::vector<DataPoint> &data_point) 
{   
    // 1) swing phase
    // find the end pose
    double hip_x_temp = O_start_x + (swing_t/(1-swing_t)) * step_len;
    double terrain_height_temp = terrain.height(hip_x_temp);
    auto eta_temp = leg.inverse({0.0, -stand_h + leg.r}, "G");
    // 後退 shift
    for (double s1 = 0; s1 < shift; s1 += 0.001) {
        eta_temp = leg.move(eta_temp[0], eta_temp[1], { -0.001, 0.0 }, 0.0);
        if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
    }
    // 前進半步長
    for (double s = 0; s < 0.5 * step_len; s += 0.001) {
        try {
             eta_temp = leg.move(eta_temp[0], eta_temp[1], {-0.001, 0.0}, 0.0);
        } catch (std::runtime_error &e) {
            return UN_CONVERGED;
        }
       
        if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
    }
    
    // 初始碰撞檢查
    leg.forward(eta_temp[0], eta_temp[1], true);
    if (checkCollision(leg,eta_temp[0],eta_temp[1],terrain,hip_x_temp,stand_h)){
        return COLLISION;
    }
    
    // 初始接觸深度檢查
    // use leg.contact_map(eta[0], eta[1], 0.0); and leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
    // 先測試 hip 所在的地形
    leg.contact_map(eta_temp[0], eta_temp[1], atan(terrain.slopeAt(hip_x_temp, false)));
    if ((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, false))) < clearence){
        return PENETRATION;
    }
    else if((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, false))) > clearence_M ){
        // 測試下一個地形
        leg.contact_map(eta_temp[0], eta_temp[1], atan(terrain.slopeAt(hip_x_temp, true)));
        if(!terrain.check_position(hip_x_temp + leg.contact_p[0])){
            if ((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, true)))< clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, true))) > clearence_M ){
                return FURTHER;
            }
        }
        else{
            return FURTHER;
        }
    }
    auto eta_temp_swing = eta_temp;
    double check;
    // touchdown phase
    for (double t = 0.0; t <= (1.0 - swing_t); t += dS / step_len) {
        hip_x_temp += dS;
        check = false;
        try{
            // if touching the same slope
            double contact_a;
            double contact_b;
            leg.contact_map(eta_temp[0], eta_temp[1], atan(terrain.slopeAt(hip_x_temp, false)));
            contact_a = leg.contact_p[0];
            leg.contact_map(eta_temp[0], eta_temp[1], atan(terrain.slopeAt(hip_x_temp, true)));
            contact_b = leg.contact_p[0];
            check = terrain.compare_slope(contact_a, contact_b);
            if( check ){
                // 當前所在斜率
                eta_temp = leg.move(eta_temp[0], eta_temp[1], {dS, 0.0}, atan(terrain.slopeAt(hip_x_temp, false)));
            }
            else{
                // 下一個地形斜率
                eta_temp = leg.move(eta_temp[0], eta_temp[1], {dS, 0.0}, atan(terrain.slopeAt(hip_x_temp, true)));
            }
        }
        catch(const std::exception& e){
            return UN_CONVERGED;
        }
       
        if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
        
        leg.forward(eta_temp[0], eta_temp[1], true);
        if (checkCollision(leg,eta_temp[0],eta_temp[1],terrain,hip_x_temp,stand_h)){
            return COLLISION;
        }
        
        if(check){
            leg.contact_map(eta_temp[0], eta_temp[1], atan(terrain.slopeAt(hip_x_temp, false)));
            if ((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, false)))< clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0])) > clearence_M ){
                return FURTHER;
            }
        }
        else{
            leg.contact_map(eta_temp[0], eta_temp[1], atan(terrain.slopeAt(hip_x_temp, true)));
            if ((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, true)))< clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - terrain.height(hip_x_temp + leg.contact_p[0]) )*cos(atan(terrain.slopeAt(hip_x_temp, true))) > clearence_M ){
                return FURTHER;
            }
                
        }
    } 

    leg.forward(eta_temp_swing[0], eta_temp_swing[1], false);
    if (check){
        leg.contact_map(eta_temp_swing[0], eta_temp_swing[1], atan(terrain.slopeAt(O_start_x, false)));
    }
    else{
        leg.contact_map(eta_temp_swing[0], eta_temp_swing[1], atan(terrain.slopeAt(O_start_x, true)));
    }
    
    int rim_id = leg.rim;
    double alpha0 = leg.alpha;
    // swing traj
    // TD: add collision check in swing phase
    std::cout << "Generating swing trajectory..." << std::endl;
    eta_temp[1] -= 2.0*M_PI;
    eta_temp_swing[1] -= 2.0*M_PI;
    std::cout << "eta_temp: " << eta_temp_swing[0] << ", " << eta_temp_swing[1] << std::endl;
    // cout stand_h, shift
    // std::cout << "stand_h: " << stand_h << ", shift: " << shift << std::endl;
    generate_swing_trajectory(
        leg, stand_h,
        eta_current[0], eta_current[1], eta_temp_swing[0], eta_temp_swing[1],
        200, rim_id, alpha0,
        Eigen::Vector2d(vel,0), Eigen::Vector2d(1,0), data_point
    );

    return LEGAL;
}

// ———— 輸出 touchdown 軌跡到 CSV ————
std::array<double,2> write_touchdown_csv(LegModel &leg, Terrain &terrain, double stand_h, double shift, std::ofstream &file2, double O_x,int index) { 
    // how to store the above info?
    std::cout << "Writing touchdown trajectory to CSV..." << std::endl;
    std::cout << "stand_h: " << stand_h << ", shift: " << shift << std::endl;
    double hip_x_final = O_x; // O 點 x 座標
    double target_pose[2] = {0.0, -stand_h + leg.r};
    auto eta = leg.inverse(target_pose, "G");
    for (double s = 0; s < shift; s += 0.001){
        eta = leg.move(eta[0], eta[1], { -0.001,0.0 }, 0.0);
    }
    
    for (double s = 0; s < 0.5 * step_len; s += 0.001){
        eta = leg.move(eta[0], eta[1], {-0.001, 0.0}, 0.0);
    }
    std::cout << "eta: " << eta[0] << ", " << eta[1]-index*2*M_PI  << std::endl;
    bool check_csv = false;
    // if touching the same slope
    double contactA;
    double contactB;
    leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, false)));
    contactA = leg.contact_p[0];
    leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, true)));
    contactB = leg.contact_p[0];
    check_csv = terrain.compare_slope(contactA, contactB);


    // initial contact
    if (check_csv){
        leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, false)));
    } else {
        leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, true)));
        
    }
    std::cout << "eta12012012: " << eta[0] << ", " << eta[1] << std::endl;
    eta[1] = eta[1]-index*2*M_PI ;
    file2 << std::fixed << std::setprecision(6)
        << eta[0] << "," << eta[1] << ","
        << leg.contact_p[0] << "," << leg.contact_p[1]
        << ",initial\n";
    
    for (double t = 0.0; t <= (1.0 - swing_t); t += dS / step_len) {
        hip_x_final += dS;

        check_csv = false;
        // if touching the same slope
        leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, false)));
        contactA = leg.contact_p[0];
        leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, true)));
        contactB = leg.contact_p[0];
        check_csv = terrain.compare_slope(contactA, contactB);

    
        if (check_csv){
            eta = leg.move(eta[0], eta[1], {dS, 0.0}, atan(terrain.slopeAt(hip_x_final, false)));
            leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x_final, false)));
        } else {
            eta = leg.move(eta[0], eta[1], {dS, 0.0},  atan(terrain.slopeAt(hip_x_final, true)));
            leg.contact_map(eta[0], eta[1],  atan(terrain.slopeAt(hip_x_final, true)));
        }
        
        file2 << std::fixed << std::setprecision(6)
              << eta[0] << "," << eta[1] << ","
              << leg.contact_p[0] << "," << leg.contact_p[1]
              << ",touchdown\n";
    }
    return eta; // 返回最後的 theta 和 beta
}




// ———— 主程式：手動巢狀搜尋 ————
int main() {
    std::ofstream file2("opt_1.csv");
    file2 << "theta,beta,x,y,phase\n";
    std::vector<DataPoint> data;
    Terrain terrain(-0.149);
    terrain.addPlain(0.08);
    double angle1 = 15.0 * M_PI/180.0;
    terrain.addSlope(tan(angle1), 0.10);
    terrain.addPlain(5);
    // terrain.printAll();
    double O_start_x= 0.0;
    // double O_start_x= step_len+(swing_t/(1-swing_t))*step_len; // start at the beginning of the slope
    std::cout << "O_start_x: " << O_start_x << std::endl;
    double stand_h  = 0.34;

    double best_h= stand_h, best_shift=-step_len;
    bool found = false;
    

    for (double h = best_h; h >= 0.120; h -= 0.0005) {
        // std::cout << "Testing height: " << h << std::endl;
        bool theta_bound_reached = false;
        for (double sh = -step_len; sh <= step_len; sh += 0.01) {
            LegModel leg(true);
            TouchdownStatus status = simulate_td_status(leg, terrain, h, sh, O_start_x);
            if (status == LEGAL) {
                std::cout << "Found LEGAL"<< std::endl;
                best_h = h;
                found = true;
                best_shift = sh;
                break;
            }
            else if (status == THETA_VIOLATION) {
                theta_bound_reached = true;
                // std::cout << status << std::endl;
                break; 
            }
            else if (status == COLLISION) {
                // std::cout << status << std::endl;
                continue; 
            }
            else if (status == PENETRATION) {
                // std::cout << status << std::endl;
                continue; 
            }
            else if (status == UN_CONVERGED) {
                // std::cout << status << std::endl;
                break; 
            }
            else if (status == FURTHER){
                // std::cout << status << std::endl;
                continue; 
            }
        }
        // std::cout<<"Finish searching this Height: " << h << std::endl;
        if (found) {break;}
        if (theta_bound_reached) {continue;}
        
    }
    if (found) {
        std::cout << "Found pose: stand_h = " << best_h
                  << ", shift = " << best_shift << std::endl;
        found = false;
        LegModel leg_final(true);
        // return the final pose
        std::array<double,2> eta_final = write_touchdown_csv(leg_final, terrain, best_h, best_shift, file2, O_start_x,0);
        for (double h = stand_h; h >= 0.120; h -= 0.0005) {
            // std::cout << "Testing height: " << h << std::endl;
            bool theta_bound_reached = false;
            for (double sh = -step_len; sh <= step_len; sh += 0.01) {
                LegModel leg(true);
                TouchdownStatus status = simulate_status(leg, terrain, h, sh, O_start_x+step_len+(swing_t/(1-swing_t))*step_len, eta_final, data);
                if (status == LEGAL) {
                    std::cout << "Found LEGAL"<< std::endl;
                    best_h = h;
                    found = true;
                    best_shift = sh;
                    break;
                }
                else if (status == THETA_VIOLATION) {
                    theta_bound_reached = true;
                    // std::cout << status << std::endl;
                    break; 
                }
                else if (status == COLLISION) {
                    // std::cout << status << std::endl;
                    continue; 
                }
                else if (status == PENETRATION) {
                    // std::cout << status << std::endl;
                    continue; 
                }
                else if (status == UN_CONVERGED) {
                    // std::cout << status << std::endl;
                    break; 
                }
                else if (status == FURTHER){
                    // std::cout << status << std::endl;
                    continue; 
                }
            }
            // std::cout<<"Finish searching this Height: " << h << std::endl;
            if (found) {break;}
            if (theta_bound_reached) {continue;}
            
        }
    } else {
        std::cout << "No legal pose found within bounds." << std::endl;
    }

    if (found) {
        std::cout << "Found2 pose: stand_h = " << best_h
                  << ", shift = " << best_shift << std::endl;
        LegModel leg_final(true);
        // pushback all the lines in data to file2
        // for every line in data
        for (const auto& row : data) {
            file2 << std::fixed << std::setprecision(6)
                << row.theta << ','
                << row.beta  << ','
                << row.x     << ','
                << row.y     << ','
                << row.phase << '\n';
        }
        std::array<double,2> eta_final_2 = write_touchdown_csv(leg_final, terrain, best_h, best_shift, file2,  O_start_x+step_len+(swing_t/(1-swing_t))*step_len, 1);
        file2.close();
    } else {
        std::cout << "end without a result" << std::endl;
    }


    return 0;
}

