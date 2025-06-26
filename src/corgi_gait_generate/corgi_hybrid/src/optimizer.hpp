穩定度
// std::vector<int> makeSupportSet(Robot& robot)
// {
//     std::vector<int> S;
//     for (int i = 0; i < 4; ++i)
//         if (robot.legs[i].swing_phase == 0)      // 0 = stance
//             S.push_back(i);
//     return S;               // 可能回傳 3 或 4 腳
// }
// Eigen::Vector3d getCOMProjected(const Robot& robot)
// {
//     Eigen::Vector3d pc = { robot.body_position.x() + robot.CoM_pos.x(),
//                             robot.body_position.y() + robot.CoM_pos.y(),
//                             robot.body_position.z() };

//     if(robot.terrain){                             // 斜坡或不平整
//         pc.z() = robot.terrain->heightAt(pc.x()); // 垂直投影
//     }
//     return pc;
// }

// inline double poly3(double c3,double c2,double c1,double c0,double x){
//     return ((c3*x + c2)*x + c1)*x + c0;
// }
// struct InertialWrench{
//     Eigen::Vector3d f;   // 力  (N)
//     Eigen::Vector3d n;   // 力偶(N·m)
// };

InertialWrench calcLegInertia(const std::vector<SwingPoint>& traj,
                              double T_swing)       // 取第 idx 筆 (三點差分)
{
    const double dt = 0.001;
    // T_swing / (traj.size()-1);
    const double t = T_swing*traj.size();

    /* === 1. 取 theta_Iner,  θdot, θddot (二階差分) === */
    double theta_Iner   = traj[t].theta;
    double theta_Inerd  = (traj[t+1].theta - traj[t-1].theta) / (2*dt);
    double theta_Inerdd = (traj[t+1].theta - 2*traj[t].theta + traj[t-1].theta)/(dt*dt);

    /* === 2. 質心位移 p(theta_Iner) 與一次導數 p'(theta_Iner) (for 加速度) === */
    using namespace std;
    double px  = poly3(4.320353e-08,-4.195110e-04,-1.175735e-05,1.581175e-02, theta_Iner);
    double dpx = ( 3*4.320353e-08*theta_Iner*theta_Iner + 2*-4.195110e-04*theta_Iner + -1.175735e-05 );

    /* CG 在 body-x 方向，假設 y,z 方向變化小 */
    double ax_leg = dpx * theta_Inerdd + dpx * theta_Inerd*theta_Inerd;  /* p'(theta_Iner) θ¨ + p''(theta_Iner) θ̇²，但 p'' 很小可略 */

    Eigen::Vector3d a_cg( ax_leg, 0, 0 );     // 向前/後

    /* === 3. 慣性力 m a_cg === */
    Eigen::Vector3d f = 0.681 * a_cg;

    /* === 4. 轉動慣量多項式 I(theta_Iner) === */
    double Iyy = poly3(4.930266e-03,5.096802e-03,5.014408e-03,5.177460e-03, theta_Iner);

    /* === 5. 力偶 n = I  θ¨ ê_y （繞 body-y） === */
    Eigen::Vector3d n( 0, Iyy * theta_Inerdd , 0 );

    return { f, n };
}


void assembleResultantWrench(Robot& R,
                             Eigen::Vector3d& aCOM,
                             InertialWrench& out)
{
    /* --- 0. 重力 --- */
    out.f = { 0, 0, -R.mass * 9.81 };
    out.n = { 0, 0, 0 };

    /* --- 1. COM 慣性力 (−m a_COM) --- */
    out.f -= R.mass * aCOM;

    /* --- 2. 所有 swing 腿的慣性項 --- */
    for(int k=0;k<4;++k)
        if(R.legs[k].swing_phase==1){
            const auto& traj = R.legs[k].swing_traj;
            size_t idx = R.legs[k].swing_phase_idx;   // ←您需要在 FSM 裡維護
            InertialWrench w = calcLegInertia(traj,R.swing_time,idx);
            out.f += w.f;
            out.n += w.n;
        }
}

Eigen::Vector3d calcFr(const Robot& R,
                       const Eigen::Vector3d& acom_world,
                       int swingIdx,
                       const std::vector<Eigen::Vector3d>& a_leg_cg)
{
    Eigen::Vector3d fr = {0,0,-R.mass*9.81};           // 重力
    /* ① swing leg mass-transfer force (fee) */
    fr += 0.681 * a_leg_cg;                     // m_leg × a_leg_cg
    /* ② COM 慣性力  (-m_all a_com)  */
    fr -= R.mass * acom_world;                         // 式 4.3-5
    return fr;
}
/* ⬇︎ 依「支撐腳 index 集合」計算最小 θ_i (rad) ------------- */
double computeFASM_N(Robot& robot,InertialWrench& W)
{
    std::vector<int> support = makeSupportSet(robot);
    const int n = support.size();
    if (n < 3) return -1e9;                    // 少於三點 → 無定義

    Eigen::Vector3d pc = calcCOM(robot);         // COM 投影
    const Eigen::Vector3d fr = W.f;
    const Eigen::Vector3d nr = W.n;

    double psi_min = 1e9;

    for (int k = 0; k < n; ++k) {
        Eigen::Vector3d pi = robot.legs[support[k] ].foothold_current;
        Eigen::Vector3d pj = robot.legs[support[(k+1)%n]].foothold_current;

        /* --- 翻覆軸 a_i 與單位向量 â_i --- */
        Eigen::Vector3d ai = pj - pi;
        Eigen::Vector3d a = ai.normalized();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        /* ---- (4.2-10) ---- */
        Eigen::Vector3d fi = (I - a*a.transpose())*fr;

        /* ---- (4.2-11) ： n_i = (a aᵀ) n_r ---- */
        Eigen::Vector3d ni = a * (a.dot(nr));

        /* ---- (4.2-14)  f*_i = f_i + (a × n_i)/|a| ---- */
        Eigen::Vector3d fi_star = fi + a.cross(ni)/ai.norm();

        /* ---- (4.2-6)  l_i  ---- */
        Eigen::Vector3d li = (I - a*a.transpose())*(pj - pc);

        double sigma = (fi_star.cross(li).dot(a)>=0)?1:-1;
        double theta = sigma *
            std::acos( fi_star.normalized().dot( li.normalized() ) );

        psi_min = std::min(psi_min,std::abs(theta));
    }
    return psi_min;
}

psi_min >0 穩定
        ＝0 臨界
        <0 不穩定
psi_min >=5 degree (≈0.087 rad) （安全）