#include "wlw_trans.hpp"

double deg2rad(double degrees) {
    return degrees * PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / PI;
}

double find_closest_beta(double target_beta, double ref_beta)
{
    auto r6 = [](double v){ return std::round(v * 1e6) / 1e6; };

    if (r6(target_beta) < r6(ref_beta)) {
        while (std::fabs(r6(target_beta) - r6(ref_beta)) >= M_PI) {
            target_beta += 2.0 * M_PI;
        }
    } 
    else if (r6(target_beta) > r6(ref_beta)) {
        while (std::fabs(r6(target_beta) - r6(ref_beta)) >= M_PI) {
            target_beta -= 2.0 * M_PI;
        }
    }
    return target_beta;
}

double find_smaller_closest_beta(double target_beta, double ref_beta)
{
    double tmp = find_closest_beta(target_beta, ref_beta);
    if (tmp > ref_beta) {
        tmp -= 2.0 * M_PI;
    }
    return tmp;
}

std::pair<int, double> find_hybrid_step(double RH_beta,
                                        double LH_beta,
                                        double body_angle,
                                        double radius)
{
    int step_num = 1;
    const int max_step_num = 6;

    double min_step_length = 0.1;
    double max_step_length = 0.2;

    // from Python: np.deg2rad(90) = M_PI/2
    double RH_target_beta = find_smaller_closest_beta((M_PI/2.0) - body_angle, RH_beta);
    double LH_target_beta = find_smaller_closest_beta((M_PI/2.0) - body_angle, LH_beta);

    for (int i = 0; i < max_step_num; i++)
    {
        double step_length = 0.0;
        if (step_num % 2 == 1) {
            // odd => compute step_length from RH
            step_length = std::fabs(radius * (RH_target_beta - RH_beta)) / step_num;

            if (step_length < min_step_length) {
                RH_target_beta -= 2.0 * M_PI;
            }
            else if (step_length > min_step_length && step_length < max_step_length) {
                return std::make_pair(step_num, step_length);
            }
        }
        else {
            // even => compute step_length from LH
            step_length = std::fabs(radius * (LH_target_beta - LH_beta)) / step_num;

            if (step_length < min_step_length) {
                LH_target_beta -= 2.0 * M_PI;
            }
            else if (step_length > min_step_length && step_length < max_step_length) {
                return std::make_pair(step_num, step_length);
            }
        }
        step_num++;
    }

    // If no valid step found
    return std::make_pair(0, 0.0);
}

void wlw_transform_main()
{
    // Setup
    std::vector<double> ideal_theta = {
        1.34154, // LF
        1.22377, // RF
        1.22503, // RH
        1.3401  // LH
    };
    std::vector<double> ideal_beta = {
        0.690684,   // LF
        -5.67911,  // RF
        -0.605124,   // RH
        5.59344   // LH
    };
    std::cout << "= = = Wheel to WLW Start = = =" << std::endl;
    // Wheeled mode
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_int_distribution<int> dist360(0, 359);
    // init theta
    std::vector<double> init_theta = {
        17.0*PI/180.0,
        17.0*PI/180.0,
        17.0*PI/180.0,
        17.0*PI/180.0
    };
    // init beta (4 random angles in [0..360))
    std::vector<double> init_beta(4);
    for (int i=0; i<4; i++) {
        init_beta[i] = dist360(gen) * PI/180.0;
    }
    // Build transform to initial pose in 5sec => 5000 steps
    int transform_steps = 5000;
    std::vector<std::vector<double>> traj_theta_transform(transform_steps, std::vector<double>(4));
    std::vector<std::vector<double>> traj_beta_transform(transform_steps, std::vector<double>(4));
    for(int i=0; i<transform_steps; i++){
        double t = double(i)/(transform_steps-1);
        for(int j=0; j<4; j++){
            double startTheta = 17.0*PI/180.0;
            double endTheta   = init_theta[j];
            traj_theta_transform[i][j] = startTheta + (endTheta - startTheta)*t;

            double startBeta = 0.0;
            double endBeta   = init_beta[j];
            traj_beta_transform[i][j] = startBeta + (endBeta - startBeta)*t;
        }
    }
    
    std::cout << "Initial beta = [";
    for (int i = 0; i < 4; i++) {
        std::cout << (init_beta[i] * 180.0 / PI);
        if (i < 3) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // stay for 2 sec => 2000 steps
    int stay_steps = 2000;
    std::vector<std::vector<double>> traj_theta_stay(stay_steps, std::vector<double>(4));
    std::vector<std::vector<double>> traj_beta_stay(stay_steps, std::vector<double>(4));
    for(int i=0; i<stay_steps; i++){
        for(int j=0; j<4; j++){
            traj_theta_stay[i][j] = init_theta[j];
            traj_beta_stay[i][j]  = init_beta[j];
        }
    }

    LegModel leg(true);
    double wheel_delta_beta = -body_vel / leg.radius * dt;
    std::vector<double> curr_theta = init_theta; 
    std::vector<double> curr_beta  = init_beta;
    std::vector<std::vector<double>> traj_theta;
    std::vector<std::vector<double>> traj_beta;
    traj_theta.push_back(curr_theta);
    traj_beta.push_back(curr_beta);

    // Rotate in Wheel Mode until RF_beta = 45 deg
    double RF_target_beta = deg2rad(45.0);
    RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1]);

    double body_move_dist = std::fabs(leg.radius * (RF_target_beta - curr_beta[1]));
    int delta_time_step = (int)std::round( (body_move_dist/body_vel)/dt );

    // Append trajectories each step
    for(int t=0; t<delta_time_step; t++){
        // increment all betas by wheel_delta_beta
        for(int j=0; j<4; j++){
            curr_beta[j] += wheel_delta_beta;
        }
        // Thetas remain the same
        traj_theta.push_back(curr_theta);
        traj_beta.push_back(curr_beta);
    }

    std::cout << "Front Start Beta [deg] = [";
    for(int i=0; i<4; i++){
        std::cout << (curr_beta[i] * 180.0/M_PI)
                  << ((i<3)? ", ":"");
    }
    std::cout << "]\n= = =\n";

    // Front Transform
    // double body_angle = std::asin( (stance_height - leg.radius)/body_length );
    // double RF_target_beta_new = -body_angle;
    double RF_target_beta_new = ideal_beta[1];
    RF_target_beta_new = find_smaller_closest_beta(RF_target_beta_new, curr_beta[1]);
    double RF_target_theta = ideal_theta[1];
    // double RF_target_theta = leg.inverse({0, -stance_height+leg.r},"G")[1];

    std::cout << "Body Angle = " << RF_target_beta_new << std::endl;

    std::cout << "RF Target Theta = " << (RF_target_theta * 180.0 / M_PI) << std::endl;

    std::cout << "RF Target Beta = " << (RF_target_beta * 180.0 / M_PI) << std::endl;

    body_move_dist = std::fabs(leg.radius * (RF_target_beta_new - curr_beta[1]));
    delta_time_step = (int)std::round( (body_move_dist/body_vel)/dt );

    double RF_delta_beta  = (RF_target_beta_new - curr_beta[1]) / delta_time_step;
    double RF_delta_theta = (RF_target_theta    - curr_theta[1]) / delta_time_step;

    // Hybrid 
    auto [step_num, step_length] = find_hybrid_step(
        curr_beta[2] + wheel_delta_beta*delta_time_step,
        curr_beta[3] + wheel_delta_beta*delta_time_step,
        -ideal_beta[1],
        leg.radius
    );

    
    // For LF, do leg.inverse({step_length, -stance_height+leg.r})
    double LF_target_theta = ideal_theta[0];
    double LF_target_beta  = ideal_beta[0];
    LF_target_beta = find_smaller_closest_beta(LF_target_beta, curr_beta[0]);

    double LF_delta_beta  = (LF_target_beta  - curr_beta[0]) / delta_time_step;
    double LF_delta_theta = (LF_target_theta - curr_theta[0]) / delta_time_step;

    for(int t=0; t<delta_time_step; t++){
        if(t < delta_time_step/3) {
            curr_beta[0] += wheel_delta_beta;
        }
        else if(t < (2*delta_time_step)/3) {
            curr_beta[0] += LF_delta_beta*3 - wheel_delta_beta;
        }
        else {
            curr_theta[0] += LF_delta_theta*3;
        }

        curr_theta[1] += RF_delta_theta;
        curr_beta[1]  += RF_delta_beta;

        // rear => wheel mode
        curr_beta[2] += wheel_delta_beta;
        curr_beta[3] += wheel_delta_beta;
        leg.forward(curr_theta[0],curr_beta[0]);
        leg.contact_map(curr_theta[0],curr_beta[0],0);
        if(-leg.contact_p[1]>stance_height){
            //todo give beta and height find theta
        }

        traj_theta.push_back(curr_theta);
        traj_beta.push_back(curr_beta);
    }

    std::cout << "Front End Theta [deg] = [";
    for(int i=0; i<4; i++){
        std::cout << rad2deg(curr_theta[i]) << ((i<3)? ", ":"");
    }
    std::cout << "]\n";

    std::cout << "Front End Beta [deg]  = [";
    for(int i=0; i<4; i++){
        std::cout << rad2deg(curr_beta[i]) << ((i<3)? ", ":"");
    }
    std::cout << "]\n";



    // Combine transform + stay
    // store them in "all_theta" and "all_beta".
    std::vector<std::vector<double>> all_theta;
    std::vector<std::vector<double>> all_beta;

    // push transform portion
    all_theta.insert(all_theta.end(),
                     traj_theta_transform.begin(),
                     traj_theta_transform.end());
    all_beta.insert(all_beta.end(),
                    traj_beta_transform.begin(),
                    traj_beta_transform.end());

    // push stay portion
    all_theta.insert(all_theta.end(),
                     traj_theta_stay.begin(),
                     traj_theta_stay.end());
    all_beta.insert(all_beta.end(),
                    traj_beta_stay.begin(),
                    traj_beta_stay.end());

    // push stay portion
    all_theta.insert(all_theta.end(),
                     traj_theta.begin(),
                     traj_theta.end());
    all_beta.insert(all_beta.end(),
                    traj_beta.begin(),
                    traj_beta.end());

    // Build the final 8-column CSV data:
    // [theta_LF, -beta_LF, theta_RF, beta_RF, theta_RH, beta_RH, theta_LH, -beta_LH]
    std::vector<std::vector<double>> traj_final(all_theta.size(), std::vector<double>(8, 0.0));

    for (size_t i=0; i<all_theta.size(); i++) {
        // LF => index 0
        traj_final[i][0] = all_theta[i][0];      // theta_LF
        traj_final[i][1] = -all_beta[i][0];      // -beta_LF

        // RF => index 1
        traj_final[i][2] = all_theta[i][1];      // theta_RF
        traj_final[i][3] =  all_beta[i][1];      // beta_RF

        // RH => index 2
        traj_final[i][4] = all_theta[i][2];      // theta_RH
        traj_final[i][5] =  all_beta[i][2];      // beta_RH

        // LH => index 3
        traj_final[i][6] = all_theta[i][3];      // theta_LH
        traj_final[i][7] = -all_beta[i][3];      // -beta_LH
    }

    // 8) Save to c_traj.csv
    std::ofstream fout("0212.csv");
    if (!fout.is_open()) {
        std::cerr << "Could not open 0212.csv for writing.\n";
        return;
    }
    for (auto & row : traj_final) {
        for (size_t c=0; c<row.size(); c++) {
            fout << row[c];
            if (c + 1 < row.size()) {
                fout << ",";
            }
        }
        fout << "\n";
    }
    fout.close();

    std::cout << "Saved 0212.csv with "
              << all_theta.size() << " rows.\n";

    std::cout << "= = = Wheel to WLW End = = =" << std::endl;

}

int main()
{
    wlw_transform_main();
    return 0;
}