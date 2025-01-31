#include "wheel_to_leg.hpp"

WheelToLegTransformer::WheelToLegTransformer(double init_eta[8], bool sim) :
    leg_model(sim)
{
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4] = {-init_eta[1], init_eta[3], -init_eta[5], init_eta[7]};

    initialize(init_theta, init_beta);
}

void WheelToLegTransformer::initialize(double init_theta[4], double init_beta[4]){
    for (int i=0; i<4; i++) {
        curr_theta[i] = init_theta[i];
        curr_beta[i] = init_beta[i];
    }

    wheel_delta_beta = -body_vel/leg_model.radius*dt;
}

double WheelToLegTransformer::round_3(double value){
    return std::round(value * 1e3) / 1e3;
}

double WheelToLegTransformer::round_6(double value){
    return std::round(value * 1e6) / 1e6;
}

double WheelToLegTransformer::find_closest_beta(double target_beta, double ref_beta){
    if (round_6(target_beta) < round_6(ref_beta)){
        while (abs(round_6(target_beta)-round_6(ref_beta)) >= M_PI){
            target_beta += 2 * M_PI;
        }
    }
    else if (round_6(target_beta) > round_6(ref_beta)){
        while (abs(round_6(target_beta)-round_6(ref_beta)) >= M_PI){
            target_beta -= 2 * M_PI;
        }
    }
    return target_beta;
}

double WheelToLegTransformer::find_smaller_closest_beta(double target_beta, double ref_beta){
    target_beta = find_closest_beta(target_beta, ref_beta);
    
    if (target_beta > ref_beta){
        target_beta -= 2 * M_PI;
    }
    
    return target_beta;
}

std::array<double, 2> WheelToLegTransformer::find_hybrid_steps(double RH_beta, double LH_beta, double body_angle){
    double step_num = 1;
    double max_step_num = 6;
    double min_step_length = 0.1;
    double max_step_length = 0.2;
    double step_length;

    double RH_target_beta = find_smaller_closest_beta(55/180.0*M_PI-body_angle, RH_beta);
    double LH_target_beta = find_smaller_closest_beta(55/180.0*M_PI-body_angle, LH_beta);
    
    for (int i=0; i<max_step_num; i++){
        if ((int)step_num%2 == 1){
            step_length = abs(leg_model.radius * (RH_target_beta-RH_beta)) / step_num;
            if (step_length < min_step_length){
                RH_target_beta -= 2 * M_PI;
            }
            else if ((min_step_length < step_length) && (step_length < max_step_length)){
                return {step_num, step_length};
            }
        }
        else{
            step_length = abs(leg_model.radius * (LH_target_beta-LH_beta)) / step_num;
            if (step_length < min_step_length){
                LH_target_beta -= 2 * M_PI;
            }
            else if ((min_step_length < step_length) && (step_length < max_step_length)){
                return {step_num, step_length};
            }
        }
        step_num += 1;
    }
    
    return {0, 0};
}
        

std::array<std::array<double, 4>, 2> WheelToLegTransformer::step(){
    switch (stage)
    {
    case 0:  // stay
        if (step_count == 2000) {
            RF_target_beta = 45/180.0*M_PI;
            RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1]);
            body_move_dist = abs(leg_model.radius * (RF_target_beta-curr_beta[1]));
            delta_time_step = int(round_3(body_move_dist/body_vel)/dt);
            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 0 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;
        
    case 1:  // rotate until RF_beta ~= 45 deg
        for (int i=0; i<4; i++){
            curr_beta[i] += wheel_delta_beta;
        }
        if (step_count == delta_time_step){
            body_angle = asin(((stance_height-leg_model.radius)/BL));
            RF_target_beta = -body_angle;
            RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1]);
            G_p[0] = 0;
            G_p[1] = -stance_height+leg_model.r;
            RF_target_theta = leg_model.inverse(G_p, "G")[0];

            std::cout << "Body Angle = " << round_3(body_angle) << std::endl;
            std::cout << "RF Target Theta = " << round_3(RF_target_theta) << std::endl;
            std::cout << "RF Target Beta = " << round_3(RF_target_beta) << std::endl;

            body_move_dist = abs(leg_model.radius * (RF_target_beta-curr_beta[1]));
            delta_time_step = int(round_3(body_move_dist/body_vel)/dt);

            RF_delta_beta = (RF_target_beta-curr_beta[1])/delta_time_step;
            RF_delta_theta = (RF_target_theta-curr_theta[1])/delta_time_step;

            // # determine the step length from hybrid mode
            hybrid_steps = find_hybrid_steps(curr_beta[2]+wheel_delta_beta*delta_time_step,
                                             curr_beta[3]+wheel_delta_beta*delta_time_step, body_angle);
            step_num = hybrid_steps[0];
            step_length = hybrid_steps[1];

            G_p[0] = step_length;
            G_p[1] = -stance_height+leg_model.r;
            LF_target_theta = leg_model.inverse(G_p, "G")[0];
            LF_target_beta = leg_model.inverse(G_p, "G")[1];
            LF_target_beta = find_smaller_closest_beta(LF_target_beta-body_angle, curr_beta[0]);

            LF_delta_beta = (LF_target_beta-curr_beta[0])/delta_time_step;
            LF_delta_theta = (LF_target_theta-curr_theta[0])/delta_time_step;

            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 1 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;

    case 2:  // front transform
        if (step_count < delta_time_step/3.0) { curr_beta[0] += wheel_delta_beta; }
        else if (step_count < delta_time_step*2/3.0) { curr_beta[0] += LF_delta_beta*3 - wheel_delta_beta; }
        else { curr_theta[0] += LF_delta_theta * 3; }
        
        curr_theta[1] += RF_delta_theta;
        curr_beta[1] += RF_delta_beta;
        
        curr_beta[2] += wheel_delta_beta;
        curr_beta[3] += wheel_delta_beta;

        if (step_count == delta_time_step){
            delta_time_step_each = int(round_3(step_length/body_vel)/dt);
            delta_time_step = delta_time_step_each * step_num;

            std::cout << "Step Number = " << step_num << std::endl;
            std::cout << "Step Length = " << round_3(step_length) << std::endl;

            // swing phase trajectory
            // sp = swing.SwingProfile(step_length, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -stance_height+leg_model.r, 0.0);
            // d = np.linspace(0, 1, delta_time_step_each)
            // curve_points = [sp.getFootendPoint(_) for _ in d]

            hip_pos = {0, 0};
            // for point in curve_points:
            //     [swing_theta, swing_beta] = leg.inverse(point-hip_pos)
            //     swing_target_theta_traj.append(swing_theta)
            //     swing_target_beta_traj.append(swing_beta-body_angle)
            //     hip_pos[0] += step_length / delta_time_step_each

            // stance phase trajectory
            move_vector = {step_length, 0};
            stance_theta = swing_target_theta_traj[-1];
            stance_beta = swing_target_beta_traj[-1]+body_angle;
            // for t in range(delta_time_step_each):
            //     [stance_theta, stance_beta] = leg.move(stance_theta, stance_beta, move_vector/delta_time_step_each)
            //     stance_target_theta_traj.append(stance_theta)
            //     stance_target_beta_traj.append(stance_beta-body_angle)

            leg_model.forward(stance_target_theta_traj[0], stance_target_beta_traj[0]+body_angle);
            std::cout << "Stance Start Point = " << round_3(leg_model.G[0]) << ", " << round_3(leg_model.G[1]-leg_model.r) << std::endl;
            leg_model.forward(stance_target_theta_traj[-1], stance_target_beta_traj[-1]+body_angle);
            std::cout << "Stance End Point = " << round_3(leg_model.G[0]) << ", " << round_3(leg_model.G[1]-leg_model.r) << std::endl;
            leg_model.forward(swing_target_theta_traj[0], swing_target_beta_traj[0]+body_angle);
            std::cout << "Swing Start Point = " << round_3(leg_model.G[0]) << ", " << round_3(leg_model.G[1]-leg_model.r) << std::endl;
            leg_model.forward(swing_target_theta_traj[-1], swing_target_beta_traj[-1]+body_angle);
            std::cout << "Swing End Point = " << round_3(leg_model.G[0]) << ", " << round_3(leg_model.G[1]-leg_model.r) << std::endl;


            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 2 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;

    case 3:  // hybrid mode
        traj_idx = step_count % delta_time_step_each;
        if ((step_count / delta_time_step_each) % 2 == 0){
            curr_theta[1] = swing_target_theta_traj[traj_idx];
            curr_beta[1] = find_closest_beta(swing_target_beta_traj[traj_idx], curr_beta[1]);
            
            curr_theta[0] = stance_target_theta_traj[traj_idx];
            curr_beta[0] = find_closest_beta(stance_target_beta_traj[traj_idx], curr_beta[0]);
        }
        else{
            curr_theta[0] = swing_target_theta_traj[traj_idx];
            curr_beta[0] = find_closest_beta(swing_target_beta_traj[traj_idx], curr_beta[0]);
            
            curr_theta[1] = stance_target_theta_traj[traj_idx];
            curr_beta[1] = find_closest_beta(stance_target_beta_traj[traj_idx], curr_beta[1]);
        }
        curr_beta[2] += wheel_delta_beta;
        curr_beta[3] += wheel_delta_beta;

        if (step_count == delta_time_step){

            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 3 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;

    case 4:  // maintain stability

        if (step_count == delta_time_step){

            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 4 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;

    case 5:  // hind transform -> final transform
    
        if (step_count == delta_time_step){

            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 5 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;

    case 6:  // maintain stability

        if (step_count == delta_time_step){

            step_count = 0;
            stage++;
            std::cout << std::endl << "Stage 6 Finished.\n= = = = =" << std::endl << std::endl;
        }
        break;

    case 7:  // transform finished
        transform_finished = true;

    default:
        break;
    }

    step_count++;

    return {curr_theta, curr_beta};
}