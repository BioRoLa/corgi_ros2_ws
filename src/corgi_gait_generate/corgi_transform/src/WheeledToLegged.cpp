// WheeledToLegged.cpp
#include <iostream>
#include "transform_gen.hpp"

class WheeledToLegged : public IGaitTransform, public Legged 
{
    public:
        WheeledToLegged(ros::NodeHandle& nh) : Legged(nh){}
        
        void transform(double shift, int wait_step, bool transfer_state, double expect_height) override {
            for (int i=0; i<4; i++) {
                curr_theta[i] = eta[i][0];
                curr_beta[i] = -eta[i][1];
                
            }
        
            wheel_delta_beta = -velocity/leg_model.radius*dt;
            
            stage = 0;
            transform_finished = false;
            stand_height = 0.18;

            while(transform_finished != true){
                keep_going();
                for (int i=0; i<4; i++) {
                    eta[i][0] = curr_theta[i];
                    eta[i][1] = -curr_beta[i];
                    
                }
                Send(1);
            }
            

        }
    private:
        std::array<double, 4> curr_theta;
        std::array<double, 4> curr_beta;
        double wheel_delta_beta;
        double dt = 0.001;
        int step_count = 0;
        int stay_time_step = 0;
        int stage = 0;
        bool transform_finished = false;
        double body_move_dist;
        double delta_time_step;
        double body_angle;
        double G_p[2] = {0, 0};
        std::array<double, 2> hybrid_steps;
        int step_num;
        int delta_time_step_each;

        double RF_target_beta, RF_target_theta;
        double LF_target_beta, LF_target_theta;

        double RF_delta_beta, RF_delta_theta;
        double LF_delta_beta, LF_delta_theta;

        double last_transform_step_x = 0.15;
        std::array<double, 2> p_lo, p_td;
        SwingProfile sp;

        std::array<double, 2> curve_point_temp;
        double curve_point[2];
        std::array<double, 2> swing_eta;

        std::array<double, 2> stance_eta;
        std::array<double, 2> move_vector;

        int traj_idx;

        double transform_start_beta, transform_start_theta;
        double transform_target_beta, transform_target_theta;
        double transform_delta_beta, transform_delta_theta;

        double last_start_beta, last_start_theta;
        double last_target_theta, last_target_beta;
        double last_delta_beta, last_delta_theta;

        double hind_body_height;
        
        double LF_target_pos[2], RF_target_pos[2];

        double find_closest_beta(double target_beta, double ref_beta){
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
        double find_smaller_closest_beta(double target_beta, double ref_beta){
            target_beta = find_closest_beta(target_beta, ref_beta);
            
            if (target_beta > ref_beta){
                target_beta -= 2 * M_PI;
            }
            
            return target_beta;
        }
        double round_3(double value){
            return std::round(value * 1e3) / 1e3;
        }  
        double round_6(double value){
            return std::round(value * 1e6) / 1e6;
        }
        std::array<double, 2> find_hybrid_steps(double RH_beta, double LH_beta, double body_angle){
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

        std::array<std::array<double, 4>, 2> keep_going(){
            switch (stage){
                case 0:  // stay
                    if (step_count == stay_time_step) {
                        RF_target_beta = 45/180.0*M_PI;
                        RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1]);
                        body_move_dist = abs(leg_model.radius * (RF_target_beta-curr_beta[1]));
                        delta_time_step = int(round_3(body_move_dist/velocity)/dt);
                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 0 Finished.\n= = = = =" << std::endl;
                    }
                    break;
                    
                case 1:  // rotate until RF_beta ~= 45 deg
                    for (int i=0; i<4; i++){
                        curr_beta[i] += wheel_delta_beta;
                    }
                    if (step_count == delta_time_step-1){
                        body_angle = asin(((stand_height-leg_model.radius)/BL));
                        RF_target_beta = -body_angle;
                        RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1]);
                        G_p[0] = 0;
                        G_p[1] = -stand_height+leg_model.r;
                        RF_target_theta = leg_model.inverse(G_p, "G")[0];

                        // std::cout << "Body Angle = " << round_3(body_angle) << std::endl;
                        // std::cout << "RF Target Theta = " << round_3(RF_target_theta) << std::endl;
                        // std::cout << "RF Target Beta = " << round_3(RF_target_beta) << std::endl;

                        body_move_dist = abs(leg_model.radius * (RF_target_beta-curr_beta[1]));
                        delta_time_step = int(round_3(body_move_dist/velocity)/dt);

                        RF_delta_beta = (RF_target_beta-curr_beta[1])/delta_time_step;
                        RF_delta_theta = (RF_target_theta-curr_theta[1])/delta_time_step;

                        // # determine the step length from hybrid mode
                        hybrid_steps = find_hybrid_steps(curr_beta[2]+wheel_delta_beta*delta_time_step,
                                                        curr_beta[3]+wheel_delta_beta*delta_time_step, body_angle);
                        step_num = (int)hybrid_steps[0];
                        step_length = hybrid_steps[1];

                        G_p[0] = step_length;
                        G_p[1] = -stand_height+leg_model.r;
                        LF_target_theta = leg_model.inverse(G_p, "G")[0];
                        LF_target_beta = leg_model.inverse(G_p, "G")[1];
                        LF_target_beta = find_smaller_closest_beta(LF_target_beta-body_angle, curr_beta[0]);

                        LF_delta_beta = (LF_target_beta-curr_beta[0])/delta_time_step;
                        LF_delta_theta = (LF_target_theta-curr_theta[0])/delta_time_step;

                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 1 Finished.\n= = = = =" << std::endl << std::endl;
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
            
                    if (step_count == delta_time_step-1){
                        delta_time_step_each = int(round_3(step_length/velocity)/dt);
                        delta_time_step = delta_time_step_each * step_num;
            
                        // std::cout << "Step Number = " << step_num << std::endl;
                        // std::cout << "Step Length = " << round_3(step_length) << std::endl;
            
                        // swing phase trajectory
                        p_lo = {0, -stand_height+leg_model.r};
                        p_td = {step_length, -stand_height+leg_model.r};
                        sp = SwingProfile(p_lo, p_td, step_height, 0);
            
                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 2 Finished.\n= = = = =" << std::endl << std::endl;
                    }
                    break;
            
                case 3:  // hybrid mode
                    traj_idx = step_count % delta_time_step_each;
                    curve_point_temp = sp.getFootendPoint(traj_idx/(double)delta_time_step_each);
                    curve_point[0] = curve_point_temp[0];
                    curve_point[1] = curve_point_temp[1];
            
                    swing_eta = leg_model.inverse(curve_point, "G");
            
                    if ((step_count / delta_time_step_each) % 2 == 0){
                        curr_theta[1] = swing_eta[0];
                        curr_beta[1] = find_closest_beta(swing_eta[1]-body_angle, curr_beta[1]);
            
                        stance_eta = leg_model.move(curr_theta[0], curr_beta[0]+body_angle, {step_length/(double)delta_time_step_each, 0});
                        curr_theta[0] = stance_eta[0];
                        curr_beta[0] = find_closest_beta(stance_eta[1]-body_angle, curr_beta[0]);
                    }
                    else{
                        curr_theta[0] = swing_eta[0];
                        curr_beta[0] = find_closest_beta(swing_eta[1]-body_angle, curr_beta[0]);
            
                        stance_eta = leg_model.move(curr_theta[1], curr_beta[1]+body_angle, {step_length/(double)delta_time_step_each, 0});
                        curr_theta[1] = stance_eta[0];
                        curr_beta[1] = find_closest_beta(stance_eta[1]-body_angle, curr_beta[1]);
                    }
                    curr_beta[2] += wheel_delta_beta;
                    curr_beta[3] += wheel_delta_beta;
            
                    if (step_count == delta_time_step-1){
                        body_move_dist = leg_model.radius * 10/180.0*M_PI;
                        delta_time_step = int(round_3(body_move_dist/velocity)/dt);
            
                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 3 Finished.\n= = = = =" << std::endl << std::endl;
                    }
                    break;
            
                case 4:  // maintain stability
            
                    for (int i=0; i<2; i++){
                        stance_eta = leg_model.move(curr_theta[i], curr_beta[i]+body_angle, {body_move_dist/delta_time_step, 0});
                        curr_theta[i] = stance_eta[0];
                        curr_beta[i] = find_closest_beta(stance_eta[1]-body_angle, curr_beta[i]);
                    }
            
                    curr_beta[2] += wheel_delta_beta;
                    curr_beta[3] += wheel_delta_beta;
            
                    if (step_count == delta_time_step){
            
                        body_move_dist = leg_model.radius * (45/180.0*M_PI - body_angle);
                        delta_time_step = int(round_3(body_move_dist/velocity)/dt);
            
                        if (step_num%2 == 0) {
                            transform_start_beta = curr_beta[3];
                            transform_start_theta = curr_theta[3];
                        }
                        else {
                            transform_start_beta = curr_beta[2];
                            transform_start_theta = curr_theta[2];
                        }
            
                        transform_target_beta = find_smaller_closest_beta(0, transform_start_beta);
                        G_p[0] = 0;
                        G_p[1] = -stand_height+leg_model.r;
                        transform_target_theta = leg_model.inverse(G_p, "G")[0];
            
                        transform_delta_beta = (transform_target_beta-transform_start_beta)/delta_time_step;
                        transform_delta_theta = (transform_target_theta-transform_start_theta)/delta_time_step;
            
                        if (step_num%2 == 0) {
                            last_start_beta = curr_beta[2];
                            last_start_theta = curr_theta[2];
                        }
                        else {
                            last_start_beta = curr_beta[3];
                            last_start_theta = curr_theta[3];
                        }
            
                        G_p[0] = last_transform_step_x;
                        G_p[1] = -stand_height+leg_model.r;
                        last_target_theta = leg_model.inverse(G_p, "G")[0];
                        last_target_beta = leg_model.inverse(G_p, "G")[1];
                        last_target_beta = find_smaller_closest_beta(last_target_beta, last_start_beta);
            
                        last_delta_beta = (last_target_beta-last_start_beta-wheel_delta_beta*delta_time_step*2/3.0)/(delta_time_step/3.0);
                        last_delta_theta = (last_target_theta-last_start_theta)/delta_time_step;
            
                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 4 Finished.\n= = = = =" << std::endl << std::endl;
                    }
                    break;
            
                case 5:  // hind transform
                    leg_model.contact_map(transform_start_theta+transform_delta_theta*step_count,
                                        transform_start_beta+transform_delta_beta*step_count+body_angle);
                    leg_model.forward(transform_start_theta+transform_delta_theta*step_count,
                                    transform_start_beta+transform_delta_beta*step_count+body_angle);
                    
                    if (leg_model.rim == 2){
                        hind_body_height = abs(leg_model.L_l[1] - leg_model.radius);
                    }
                    else if (leg_model.rim == 3){
                        hind_body_height = abs(leg_model.G[1] - leg_model.r);
                    }
            
                    body_angle = asin(((stand_height-hind_body_height)/BL));
            
                    leg_model.forward(curr_theta[0], curr_beta[0]+body_angle);
                    LF_target_pos[0] = leg_model.G[0]-body_move_dist/delta_time_step;
                    LF_target_pos[1] = -stand_height+leg_model.r;
            
                    LF_target_theta = leg_model.inverse(LF_target_pos, "G")[0];
                    LF_target_beta = leg_model.inverse(LF_target_pos, "G")[1];
                    LF_target_beta = find_smaller_closest_beta(LF_target_beta-body_angle, curr_beta[0]);
                    
            
                    leg_model.forward(curr_theta[1], curr_beta[1]+body_angle);
                    RF_target_pos[0] = leg_model.G[0]-body_move_dist/delta_time_step;
                    RF_target_pos[1] = -stand_height+leg_model.r;
            
                    RF_target_theta = leg_model.inverse(RF_target_pos, "G")[0];
                    RF_target_beta = leg_model.inverse(RF_target_pos, "G")[1];
                    RF_target_beta = find_smaller_closest_beta(RF_target_beta-body_angle, curr_beta[1]);
                    
                    curr_theta[0] = LF_target_theta;
                    curr_beta[0] = LF_target_beta;
                    
                    curr_theta[1] = RF_target_theta;
                    curr_beta[1] = RF_target_beta;
                    
                    if (step_num%2 == 0){
                        curr_theta[3] += transform_delta_theta;
                        curr_beta[3] += transform_delta_beta;
                        
                        if (step_count < delta_time_step/3.0){
                            curr_beta[2] += wheel_delta_beta;
                        }
                        else if (step_count < delta_time_step*2/3.0){
                            curr_beta[2] += last_delta_beta;
                            curr_theta[2] += last_delta_theta * 3/2.0;
                        }
                        else {
                            curr_beta[2] += wheel_delta_beta;
                            curr_theta[2] += last_delta_theta * 3/2.0;
                        }
                    }
                    else {
                        curr_theta[2] += transform_delta_theta;
                        curr_beta[2] += transform_delta_beta;
                        
                        if (step_count < delta_time_step/3.0){
                            curr_beta[3] += wheel_delta_beta;
                        }
                        else if (step_count < delta_time_step*2/3.0){
                            curr_beta[3] += last_delta_beta;
                            curr_theta[3] += last_delta_theta * 3/2.0;
                        }
                        else {
                            curr_beta[3] += wheel_delta_beta;
                            curr_theta[3] += last_delta_theta * 3/2.0;
                        }
                    }
            
                    if (step_count == delta_time_step){
                        body_move_dist = 0.02;
                        delta_time_step = int(round_3(body_move_dist/velocity)/dt);
            
                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 5 Finished.\n= = = = =" << std::endl << std::endl;
                    }
                    break;
            
                case 6:  // maintain stability
                    for (int i=0; i<4; i++){
                        // std::cout << i << std::endl;
                        stance_eta = leg_model.move(curr_theta[i], curr_beta[i]+body_angle, {body_move_dist/delta_time_step, 0});
                        // std::cout << "feed" << std::endl;
                        curr_theta[i] = stance_eta[0];
                        // std::cout << "beta" << std::endl;
                        curr_beta[i] = find_closest_beta(stance_eta[1]-body_angle, curr_beta[i]);
                    }
            
                    if (step_count == delta_time_step){
                        step_count = 0;
                        stage++;
                        // std::cout << std::endl << "Stage 6 Finished.\n= = = = =" << std::endl << std::endl;
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
};

extern "C" IGaitTransform* createWheeledToLegged() {
    ros::NodeHandle nh;
    return new WheeledToLegged(nh);
}

