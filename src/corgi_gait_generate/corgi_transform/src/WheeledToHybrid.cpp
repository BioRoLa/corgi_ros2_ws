// WheeledToHybrid.cpp
#include <iostream>
#include "transform_gen.hpp"

class WheeledToHybrid : public IGaitTransform 
{
    public:
        WheeledToHybrid(std::shared_ptr<Hybrid> transform_hybrid_ptr,std::shared_ptr<Legged> transform_leg_ptr)
            : transform_hybrid(transform_hybrid_ptr),inherit(transform_hybrid_ptr->gaitSelector) {}
        ~WheeledToHybrid() override = default;
        // Inherit the gaitSelector from Hybrid
        std::shared_ptr<GaitSelector> inherit;
        void transform() override {
            std::cout << "Enter transform" << std::endl;
            
           
            inherit->Receive();
            wheel_delta_beta = inherit->velocity/(inherit->leg_model.radius*inherit->pub_rate);
            
            stage = 0;
            transform_finished = false;
            inherit->stand_height = 0.16;
            for (int i=0; i<4; i++) {
                inherit->current_stand_height[i]  = inherit->stand_height;
            }
            while(transform_finished != true){
                keep_going();
                inherit->Send();
            }
        }           
    
    private:
        std::shared_ptr<Hybrid> transform_hybrid;
        int stage;
        bool transform_finished;
        double wheel_delta_beta;
        double check_beta[2];
        int which_leg;
        int index ;
        double terrain_slope;
        double transform_angle = 45 * PI / 180.0; // 45 degrees in radians
        double body_move_dist;
        int delta_time_step;
        double body_angle;
        double target_theta[2];
        double target_beta[2];
        double delta_theta[2];
        double delta_beta[2];
        double hybrid_steps;
        double closer_beta(double ref_rad, int leg_index)
        {
            double val0 = inherit->eta[leg_index][1];
            double delta_beta;
            while (val0 < ref_rad){
                    ref_rad -= 2*PI;
            }
            delta_beta = val0 - ref_rad ;
            return delta_beta;
        }
        struct StepStrategyResult {
            int step_num;
            double step_length;
            int last_leg;   // "RH" 或 "LH"
        };
        double adjust_beta(double target, double start){
            double adjusted = target;
            while (adjusted > start) {
                adjusted -= 2*PI; // beta會越來越小
            } 
            return adjusted;
        }
        int step_count = 0;
        int delta_time_step_each;    
        int traj_idx;
        int temp_leg ;
        int clamp_index;
        double phase_ratio ;
        std::array<double, 2> variation;
        std::array<double, 2> swing_eta;
        std::array<double, 3> find_hybrid_steps(double RH_beta, double LH_beta, int which) {
            // std::cout << "RH_beta" <<  RH_beta << " , LH_beta = " << LH_beta << std::endl;
            // std::cout << "which leg = " << which << std::endl;
            double step_num = 1;
            double max_step_num = 6;
            double min_step_length = 0.1;
            double max_step_length = 0.2;
            double step_length;

            // target = 45 *pi/180 
            double RH_target_beta = adjust_beta(transform_angle , RH_beta);
            double LH_target_beta = adjust_beta(transform_angle , LH_beta);
            // beta會越來越小
            // 若偶數步 → which_leg同側 move, 異側swing
            // 若奇數步 → which_leg異側 move, 同側swing
            for (int i = 0; i < max_step_num; ++i) {
                int leg_index = -1;

                if ((int)step_num % 2 == 1) {  // 奇數：if which leg = 1, RH touchdown（對應 RF move）
                    if (which==1){
                        step_length = std::abs(inherit->leg_model.radius * (RH_target_beta - RH_beta)) / step_num;
                        if (step_length < min_step_length) {
                            RH_target_beta -= 2 * M_PI;
                            step_length = std::abs(inherit->leg_model.radius * (RH_target_beta - RH_beta)) / step_num;
                        } else if (step_length < max_step_length) {
                            leg_index = 3;  // LH swing
                            // std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 3;  // LH swing
                            // std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                    }
                    else{
                        step_length = std::abs(inherit->leg_model.radius * (LH_target_beta - LH_beta)) / step_num;
                        if (step_length < min_step_length) {
                            LH_target_beta -= 2 * M_PI;
                            step_length = std::abs(inherit->leg_model.radius * (LH_target_beta - LH_beta)) / step_num;
                        } else if (step_length < max_step_length) {
                            leg_index = 2;  // RH swing
                            // std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 2;  // RH swing
                            // std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                    }
                    
                } else {  // 偶數：if which leg = 1; LH touchdown
                    if(which==1){
                        step_length = std::abs(inherit->leg_model.radius * (LH_target_beta - LH_beta)) / step_num;
                        if (step_length < min_step_length) {
                            LH_target_beta -= 2 * M_PI;
                            step_length = std::abs(inherit->leg_model.radius * (LH_target_beta - LH_beta)) / step_num;
                        } else if (step_length < max_step_length) {
                            leg_index = 2;  // RH swing
                            // std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 2;  // RH swing
                            // std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                    }
                    else{
                        step_length = std::abs(inherit->leg_model.radius * (RH_target_beta - RH_beta)) / step_num;
                        if (step_length < min_step_length) {
                            RH_target_beta -= 2 * M_PI;
                            step_length = std::abs(inherit->leg_model.radius * (RH_target_beta - RH_beta)) / step_num;
                        } else if (step_length < max_step_length) {
                            leg_index = 3;  // LH swing
                            // std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 3;  // LH swing
                            // std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                    }
                  
                }

                step_num += 1;
            }

            // 無合適結果
            // std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
            return {0.0, 0.0, 0.0};
        }
        StepStrategyResult result;
        double transform_start_beta, transform_start_theta,transform_target_beta,transform_target_theta,last_start_beta, last_start_theta;
        double transform_delta_beta ;
        double transform_delta_theta ;
        double last_delta_beta ;
        double last_delta_theta;
        void keep_going(){
            switch (stage){
                case 0:  // cal
                    if (step_count == 0) {
                        inherit->step_length = 0.2;
                        inherit->dS = inherit->velocity / inherit->pub_rate;
                        inherit->incre_duty = inherit->dS / inherit->step_length;
                        check_beta[0] = closer_beta( transform_angle, 0);
                        check_beta[1] = closer_beta( transform_angle, 1);  
                        if(check_beta[0]<= check_beta[1]){ which_leg = 0;}
                        else{which_leg = 1;}
                        body_move_dist = (inherit->leg_model.radius * check_beta[which_leg]);
                        delta_time_step = int(check_beta[which_leg]/wheel_delta_beta);
                        step_count = 0;
                        stage++;
                        std::cout << std::endl << "= = = = = Stage 0 Finished = = = = =" << std::endl;
                    }
                    break;
                case 1: // rotate until the closest front leg beta ~= 45 deg
                    for (int j=0;j<4;j++){
                        inherit->next_eta[j][0] = 17 * PI/180;
                        inherit->next_eta[j][1] = inherit->eta[j][1] -  wheel_delta_beta;
                    }
                    if(step_count==delta_time_step-1){
                        terrain_slope  = atan(inherit->stand_height-inherit->leg_model.radius/inherit->BL);
                        auto temp = transform_hybrid->find_pose(inherit->stand_height, 0, inherit->step_length/2, 0.4 , -terrain_slope);
                        // std::cout << "Target Theta(which_leg) = " << temp[0] *180/PI <<" , " << temp[1] *180/PI  << std::endl;
                        target_theta[which_leg] = temp[0];
                        target_beta[which_leg] = temp[1];
                        check_beta[which_leg] = closer_beta(target_beta[which_leg], which_leg);
                        body_move_dist = abs(inherit->leg_model.radius * check_beta[which_leg]);
                        delta_time_step = int(check_beta[which_leg]/wheel_delta_beta);

                        delta_beta[which_leg] = check_beta[which_leg]/delta_time_step;
                        delta_theta[which_leg] = (target_theta[which_leg] - inherit->eta[which_leg][0]) / delta_time_step;
                        index = (which_leg == 0) ? 1 : 0; 

                        // determine the step length from hybrid mode and decide the land pose of index leg
                        auto hybrid_steps = find_hybrid_steps(inherit->next_eta[2][1]-wheel_delta_beta*delta_time_step,
                                                              inherit->next_eta[3][1]-wheel_delta_beta*delta_time_step, which_leg);
                        
                        result.step_num = (int)hybrid_steps[0];
                        result.step_length = hybrid_steps[1];
                        result.last_leg  = (int)hybrid_steps[2];
                        std::cout << "step_num = " << result.step_num << ", step_length = " << result.step_length << ", last_leg = " << result.last_leg << std::endl;
                        temp_leg =  (result.last_leg == 3) ? 2 : 3; 
                        inherit->step_length = result.step_length;
                        inherit->incre_duty = inherit->dS / inherit->step_length;
                        
                        // auto pose = transform_hybrid->find_pose(inherit->stand_height, 0, inherit->step_length/2, 0.0 , -terrain_slope);
                        auto pose = temp;
                        for (double t = 0.0; t <= inherit->step_length; t += inherit->dS) {
                            pose = inherit->leg_model.move(pose[0], pose[1], {-inherit->dS, 0},terrain_slope);
                            
                        }
                        target_theta[index]= pose[0];
                        target_beta[index] = pose[1];
                        swing_eta = {target_theta[index], target_beta[index]};
                        check_beta[index] = closer_beta(target_beta[index], index);
                        delta_beta[index] = check_beta[index]/delta_time_step;
                        delta_theta[index] = (target_theta[index] - inherit->eta[index][0]) / delta_time_step; 

                        step_count = 0;
                        stage++;
                        std::cout << std::endl << "= = = = = Stage 1 Finished = = = = =" << std::endl;
                    }
                   
                    break;
                case 2:  // front transform
                    if (step_count < delta_time_step/3.0) { 
                        inherit->next_eta[index][1] -= wheel_delta_beta; 
                    }
                    else if (step_count < delta_time_step*2/3.0) { 
                        inherit->next_eta[index][1] -= (delta_beta[index]*3 - wheel_delta_beta); 
                    }
                    else { 
                        inherit->next_eta[index][0] += delta_theta[index] * 3; 

                    }
                        
                    inherit->next_eta[which_leg][0] += delta_theta[which_leg];
                    inherit->next_eta[which_leg][1] -= delta_beta[which_leg];
            
                    inherit->next_eta[2][1] -= wheel_delta_beta;
                    inherit->next_eta[3][1] -= wheel_delta_beta;
            
                    if (step_count == delta_time_step-1){ 
                        // 若偶數步 → which_leg同側 擺動(which_leg)
                        // 若奇數步 → which_leg異側 擺動  
                        delta_time_step_each = int(inherit->step_length/inherit->dS);
                        // std::cout << "delta_time_step_each = " << delta_time_step_each << std::endl;
                        delta_time_step = delta_time_step_each * result.step_num;
                        // swing phase trajectory of first leg
                        auto swing_end = transform_hybrid->find_pose(inherit->stand_height, 0, inherit->step_length/2, 0.4 , -terrain_slope);
                        for (double t = 0.0; t <= inherit->step_length; t += inherit->dS) {
                            swing_end = inherit->leg_model.move(swing_end[0], swing_end[1], {-inherit->dS, 0},terrain_slope);
                            
                        }
                        transform_hybrid->Swing(inherit->eta, swing_end, transform_hybrid->swing_variation, which_leg);
                        transform_hybrid->swing_traj[index] = transform_hybrid->swing_traj[which_leg];
                        step_count = 0;
                        stage++;
                        std::cout << std::endl << "= = = = = Stage 2 Finished = = = = =" << std::endl;
                    }
                    break;
                case 3: // hybrid mode
                    traj_idx = step_count % delta_time_step_each;
                    
                    phase_ratio = static_cast<double>(traj_idx) / delta_time_step_each;
                    clamp_index = static_cast<int>(phase_ratio * 500);
                    if ((step_count / delta_time_step_each) % 2 == 0){
                        inherit->next_eta[which_leg][0] = transform_hybrid->swing_traj[which_leg][clamp_index].theta;
                        inherit->next_eta[which_leg][1] = adjust_beta(transform_hybrid->swing_traj[which_leg][clamp_index].beta, inherit->eta[which_leg][1]);
            
                        auto stance_eta = inherit->leg_model.move(inherit->next_eta[index][0], inherit->next_eta[index][1], {inherit->dS, 0}, -terrain_slope);
                        inherit->next_eta[index][0] = stance_eta[0];
                        inherit->next_eta[index][1] = adjust_beta(stance_eta[1], inherit->next_eta[index][1]);
                    }
                    else{
                        inherit->next_eta[index][0] = transform_hybrid->swing_traj[index][clamp_index].theta;
                        inherit->next_eta[index][1] = adjust_beta(transform_hybrid->swing_traj[index][clamp_index].beta, inherit->eta[index][1]);
            
                        auto stance_eta = inherit->leg_model.move(inherit->next_eta[which_leg][0], inherit->next_eta[which_leg][1], {inherit->dS, 0}, -terrain_slope);
                        inherit->next_eta[which_leg][0] = stance_eta[0];
                        inherit->next_eta[which_leg][1] = adjust_beta(stance_eta[1], inherit->next_eta[which_leg][1]);
                    
                    }
                    inherit->next_eta[2][1] -= wheel_delta_beta;
                    inherit->next_eta[3][1] -= wheel_delta_beta;
            
                    if (step_count == delta_time_step-1){  
                        // 若偶數步 → which_leg同側 擺動(which_leg)
                        // 若奇數步 → which_leg異側 擺動          
                        transform_start_theta = inherit->next_eta[temp_leg][0];
                        transform_start_beta = inherit->next_eta[temp_leg][1];  
                        auto temp = transform_hybrid->find_pose(inherit->stand_height, 0, inherit->step_length, 0.4 , 0);
                        transform_target_theta = temp[0];
                        transform_target_beta = temp[1];
                        transform_target_beta = adjust_beta(transform_target_beta,transform_start_beta);

                        body_move_dist = abs(inherit->leg_model.radius * (transform_target_beta-transform_start_beta));
                        delta_time_step = body_move_dist*inherit->pub_rate/inherit->velocity;
                        

                        transform_delta_beta = abs(transform_target_beta-transform_start_beta)/delta_time_step;
                        transform_delta_theta = (transform_target_theta-transform_start_theta)/delta_time_step;
                        
                        last_start_beta = inherit->next_eta[result.last_leg][1];
                        last_start_theta = inherit->next_eta[result.last_leg][0];
                        auto pose = transform_hybrid->find_pose(inherit->stand_height, 0, inherit->step_length, 0.4 , 0);
                        for (double t = 0.0; t <= inherit->step_length; t += inherit->dS) {
                            pose = inherit->leg_model.move(pose[0], pose[1], {-inherit->dS, 0},0);
                            
                        }
                        double last_target_theta = pose[0];
                        double last_target_beta = pose[1];
                        last_target_beta = adjust_beta(last_target_beta, last_start_beta);

                        last_delta_beta = abs(last_target_beta-last_start_beta)/(delta_time_step);
                        last_delta_theta = (last_target_theta-last_start_theta)/delta_time_step;

                        step_count = 0;
                        stage++;
                        std::cout << std::endl << "= = = = = Stage 3 Finished = = = = =" << std::endl;
                    }
                    break;
                case 4: // hind transform
                    // for front leg the front one: duty = 0.0 keep move with  slope = 0
                    if (result.step_num %2 ==0){
                        // even step -> the front = index leg
                        auto stance_eta = inherit->leg_model.move(inherit->next_eta[index][0], inherit->next_eta[index][1], {inherit->dS, 0},0);
                        inherit->next_eta[index][0] = stance_eta[0];
                        inherit->next_eta[index][1] = adjust_beta(stance_eta[1], inherit->next_eta[index][1]);

                        stance_eta = inherit->leg_model.move(inherit->next_eta[which_leg][0], inherit->next_eta[which_leg][1], {inherit->dS, 0},terrain_slope);
                        inherit->next_eta[which_leg][0] = stance_eta[0];
                        inherit->next_eta[which_leg][1] = adjust_beta(stance_eta[1], inherit->next_eta[which_leg][1]);

                    }
                    else{
                        // odds step -> the front = which leg
                        auto stance_eta = inherit->leg_model.move(inherit->next_eta[which_leg][0], inherit->next_eta[which_leg][1], {inherit->dS, 0},0);
                        inherit->next_eta[which_leg][0] = stance_eta[0];
                        inherit->next_eta[which_leg][1] = adjust_beta(stance_eta[1], inherit->next_eta[which_leg][1]);

                        stance_eta = inherit->leg_model.move(inherit->next_eta[index][0], inherit->next_eta[index][1], {inherit->dS, 0},terrain_slope);
                        inherit->next_eta[index][0] = stance_eta[0];
                        inherit->next_eta[index][1] = adjust_beta(stance_eta[1], inherit->next_eta[index][1]);

                    }

                    inherit->next_eta[temp_leg][0] += transform_delta_theta;
                    inherit->next_eta[temp_leg][1] -= transform_delta_beta;
                    if (step_count < delta_time_step/3.0){
                        inherit->next_eta[result.last_leg][1] -= wheel_delta_beta;
                    }
                    else if (step_count < delta_time_step*2/3.0){
                        inherit->next_eta[result.last_leg][1] -= (last_delta_beta*3 - wheel_delta_beta);
                    }
                    else {
                        inherit->next_eta[result.last_leg][0] += last_delta_theta * 3;
                    }
                    if (step_count == delta_time_step){
                        body_move_dist = 0.02;
                        delta_time_step = (int)body_move_dist/inherit->dS;
                        step_count = 0;
                        stage++;
                        std::cout << std::endl << "= = = = = Stage 4 Finished = = = = =" << std::endl;
                    }
                    break;
                case 5: // maintain and connect to hybrid mode
                    inherit->leg_model.contact_map(inherit->next_eta[temp_leg][0], inherit->next_eta[temp_leg][1],0);
                    inherit->stand_height = -inherit->leg_model.contact_p[1];
                    // swing a leg and stop before four leg td
                    for(int i=0; i<4; i++){
                        inherit->current_stand_height[i] = inherit->stand_height;
                    }
                    // set duty for all leg 
                    if (result.step_num %2 ==0){
                        // the back one is which leg
                        inherit->duty = {1.0-inherit->swing_time, 0.5- inherit->swing_time, 0.5, 0.0};
                    }
                    else{
                        // the back one is index leg
                        inherit->duty = {0.5-inherit->swing_time, 1- inherit->swing_time, 0.0, 0.5};
                    }

                    transform_hybrid->update_nextFrame();
                    inherit->body = inherit->next_body;
                    inherit->hip = inherit->next_hip;
                    inherit->foothold = inherit->next_foothold;
                    inherit->step_length = 0.2;
                    inherit->incre_duty = inherit->dS / inherit->step_length;
                    body_move_dist = 0.1;
                    delta_time_step = int(body_move_dist/inherit->dS);
                    std::cout<< "delta_time_step" << delta_time_step << std::endl;
                    step_count = 0;
                    stage++;
                    std::cout << std::endl << "= = = = = Stage 5 Finished = = = = =" << std::endl;
                    break;
                case 6: // hybrid mode
                    for (int i=0; i<4; i++) {
                        auto stance_eta = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {inherit->dS, 0},0);
                        inherit->next_eta[i][0] = stance_eta[0];
                        inherit->next_eta[i][1] = adjust_beta(stance_eta[1], inherit->next_eta[i][1]);
                    }
                    if (step_count == delta_time_step){  
                        step_count = 0;
                        stage++;
                        std::cout << std::endl << "= = = = = Stage 6 Finished = = = = =" << std::endl;
                    }
                    break;
                // case 7: // end
                //     // transform_hybrid->Step();
                //     break;
                default:
                    transform_finished = true;
                    break;
            }
            step_count++;
        }
                    
        
};

extern "C" IGaitTransform* createWheeledToHybrid(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr) {
    return new WheeledToHybrid(hybrid_ptr, legged_ptr);
}

