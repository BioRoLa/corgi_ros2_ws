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
            std::cout << "Transforming from Wheeled to Hybrid Gait..." << std::endl;
            // state 0 (paramerters)
            for (int i =0;i<10;i++){
                for (int j=0;j<4;j++){
                    inherit->next_eta[j][0] = 17 * PI/180;
                    if (j==0){
                        inherit->next_eta[j][1] = 50 * PI/180;
                    }
                    else if (j==1 ){
                        inherit->next_eta[j][1] = 50 * PI/180;
                    }
                    else{
                        inherit->next_eta[j][1] = 0;
                    }
                    
                }
                inherit->Send();
            }
            for (int i=0; i<4; i++) {
                if (i==0 || i==3) {
                    inherit->eta[i][0] =  inherit->motor_cmd_modules[i]->theta;
                    inherit->eta[i][1] = -inherit->motor_cmd_modules[i]->beta;
                } else {
                    inherit->eta[i][0] =  inherit->motor_cmd_modules[i]->theta;
                    inherit->eta[i][1] =  inherit->motor_cmd_modules[i]->beta;
                }
                inherit->next_eta[i][0] = inherit->eta[i][0];
                inherit->next_eta[i][1] = inherit->eta[i][1];
            }
            expect_height = 0.16;
            wheel_delta_beta = inherit->velocity/(inherit->leg_model.radius * inherit->pub_rate);

            // rotate until the closest front leg beta ~= 45 deg
            check_beta[0] = closer_beta( transform_angle, 0);
            check_beta[1] = closer_beta( transform_angle, 1);  
            if(check_beta[0]<= check_beta[1]){ which_leg = 0;}
            else{which_leg = 1;}

            body_move_dist = (inherit->leg_model.radius * check_beta[which_leg]);
            delta_time_step = int(check_beta[which_leg]/wheel_delta_beta);
            for (int i =0;i<delta_time_step;i++){
                for (int j=0;j<4;j++){
                    inherit->next_eta[j][0] = 17 * PI/180;
                    inherit->next_eta[j][1] = inherit->eta[j][1] -  wheel_delta_beta;
                }
                inherit->Send();
            };            

            // front transform 
            body_angle = asin((expect_height - inherit->leg_model.radius) / inherit->BL);
            std::cout << "Body Angle = " << body_angle * 180 / PI << std::endl;
            auto temp = transform_hybrid->find_pose(expect_height*cos(body_angle), 0, inherit->step_length/2, 0.4 , 0);
            std::cout << "Target Theta(which_leg) = " << temp[0] *180/PI <<" , " << temp[1] *180/PI  << std::endl;
            target_theta[which_leg] = temp[0];
            target_beta[which_leg] = temp[1];
            check_beta[which_leg] = closer_beta(target_beta[which_leg], which_leg);
            

            body_move_dist = abs(inherit->leg_model.radius * check_beta[which_leg]);
            delta_time_step = int(check_beta[which_leg]/wheel_delta_beta);

            delta_beta[which_leg] = check_beta[which_leg]/delta_time_step;
            delta_theta[which_leg] = (target_theta[which_leg] - inherit->eta[which_leg][0]) / delta_time_step;
            int index = (which_leg == 0) ? 1 : 0; 
            
            // determine the step length from hybrid mode
            auto hybrid_steps = find_hybrid_steps(inherit->next_eta[2][1]-wheel_delta_beta*delta_time_step,
                                                  inherit->next_eta[3][1]-wheel_delta_beta*delta_time_step, which_leg);
            StepStrategyResult result;
            result.step_num = (int)hybrid_steps[0];
            result.step_length = hybrid_steps[1];
            result.last_leg  = (int)hybrid_steps[2];
            int temp_leg =  (result.last_leg == 3) ? 2 : 3; 
            
            auto pose = transform_hybrid->find_pose(expect_height*cos(body_angle), 0, inherit->step_length/2, 0.0 , 0);
            target_theta[index]= pose[0];
            target_beta[index] = pose[1];
            swing_eta = {target_theta[index], target_beta[index]};
            check_beta[index] = closer_beta(target_beta[index], index);
            delta_beta[index] = check_beta[index]/delta_time_step;
            delta_theta[index] = (target_theta[index] - inherit->eta[index][0]) / delta_time_step; 

            // move
            int step_count = 0;
            for (int i =0;i<delta_time_step;i++){
                if (i < delta_time_step/3.0) { 
                    inherit->next_eta[index][1] -= wheel_delta_beta; 
                }
                else if (i < delta_time_step*2/3.0) { 
                    inherit->next_eta[index][1] -= (delta_beta[index]*3 - wheel_delta_beta); 
                }
                else { 
                    inherit->next_eta[index][0] += delta_theta[index] * 3; 

                }
                    
                inherit->next_eta[which_leg][0] += delta_theta[which_leg];
                inherit->next_eta[which_leg][1] -= delta_beta[which_leg];
                
                inherit->next_eta[2][1] -= wheel_delta_beta;
                inherit->next_eta[3][1] -= wheel_delta_beta;

                inherit->Send();
                
            };
            
            // (result.step_length*result.step_num)/inherit->leg_model.radius
            inherit->step_length = result.step_length;
            delta_time_step_each = (int)((result.step_length/inherit->leg_model.radius)/wheel_delta_beta);
            //每移step的次數
            delta_time_step = delta_time_step_each * result.step_num;
            // all次數

            std::cout << "Step Number = " << result.step_num << std::endl;
            std::cout << "Step Length = " << result.step_length << std::endl;
            // 若偶數步 → which_leg同側 擺動(which_leg)
            // 若奇數步 → which_leg異側 擺動
            
            auto swing_end = transform_hybrid->find_pose(expect_height*cos(body_angle), 0, inherit->step_length/2, 0.4 , 0);
            // swing the further leg and the closer one keep move
            swing_eta[1] = adjust_beta(swing_eta[1], swing_end[1]);
            std::cout << "swing_eta = " << swing_eta[0]  << " , " << swing_eta[1]  << std::endl;
            std::cout << "which_leg = " << inherit->eta[which_leg][0]  << " , " << inherit->eta[which_leg][1]  << std::endl;
            std::vector<SwingPoint> traj_tr;
            for (int i = 0; i <= 100; ++i) {
                double t = static_cast<double>(i) / 100;
                double theta = 0.0, beta = 0.0;

                double mid_theta = 17.0 * M_PI / 180.0;
                if (t < 0.5) {
                    theta = swing_end[0] + (mid_theta - swing_end[0]) * (2 * t);
                } else {
                    theta = mid_theta + (swing_eta[0] - mid_theta) * (2 * (t - 0.5));
                }
                beta  = swing_end[1]  + (swing_eta[1] - swing_end[1] ) * (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t);
        

                traj_tr.push_back({t, theta, beta});
            }

            std::cout << "Trajectory size: " << traj_tr.size() << std::endl;
            step_count = 0;
            for(step_count;step_count<delta_time_step;step_count++){
                int swing_index = step_count % delta_time_step_each;
                double phase_ratio = static_cast<double>(swing_index) / delta_time_step_each;

                int idx = static_cast<int>(phase_ratio * 100);
                idx = std::min(idx, 99);
                
                if ((step_count / delta_time_step_each) % 2 == 0){
                    // std::cout <<"A" << std::endl;Swing_step(swing_eta, variation, int swing_leg, double duty_ratio)
                    inherit->next_eta[which_leg][0] = traj_tr[idx].theta;
                    inherit->next_eta[which_leg][1] = adjust_beta(traj_tr[idx].beta, inherit->eta[which_leg][1]);
        
                    auto stance_eta = inherit->leg_model.move(inherit->next_eta[index][0], inherit->next_eta[index][1], {result.step_length/(double)delta_time_step_each, 0});
                    inherit->next_eta[index][0] = stance_eta[0];
                    inherit->next_eta[index][1] = adjust_beta(stance_eta[1], inherit->next_eta[index][1]);
                }
                else{
                    // std::cout <<"B" << std::endl;
                    inherit->next_eta[index][0] = traj_tr[idx].theta;
                    inherit->next_eta[index][1] = adjust_beta(traj_tr[idx].beta, inherit->eta[which_leg][1]);
        
                    auto stance_eta = inherit->leg_model.move(inherit->next_eta[which_leg][0], inherit->next_eta[which_leg][1], {result.step_length/(double)delta_time_step_each, 0});
                    inherit->next_eta[which_leg][0] = stance_eta[0];
                    inherit->next_eta[which_leg][1] = adjust_beta(stance_eta[1], inherit->next_eta[which_leg][1]);
                
                }
                inherit->next_eta[2][1] -= wheel_delta_beta;
                inherit->next_eta[3][1] -= wheel_delta_beta;
                inherit->Send();
            }

            // 若偶數步 → which_leg同側 擺動(which_leg)
            // 若奇數步 → which_leg異側 擺動
            // std::cout << "temp_leg: " << temp_leg << std::endl;
            double transform_start_beta, transform_start_theta,transform_target_beta,transform_target_theta,last_start_beta, last_start_theta;
            transform_start_theta = inherit->next_eta[temp_leg][0];
            transform_start_beta = inherit->next_eta[temp_leg][1];    
            // std::cout << "Transform Start Theta(temp_leg) = " << transform_start_theta *180/PI <<" , " << transform_start_beta *180/PI  << std::endl;  
            // std::cout << "Transform Start Theta(temp_leg) = " << inherit->next_eta[temp_leg][0] *180/PI <<" , " << inherit->next_eta[temp_leg][1] *180/PI  << std::endl;        

            temp = transform_hybrid->find_pose(expect_height*cos(body_angle), 0, inherit->step_length/2, 0.2 , 0);
            // std::cout << "Target Theta(temp_leg) = " << temp[0] *180/PI <<" , " << temp[1] *180/PI  << std::endl;
            transform_target_theta = temp[0];
            transform_target_beta = temp[1];
            transform_target_beta = adjust_beta(transform_target_beta,transform_start_beta);
            // std::cout << "Target Theta(temp_leg) = " << transform_target_theta *180/PI  <<" , " << transform_target_beta *180/PI   << std::endl;

            body_move_dist = abs(inherit->leg_model.radius * (transform_target_beta-transform_start_beta));
            // std::cout << "Body Move Distance = " << body_move_dist << std::endl;
            delta_time_step = body_move_dist*inherit->pub_rate/inherit->velocity;
            // std::cout << "Delta Time Step = " << delta_time_step << std::endl;

            double transform_delta_beta = abs(transform_target_beta-transform_start_beta)/delta_time_step;
            double transform_delta_theta = (transform_target_theta-transform_start_theta)/delta_time_step;
            
            last_start_beta = inherit->next_eta[result.last_leg][1];
            last_start_theta = inherit->next_eta[result.last_leg][0];
            pose = transform_hybrid->find_pose(expect_height*cos(body_angle), 0.1, inherit->step_length/2, 0.0 , 0);
            std::cout << "Target Theta(lat leg) = " << pose[0] *180/PI <<" , " << pose[1] *180/PI  << std::endl;
            double last_target_theta = pose[0];
            double last_target_beta = pose[1];
            last_target_beta = adjust_beta(last_target_beta, last_start_beta);
            std::cout << "last_target_beta = " << last_target_beta *180/PI  << std::endl;

            double last_delta_beta = abs(last_target_beta-last_start_beta)/(delta_time_step);
            double last_delta_theta = (last_target_theta-last_start_theta)/delta_time_step;

            // hind transform
            std::cout << "Transforming to Hybrid Gait..." << std::endl;
            step_count = 0;
            for(step_count;step_count<delta_time_step;step_count++){
                for (int i=0; i<2; i++){
                    auto stance_eta = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {result.step_length/(double)delta_time_step_each, 0},0);
                    inherit->next_eta[i][0] = stance_eta[0];
                    inherit->next_eta[i][1] = adjust_beta(stance_eta[1], inherit->next_eta[i][1]);
                }

                inherit->next_eta[temp_leg][0] += transform_delta_theta;
                inherit->next_eta[temp_leg][1] -= transform_delta_beta;
                if (step_count < delta_time_step/3.0){
                    inherit->next_eta[result.last_leg][1] -= wheel_delta_beta;
                }
                else if (step_count < delta_time_step*2/3.0){
                    inherit->next_eta[result.last_leg][1] -= (last_delta_beta*3 - wheel_delta_beta);
                    // inherit->next_eta[result.last_leg][0] += last_delta_theta * 3/2.0;
                }
                else {
                    inherit->next_eta[result.last_leg][0] += last_delta_theta * 3;

                }

                inherit->Send();
            }
            std::cout << "Result Theta(temp_leg) = " << inherit->next_eta[temp_leg][0] *180/PI <<" , " << inherit->next_eta[temp_leg][1] *180/PI  << std::endl;
            std::cout << "Result Theta(lat leg) = " << inherit->next_eta[result.last_leg][0] *180/PI <<" , " << inherit->next_eta[result.last_leg][1] *180/PI  << std::endl;
            
       
        // adjust to a better position (find the hear one and swing to the front)
        // int adjust;
        // if(result.step_num % 2 == 0) {
        //     if(which_leg == 1){
        //         adjust=1;
        //     }
        //     else{adjust =0;}
        // }
        // else{
        //     if(which_leg == 1){
        //         adjust=0;
        //     }
        //     else{adjust = 1;}
        // }
        // std::cout << "adjust"<< adjust << std::endl;
        // swing_end = transform_hybrid->find_pose(expect_height*cos(body_angle), 0.05, inherit->step_length/2, 0.0 , 0);
        // // swing the further leg and the closer one keep move
        // swing_end[1] = adjust_beta(swing_end[1], inherit->next_eta[adjust][1]);
        // traj_tr.clear();
        // for (int i = 0; i <= 100; ++i) {
        //     double t = static_cast<double>(i) / 100;
        //     double theta = 0.0, beta = 0.0;

        //     double mid_theta = 17.0 * M_PI / 180.0;
        //     if (t < 0.5) {
        //         theta = inherit->next_eta[adjust][0] + (mid_theta - inherit->next_eta[adjust][0]) * (2 * t);
        //     } else {
        //         theta = mid_theta + (swing_end[0] - mid_theta) * (2 * (t - 0.5));
        //     }
        //     beta  = inherit->next_eta[adjust][1]  + (swing_end[1] - inherit->next_eta[adjust][1] ) * (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t);
        //     traj_tr.push_back({t, theta, beta});
        // }
        // std::cout << "traj_tr"<< traj_tr.size() << std::endl;
        // // cout the 10 traj
        // for (int i = 0; i < 10 && i < traj_tr.size(); ++i) {
        //     std::cout << "Time: " << traj_tr[i].time << ", Theta: " << traj_tr[i].theta * 180 / PI 
        //               << ", Beta: " << traj_tr[i].beta * 180 / PI << std::endl;
        // }
        // delta_time_step  = inherit->step_length*inherit->swing_time*inherit->pub_rate/inherit->velocity;
        // step_count =0;
        // double dis = (inherit->step_length*(1-inherit->swing_time))/delta_time_step;
        // std::cout << "dis = " << dis << std::endl;
        // for(step_count;step_count<delta_time_step;step_count++){
        //     // std::cout << "step_count = " << step_count << std::endl;
        //     int swing_index = step_count % delta_time_step_each;
        //     double phase_ratio = static_cast<double>(swing_index) / delta_time_step_each;

        //     int idx = static_cast<int>(phase_ratio * 100);
        //     idx = std::min(idx, 99);
        //     // std::cout << "idx = " << idx << std::endl;
        //     for (int i=0; i<4; i++){
        //         // std::cout << "i = " << i << std::endl;
        //         if(i == adjust){
        //             inherit->next_eta[adjust][0] = traj_tr[idx].theta;
        //             inherit->next_eta[adjust][1] = adjust_beta(traj_tr[idx].beta, inherit->eta[adjust][1]);

        //         }
        //         else{
        //             auto stance_eta = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {dis, 0},0);
        //             inherit->next_eta[i][0] = stance_eta[0];
        //             inherit->next_eta[i][1] = stance_eta[1];
        //         }
                
        //     }
        //     inherit->Send();
        // }

         
            
    }
    private:
        std::shared_ptr<Hybrid> transform_hybrid;
        double expect_height;
        double wheel_delta_beta;
        double check_beta[2];
        int which_leg;
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

        int delta_time_step_each;    
        std::array<double, 2> variation;
        std::array<double, 2> swing_eta;
        std::array<double, 3> find_hybrid_steps(double RH_beta, double LH_beta, int which) {
            std::cout << "RH_beta" <<  RH_beta << " , LH_beta = " << LH_beta << std::endl;
            std::cout << "which leg = " << which << std::endl;
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
                            std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 3;  // LH swing
                            std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
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
                            std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 2;  // RH swing
                            std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
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
                            std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 2;  // RH swing
                            std::cout << "LH_target_beta: " <<  LH_target_beta <<std::endl;
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
                            std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                        if (step_length >= min_step_length && step_length < max_step_length) {
                            leg_index = 3;  // LH swing
                            std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
                            return {step_num, step_length, static_cast<double>(leg_index)};
                        }
                    }
                  
                }

                step_num += 1;
            }

            // 無合適結果
            std::cout << "RH_target_beta: " <<  RH_target_beta <<std::endl;
            return {0.0, 0.0, 0.0};
        }

        
};

extern "C" IGaitTransform* createWheeledToHybrid(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr) {
    return new WheeledToHybrid(hybrid_ptr, legged_ptr);
}

