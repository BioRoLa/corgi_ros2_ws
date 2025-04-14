// WheeledToHybrid.cpp
#include <iostream>
#include "transform_gen.hpp"

class WheeledToHybrid : public IGaitTransform, public Hybrid 
{
    public:
        WheeledToHybrid(ros::NodeHandle& nh) : Hybrid(nh) , wheel_delta_beta(0.0), swing_dir(0){}
        void transform(double shift, int wait_step, bool transfer_state, double expect_height) override {
            Receive();         
            stand_height = expect_height;
            // Rotate in Wheel Mode until RF/LF_beta = 45 deg
            wheel_delta_beta = velocity/(leg_model.radius * pub_rate);
            check_beta[0] = closer_beta(-45*PI/180, 0);
            check_beta[1] = closer_beta( 45*PI/180, 1);  
            
            if(check_beta[0]<= check_beta[1]){
                state_index = false;
            }
            else{
                state_index = true;
            }
            body_move_dist = (leg_model.radius * check_beta[state_index]);
            delta_time_step = int(check_beta[state_index]/wheel_delta_beta);
            for (int i =0;i<delta_time_step;i++){
                for (int j=0;j<4;j++){
                    eta[j][0] = 17 * PI/180;
                    eta[j][1] = eta[j][1] +  wheel_delta_beta;
                }
                Send(1);
            }

            // Front Transform
            body_angle = asin((stand_height - leg_model.radius) / BL);
            if(state_index ==0){
                check_beta[state_index] = closer_beta(body_angle, state_index);
            }
            else{
                check_beta[state_index] = closer_beta(-body_angle, state_index);
            }        
            
            temp = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, -body_angle);
            target_theta = temp[0];

            if(state_index ==0){
                check_beta[state_index] = closer_beta(temp[1], state_index);
            }
            else{
                check_beta[state_index] = closer_beta(-temp[1], state_index);
            }    

            body_move_dist = (leg_model.radius * check_beta[state_index]);

            delta_time_step = int(check_beta[state_index] /wheel_delta_beta);
            target_theta = (target_theta - eta[state_index][0])/delta_time_step;  
 
            temp = find_pose(stand_height, shift, (step_length/2), -body_angle); 

            if (state_index ==0){
                check_beta[!state_index] = closer_beta(-temp[1], !state_index);
            }
            else{
                check_beta[!state_index] = closer_beta(temp[1], !state_index);
            }       

            delta_beta = (check_beta[!state_index]-wheel_delta_beta*delta_time_step*1/3)/(delta_time_step*2/3);
            delta_theta = (temp[0]-eta[!state_index][0])/(delta_time_step*2/3);            

            for (int i =0;i<delta_time_step;i++){
                for (int j=0;j<4;j++){
                    if (j == state_index){
                        eta[j][0] += target_theta;
                        eta[j][1] = eta[j][1] +  wheel_delta_beta;
                    }
                    else if(j == !state_index && i>=(delta_time_step*1/3)){
                        eta[j][0] += delta_theta;
                        eta[j][1] = eta[j][1] +  delta_beta;
                    }  
                    else if(j == !state_index){
                        eta[j][0] = 17 * PI/180;
                        eta[j][1] = eta[j][1] +  wheel_delta_beta;
                    }
                    else{
                        eta[j][0] = 17 * PI/180;
                        eta[j][1] = eta[j][1] +  wheel_delta_beta;
                    }
                }
                Send(1);
            }
            
            // Hybrid mode 
            // find the closest to the initial pose
            if (state_index ==0){
                duty = {0.5, 0, (0.5-swing_time)*2/3, 0.5+(0.5-swing_time)*2/3};   
            }
            else{
                duty = {0, 0.5 ,0.5+(0.5-swing_time)*2/3, (0.5-swing_time)*2/3};
            }

            swing_phase_temp = {0,0,0,0};
            state_index = false;
            Get_index(false);
            Get_index(true);
            while(!state_index){
                // std::cout << "still false" << std::endl;
                for (int i=0; i<4; i++) {
                    duty[i] += incre_duty;     
                }        
                for(int j =2; j<4; j++){
                    if (duty[j] < 0.0){ duty[j] += 1.0; }
                    if ((duty[j] >= (1 - swing_time)) && swing_phase_temp[j] == 0) {
                        swing_phase_temp[j] = 1;
                    } 
                    else if ((duty[j] > 1.0)) {                  
                        swing_phase_temp[j] = 0;
                        duty[j] -= 1.0; 
                    }

                    eta[j][0] = 17 * PI/180;
                    eta[j][1] = abs(eta[j][1]) +  wheel_delta_beta;                
                }
                for (int i=0; i<2; i++) {
                    if (duty[i] < 0){ duty[i] += 1.0; }
                    if ((duty[i] >= (1 - swing_time)) && swing_phase_temp[i] == 0) {
                        swing_phase_temp[i] = 1;
                        swing_pose = find_pose(stand_height, shift, (step_length*3/6), -body_angle);  
                        Swing(eta, swing_pose, swing_variation, i);
                    } 
                    else if ((duty[i] > 1.0)) {                  
                        swing_phase_temp[i] = 0;
                        duty[i] -= 1.0; 
                        
                    }

                    if (swing_phase_temp[i] == 0) { 
                        leg_model.forward(eta[i][0], eta[i][1],true);
                        std::array<double, 2> result_eta;
                        result_eta = leg_model.move(eta[i][0], eta[i][1], {-dS, 0}, -body_angle);
                        eta[i][0] = result_eta[0];
                        eta[i][1] = result_eta[1];
                    } 
                    // read the next Swing phase traj
                    else { 
                        Swing_step(swing_pose, swing_variation, i, duty[i]);
                    }
                }
                // add when the hear leg with extend
                // 1. front legs both not in swing
                // 2. see who reach the ideal one and the other one swing to the position 
                if(swing_phase_temp[0] == 0 && swing_phase_temp[1] == 0 ){
                    if(duty[2]>=duty[3]){
                        // ideal hear pose
                        auto tmp0 = find_pose(stand_height, shift, (step_length/2), 0);
                        next_eta[2][0] = tmp0[0];
                        next_eta[2][1] = tmp0[1];
                        auto tmp1 = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, 0);
                        next_eta[3][0] = tmp1[0];
                        next_eta[3][1] = tmp1[1];
                    }
                    else{
                        auto tmp0 = find_pose(stand_height, shift, (step_length/2), 0);
                        next_eta[3][0] = tmp0[0];
                        next_eta[3][1] = tmp0[1];
                        auto tmp1 = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, 0);
                        next_eta[2][0] = tmp1[0];
                        next_eta[2][1] = tmp1[1];
                    }                
                    check_beta[2] = closer_beta(next_eta[2][1], 2);
                    // std::cout << "check_beta[2] " << check_beta[2] * 180 / M_PI  << ", check_beta[3] " << check_beta[3] * 180 / M_PI  << std::endl;
                    check_beta[3] = closer_beta(-next_eta[3][1], 3);
                    // std::cout << "check_beta[3] " << check_beta[3] * 180 / M_PI  << std::endl;
                    Get_index(false);
                    // std::cout << "test state_index:  "<< other_index << std::endl;
                    // std::cout << "check_beta(or): " << check_beta[other_index+2] * 180 / M_PI << std::endl;
                    Get_index(true);
                    // std::cout << "check_beta(!): " << check_beta[other_index+2] * 180 / M_PI << std::endl;

                    // std::cout << "test state_index:  "<< state_index << std::endl;

                    for (int i = 0; i<2;i++){
                        if(duty[0]>=duty[1]){
                            if ((leg_model.radius *check_beta[i+2]) < (dS*(0.8-duty[0])/incre_duty)){
                                state_index = true;
                            }
                        }
                        else{
                            if ((leg_model.radius *check_beta[i+2]) < (dS*(0.8-duty[1])/incre_duty)){
                                state_index = true;
                            }
                        }
                    }
                    
                }          
                Send(1);
            }
            // std::cout << "----------------------------------------- " << std::endl;
            // find the one that is going to change -> decide the #step
            // the other one swing to the ideal position
            // the front leg walk as original
            // calculate ( 30 the same side -)
            if(check_beta[2]<= check_beta[3]){
                state_index = false;
            }
            else{
                state_index = true;
            }
            Get_index(false);
            // the hear closer leg [state_index+2]
            body_move_dist = (leg_model.radius * check_beta[other_index+2]);
            // std::cout << "check_beta: " << check_beta[2] * 180 / M_PI << ", " 
                            //   << check_beta[3] * 180 / M_PI << std::endl;
            delta_time_step = int(check_beta[(other_index+2)]/wheel_delta_beta);
            // std::cout << "delta_time_step:  "<< delta_time_step << std::endl;
            target_theta = (next_eta[(other_index+2)][0] - eta[(other_index+2)][0])/delta_time_step;
            // the hear further leg  [!state_index+2]
            // 最終要到達的位置
            Get_index(true);
            if(check_beta[(other_index+2)] >= 2*PI - check_beta[(other_index+2)]){
                swing_dir = 0;
            }
            else{
                swing_dir = 1;
            }

           
            if (swing_dir){
                delta_beta = (check_beta[(other_index+2)] -wheel_delta_beta*delta_time_step*1/3)/(delta_time_step*2/3);
                delta_theta = (next_eta[(other_index+2)][0]-eta[(other_index+2)][0])/(delta_time_step*1/3);
            }
            else{
                delta_beta = (2*PI - check_beta[(other_index+2)] -wheel_delta_beta*delta_time_step*1/3)/(delta_time_step*2/3);
                delta_theta = (next_eta[(other_index+2)][0]-eta[(other_index+2)][0])/(delta_time_step*1/3);
            }          
            // std::cout << "delta_time_step:  "<< delta_time_step << std::endl;
            Get_index(false);
            for (int j =0;j<delta_time_step;j++){
                // std::cout << "duty: "<< duty[0] << " , " << duty[1] << " , " << duty[2] << " , " << duty[3] << std::endl; 
                for (int i=0; i<4; i++) {
                    duty[i] += incre_duty;     
                }
                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (duty[i] < 0){ duty[i] += 1.0; }
                    /* Calculate next foothold if entering swing phase(1) */
                    // Enter SW (calculate swing phase traj)
                    if ((duty[i] >= (1 - swing_time)) && swing_phase_temp[i] == 0) {
                        swing_phase_temp[i] = 1;                    
                    } 
                    // Enter TD
                    else if ((duty[i] > 1.0)) {                  
                        swing_phase_temp[i] = 0;
                        duty[i] -= 1.0; 
                    }
                    // calculate the nest Stance phase traj
                    // front leg  slope -> 0
                    if (i == 0 || i == 1){
                        leg_model.forward(eta[i][0], eta[i][1],true);
                        std::array<double, 2> result_eta;
                        result_eta = leg_model.move(eta[i][0], eta[i][1], {-dS, 0}, (-body_angle+body_angle*(j/delta_time_step)));
                        eta[i][0] = result_eta[0];
                        eta[i][1] = result_eta[1];
                    }
                    // hear leg 
                    else if(i == (other_index+2)){
                        // closer one
                        eta[i][0] += target_theta;
                        eta[i][1] = eta[i][1] +  wheel_delta_beta;
                        // std::cout<<  i <<": "<<eta[i][1] * 180 / M_PI << std::endl; 
                    }
                    else{
                        // eta[i][0] = eta[i][0];
                        // eta[i][1] = eta[i][1] +  delta_beta;
                        if (swing_dir){
                            if(j>=(delta_time_step*1/3) && j<=(delta_time_step*2/3)){
                                eta[i][0] = 17 * PI/180;
                                eta[i][1] = eta[i][1] +  delta_beta;
                            }
                            else if(j>(delta_time_step*2/3)){
                                eta[i][0] += delta_theta;
                                eta[i][1] = eta[i][1] +  delta_beta;
                            }
                            else{
                                eta[i][0] = 17 * PI/180;
                                eta[i][1] = eta[i][1] + wheel_delta_beta;
                            }
                        }
                        else{
                            if(j>=(delta_time_step*1/3) && j<=(delta_time_step*2/3)){
                                eta[i][0] = 17 * PI/180;
                                eta[i][1] = eta[i][1] -  delta_beta;
                            }
                            else if(j>(delta_time_step*2/3)){
                                eta[i][0] += delta_theta;
                                eta[i][1] = eta[i][1] -  delta_beta;
                            }
                            else{
                                eta[i][0] = 17 * PI/180;
                                eta[i][1] = eta[i][1] - wheel_delta_beta;
                            }
                        }                        
                    }
                
                }
                Send(1);
            }
            
            
            // co-variable cout to check
            // duty, swing_phase, next_hip, hip
            // keep walk until all leg are on the ground 
            

            // // cout to check
            // std::cout << "duty: "<< duty[0] << " , " << duty[1] << " , " << duty[2] << " , " << duty[3] << std::endl;
            // std::cout << "swing_phase: "<< swing_phase_temp[0] << " , " << swing_phase_temp[1] << " , " << swing_phase_temp[2] << " , " << swing_phase_temp[3] << std::endl;
            // std::cout << "eta" << std::endl;
            // for (int i = 0; i < 4; i++){
            //     std::cout << i << ": " 
            //               << eta[i][0] * 180 / M_PI << ", " 
            //               << eta[i][1] * 180 / M_PI << std::endl;
            // }
            // std::cout << "next_hip" << std::endl;
            // for (int i = 0; i < 4; i++){
            //     std::cout << i << ": " 
            //               << next_hip[i][0]  << ", " 
            //               << next_hip[i][1]  << std::endl;
            // }
            // std::cout << "hip" << std::endl;
            // for (int i = 0; i < 4; i++){
            //     std::cout << i << ": " 
            //               << hip[i][0]  << ", " 
            //               << hip[i][1]  << std::endl;
            // }

            // // if swing phase - > swing to ground
            // // else keep walk
        
            for (int i = 0;i<4 ;i++){
                swing_phase[i] =  swing_phase_temp[i];
                if (duty[i]>=(1-swing_time) ){
                    // std::cout << "tune leg: " << i<< std::endl;
                    swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
                    Swing(eta, swing_pose, swing_variation, i);
                }
            }
            next_hip = hip;
            // std::cout << "duty: "<< duty[0] << " , " << duty[1] << " , " << duty[2] << " , " << duty[3] << std::endl;
            // std::cout << "swing_phase: "<< swing_phase[0] << " , " << swing_phase[1] << " , " << swing_phase[2] << " , " << swing_phase[3] << std::endl;
            

            // std::cout << "----------------------------------------- " << std::endl;
            // for (int i=0;i<1000 ;i++){
            //     Step(1,1,-0.03);
            //     std::cout << "duty: "<< duty[0] << " , " << duty[1] << " , " << duty[2] << " , " << duty[3] << std::endl;
            //     std::cout << "swing_phase: "<< swing_phase[0] << " , " << swing_phase[1] << " , " << swing_phase[2] << " , " << swing_phase[3] << std::endl;
            //     std::cout << "eta" << std::endl;
            //     for (int i = 0; i < 4; i++){
            //         std::cout << i << ": " 
            //                 << eta[i][0] * 180 / M_PI << ", " 
            //                 << eta[i][1] * 180 / M_PI << std::endl;
            //     }

            // }
        }
    private:
        double wheel_delta_beta;
        double check_beta[4];
        bool state_index;
        int other_index;
        double body_move_dist;
        int delta_time_step;

        double body_angle ;
        std::array<double, 2> temp;
        double target_theta;
        double delta_beta;
        double delta_theta;

        // std::array<double, 4> duty;
        std::array<int, 4> swing_phase_temp = {0, 0, 0, 0}; 
        bool swing_dir;


        void Get_index(bool inve){
            if (inve){
                other_index = (state_index == true)? 0 : 1;
            }
            else{
                other_index = (state_index == true)? 1 : 0;
            }
            // std::cout << "other_index:  "<< other_index << std::endl;
            
        }

        double closer_beta(double ref_rad, int leg_index)
        {
            double val0 = motor_state_modules[leg_index]->beta;
            // std::cout << leg_index << ",beta = " << val0 * 180 / PI << std::endl;
            double delta_beta;
            // eta[leg_index][1];
            if (leg_index ==1 | leg_index ==2){
                while (val0 < ref_rad){
                    ref_rad -= 2*PI;
                }
                delta_beta = val0 - ref_rad ;
            }
            else{
                while (val0 > ref_rad){
                    ref_rad += 2*PI;
                }
                delta_beta = ref_rad - val0;
            }
            return delta_beta;
        }

        
};

extern "C" IGaitTransform* createWheeledToHybrid() {
    ros::NodeHandle nh;
    return new WheeledToHybrid(nh);
}









