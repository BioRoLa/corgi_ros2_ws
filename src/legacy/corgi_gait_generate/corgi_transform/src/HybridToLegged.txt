// HybridToLegged.cpp
#include <iostream>
#include "transform_gen.hpp"

class HybridToLegged : public IGaitTransform, public Hybrid
{
public:
    HybridToLegged(ros::NodeHandle& nh) : Hybrid(nh) {}
    void transform(double shift, int wait_step, bool transfer_state, double expect_height) override {       
        /* Strategy */
        // read current duty and swing_phase
        // if get in swing phase then turn into walk pose
        check_point=0;
        walk_transform = {0, 0, 0, 0};
        change_Step_length(0.2);
        while(check_point<4){
            // keep walk until transform finish
            for (int i=0; i<4; i++) {
                next_hip[i][0] += dS ;
                duty[i] += incre_duty;     
            }

            for (int i=0; i<4; i++) {
                /* Keep duty in the range [0, 1] */
                if (duty[i] < 0){ duty[i] += 1.0; }

                /* Calculate next foothold if entering swing phase(1) */
                // Enter SW (calculate swing phase traj)
                if ((duty[i] >= (1 - swing_time)) && swing_phase[i] == 0) {
                    swing_phase[i] = 1;
                    swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
                    // tune step length for leg mode
                    temp_step_length = step_length;
                    
                    double pos[2] = {-step_length/2*(1-swing_time), -stand_height + leg_model.r};
                    swing_pose_temp = leg_model.inverse(pos, "G");
                    if(walk_transform[i] ==0){
                        if (walk_transform[i] ==0){
                            walk_transform[i] = 1;
                            // std::cout<< i << " get in hybrid swing phase" << std::endl;
                        }
                
                    }
                    Swing(eta, swing_pose, swing_variation, i);
                    Swing(eta, swing_pose_temp, swing_variation_temp, i);
                    
                } 
                // Enter TD
                else if ((duty[i] > 1.0)) {           
                    swing_phase[i] = 0;
                    duty[i] -= 1.0; // Keep duty in the range [0, 1]
                    if(walk_transform[i]==1){
                        walk_transform[i]=2;
                        check_point++;
                        // std::cout<< i << " start walk" << std::endl;
                        // std::cout<< "check_point add " << i << " ,current= " << check_point<< std::endl;
                        
                    }
                    else if(walk_transform[i]!=2){
                        // std::cout<< i << " still in wlw" << std::endl;  
                    }
                }

                /* Calculate next eta */
                // calculate the nest Stance phase traj
                if (swing_phase[i] == 0) { 
                        leg_model.forward(eta[i][0], eta[i][1],true);
                        std::array<double, 2> result_eta;
                        result_eta = leg_model.move(eta[i][0], eta[i][1], {-dS, 0}, 0);
                        eta[i][0] = result_eta[0];
                        eta[i][1] = result_eta[1];                 
                } 
                // read the next Swing phase traj
                else { 
                    if (walk_transform[i]!=0){
                        // to walk
                        Swing_step(swing_pose_temp, swing_variation_temp, i, duty[i]);
                    }
                    else{
                        // origin tranform
                        Swing_step(swing_pose, swing_variation, i, duty[i]);
                    }
                    
                }
            }
            Send(1);
        }   
        step_length = temp_step_length;
    }
private:
    int check_point;
    std::array<int, 4> walk_transform;
    double temp_step_length;
    std::array<double, 2> swing_pose_temp;
    std::array<double, 2> swing_variation_temp;
};

extern "C" IGaitTransform* createHybridToLegged() {
    ros::NodeHandle nh;
    return new HybridToLegged(nh);
}





