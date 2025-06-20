#include "legged_gen.hpp"

Legged::Legged(std::shared_ptr<GaitSelector> gait_selector_ptr) 
: gaitSelector(gait_selector_ptr){}
void Legged::Initial() 
{
    double init_theta[4] = {gaitSelector->eta[0][0], gaitSelector->eta[1][0], gaitSelector->eta[2][0], gaitSelector->eta[3][0]};
    double init_beta[4]  = {-gaitSelector->eta[0][1], -gaitSelector->eta[1][1], -gaitSelector->eta[2][1], -gaitSelector->eta[3][1]};
      // Get foothold in hip coordinate from initial configuration
    gaitSelector->relative_foothold[4][2] = {0.0};
    int current_rim = 0;
    // std::cout<< "0" << std::endl;
    for (int i=0; i<4; i++) {
        gaitSelector->leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = gaitSelector->leg_model.rim;
        gaitSelector->leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1) {
            gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.U_l[0];
        } else if (current_rim == 2) {
            gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.L_l[0];
        } else if (current_rim == 3) {
            gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.G[0];
        } else if (current_rim == 4) {
            gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.L_r[0];
        } else if (current_rim == 5) {
            gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.U_r[0];
        } else {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        }//end if else
        gaitSelector->relative_foothold[i][1] = -gaitSelector->stand_height;
    }//end for
    // Get initial leg duty  
    // std::cout<< "1" << std::endl;
    int first_swing_leg = 0;
    for (int i=1; i<4; i++) {
        if (gaitSelector->relative_foothold[i][0] < gaitSelector->relative_foothold[first_swing_leg][0]) {
            first_swing_leg = i;
        }//end if
    }//end for 
    if (first_swing_leg == 0) {
        gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
    } else if (first_swing_leg == 1) {
        gaitSelector->duty = {0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.0, 0.5};
    } else if (first_swing_leg == 2) {
        gaitSelector->duty = {0.5 - 2 * gaitSelector->swing_time, 1 - 2 * gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time};
    } else if (first_swing_leg == 3) {
        gaitSelector->duty = {1 - 2 * gaitSelector->swing_time, 0.5 - 2 * gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time};
    }//end if else
    // Get foothold in world coordinate
    // std::cout<< "2" << std::endl;
    gaitSelector->hip = {{{gaitSelector->BL/2, gaitSelector->stand_height} ,
                        {gaitSelector->BL/2, gaitSelector->stand_height} ,
                        {-gaitSelector->BL/2, gaitSelector->stand_height},
                        {-gaitSelector->BL/2, gaitSelector->stand_height}}};
    gaitSelector->next_hip = gaitSelector->hip;
    // Initial leg configuration
    for (int i=0; i<4; i++) {
        gaitSelector->foothold[i] = {gaitSelector->next_hip[i][0] + gaitSelector->relative_foothold[i][0], gaitSelector->next_hip[i][1] + gaitSelector->relative_foothold[i][1]};
    }
    // Initial theta/beta
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i]  = init_beta[i];
    }
}

void Legged::next_Step() {
    touchdown = false;
    for (int i=0; i<4; i++) {
        gaitSelector->next_hip[i][0] += gaitSelector->dS + gaitSelector->sign_diff[i]*gaitSelector->diff_dS;
        gaitSelector->duty[i] += gaitSelector->incre_duty;
    }
    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (gaitSelector->duty[i] < 0){
            gaitSelector->duty[i] += 1.0;
        }//end if
        /* Calculate next foothold if entering swing phase */
        if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
            gaitSelector->swing_phase[i] = 1;
            double total_step_length; // step length considering differential
            double swing_hip_move_d; // hip moving distance during swing phase 
            // change to new step length when front leg start to swing
            if ( ((gaitSelector->direction == 1) && (i==0 || i==1)) || ((gaitSelector->direction == -1) && (i==2 || i==3)) ) {  // front leg swing
                // apply new step length and differential
                gaitSelector->next_step_length[i] = gaitSelector->new_step_length;   
                double rest_time = (1.0 - 4*gaitSelector->swing_time) / 2;    // time during swing of front leg and next hind leg 
                total_step_length = gaitSelector->step_length + gaitSelector->sign_diff[i]*gaitSelector->diff_step_length;
                swing_hip_move_d = gaitSelector->direction * gaitSelector->swing_time * total_step_length;
                gaitSelector->foothold[i] = {gaitSelector->next_hip[i][0] + gaitSelector->direction*((1-gaitSelector->swing_time)/2)*(gaitSelector->new_step_length + gaitSelector->sign_diff[i]*gaitSelector->new_diff_step_length) + swing_hip_move_d + (rest_time*(gaitSelector->step_length - gaitSelector->new_step_length)) + gaitSelector->CoM_bias, 0};    // half distance between leave and touch-down position (in hip coordinate) + distance hip traveled during swing phase + hip travel difference during rest time because different incre_duty caused by change of step length + CoM_bias.
                gaitSelector->diff_step_length = gaitSelector->new_diff_step_length;
            } else {    // hind leg swing
                int last_leg = (i+2) % 4;   // Contralateral front leg
                gaitSelector->step_length = gaitSelector->current_step_length[last_leg];
                gaitSelector->next_step_length[i] = gaitSelector->step_length;    // apply hind step length corresponding to the front leg's.
                total_step_length = gaitSelector->step_length + gaitSelector->sign_diff[i]*gaitSelector->diff_step_length;
                swing_hip_move_d = gaitSelector->direction * gaitSelector->swing_time * total_step_length;
                gaitSelector->foothold[i] = {gaitSelector->next_hip[i][0] + gaitSelector->direction*((1-gaitSelector->swing_time)/2)*total_step_length + swing_hip_move_d + gaitSelector->CoM_bias, 0};
                gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            }//end if else
            /* Bezier curve setup */
            gaitSelector->leg_model.forward(theta[i], beta[i]);
            p_lo = {gaitSelector->next_hip[i][0] + gaitSelector->leg_model.G[0], gaitSelector->next_hip[i][1] + gaitSelector->leg_model.G[1]};
            // calculate contact rim when touch ground
            for (int j = 0; j < 5; j++) {   // G, L_l, U_l
                double contact_height = j == 0 ? gaitSelector->leg_model.r : gaitSelector->leg_model.radius;
                std::array<double, 2> contact_point = {
                    gaitSelector->foothold[i][0] - (gaitSelector->next_hip[i][0] + swing_hip_move_d),
                    -gaitSelector->stand_height + contact_height
                };
            
                // Convert std::array to a C-style array pointer using .data()
                result_eta = gaitSelector->leg_model.inverse(contact_point.data(), touch_rim_list[j]);
            
                gaitSelector->leg_model.contact_map(result_eta[0], result_eta[1]);
                
                if (gaitSelector->leg_model.rim == touch_rim_idx[j]) {
                    current_rim = gaitSelector->leg_model.rim;
                    break;
                }
            }
            
            // G position when touch ground
            gaitSelector->leg_model.forward(result_eta[0], result_eta[1]);
            if (current_rim == 3) {   // G
                p_td = {gaitSelector->foothold[i][0], gaitSelector->foothold[i][1] + gaitSelector->leg_model.r};
            } else if (current_rim == 2) {  // L_l
                p_td = {gaitSelector->foothold[i][0] + gaitSelector->leg_model.G[0]-gaitSelector->leg_model.L_l[0], gaitSelector->foothold[i][1] + gaitSelector->leg_model.G[1]-gaitSelector->leg_model.L_l[1] + gaitSelector->leg_model.radius};
            } else if (current_rim == 4) {  // L_r
                p_td = {gaitSelector->foothold[i][0] + gaitSelector->leg_model.G[0]-gaitSelector->leg_model.L_r[0], gaitSelector->foothold[i][1] + gaitSelector->leg_model.G[1]-gaitSelector->leg_model.L_r[1] + gaitSelector->leg_model.radius};
            } else if (current_rim == 1) {  // U_l
                p_td = {gaitSelector->foothold[i][0] + gaitSelector->leg_model.G[0]-gaitSelector->leg_model.U_l[0], gaitSelector->foothold[i][1] + gaitSelector->leg_model.G[1]-gaitSelector->leg_model.U_l[1] + gaitSelector->leg_model.radius};
            } else if (current_rim == 5) {  // U_r
                p_td = {gaitSelector->foothold[i][0] + gaitSelector->leg_model.G[0]-gaitSelector->leg_model.U_r[0], gaitSelector->foothold[i][1] + gaitSelector->leg_model.G[1]-gaitSelector->leg_model.U_r[1] + gaitSelector->leg_model.radius};
            }//end if else
            sp[i] = SwingProfile(p_lo, p_td, gaitSelector->step_height, gaitSelector->direction);
        } else if ((gaitSelector->direction == 1) && (gaitSelector->duty[i] > 1.0)) {                  // entering stance phase when velocirty > 0
            touchdown = true;
            gaitSelector->swing_phase[i] = 0;
            gaitSelector->duty[i] -= 1.0; // Keep duty in the range [0, 1]
            if (sp[i].getDirection() == gaitSelector->direction){ // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] += 1;
                gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
            }//end if
        } else if ((gaitSelector->direction == -1) && (gaitSelector->duty[i] < (1.0-gaitSelector->swing_time))) {    // entering stance phase when velocirty < 0
            touchdown = true;
            gaitSelector->swing_phase[i] = 0;
            if (sp[i].getDirection() == gaitSelector->direction){ // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] -= 1;
                gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];   
            }//end if
        }//end if else

        /* Calculate next theta, beta */
        if (gaitSelector->swing_phase[i] == 0) { // Stance phase
            // std::cout << i<<" :calculate eta"  << std::endl;
            result_eta = gaitSelector->leg_model.move(theta[i], beta[i], {gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0], gaitSelector->next_hip[i][1]-gaitSelector->hip[i][1]});
        } else { // Swing phase
            // std::cout << i<<" :calculate swing eta"  << std::endl;
            if ( sp[i].getDirection()==1 ) {    // direction == 1
                swing_phase_ratio = (gaitSelector->duty[i] - (1 - gaitSelector->swing_time)) / gaitSelector->swing_time;
            } else {    // direction == -1
                swing_phase_ratio = (1.0 - gaitSelector->duty[i]) / gaitSelector->swing_time;
            }//end if else
            curve_point_temp = sp[i].getFootendPoint(swing_phase_ratio);
            std::array<double, 2> curve_point = {curve_point_temp[0] - gaitSelector->next_hip[i][0], curve_point_temp[1] - gaitSelector->next_hip[i][1]};
            result_eta = gaitSelector->leg_model.inverse(curve_point.data(), "G");
        }//end if else
        theta[i] = result_eta[0];
        beta[i] = result_eta[1];
        gaitSelector->hip[i] = gaitSelector->next_hip[i];
    }//end for
    for(int i=0; i<4; i++){
        gaitSelector->next_eta[i][0] = theta[i]; 
        gaitSelector->next_eta[i][1] = -beta[i];
    }    
    gaitSelector->Send();
}

void Legged::set_velocity(double new_value){
    if (std::abs(new_value) > 0.5) {
        throw std::runtime_error("Velocity should not exceed 0.5 m/s.");
    }//end if
    gaitSelector->velocity = new_value;
    gaitSelector->dS = gaitSelector->velocity / gaitSelector->pub_rate;
    gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;
    gaitSelector->direction = gaitSelector->velocity>=0? 1 : -1;
    // change differential dS if the robot is turning
    if (gaitSelector->curvature != 0.0) {
        gaitSelector->diff_dS = gaitSelector->dS * (gaitSelector->outer_radius - gaitSelector->inner_radius) / (gaitSelector->outer_radius + gaitSelector->inner_radius);  // apply new value immediately
    }//end if 
}//end set_velocity

void Legged::set_stand_height(double new_value){
    if (new_value > 0.34 || new_value < 0.12+gaitSelector->step_height) {
        throw std::runtime_error("Stand height should be between 0.12+\"step_height\" and 0.34.");
    }//end if
    gaitSelector->stand_height = new_value;
    for (int i=0; i<4; i++) {
        gaitSelector->next_hip[i][1] = gaitSelector->stand_height;
    }//end for
}//end set_stand_height

void Legged::set_step_length(double new_value){
    if (new_value <= 0.0) {
        throw std::runtime_error("Step length should be larger than zero.");
    }//end if
    gaitSelector->new_step_length = new_value;
    // change differential step length if the robot is turning
    if (gaitSelector->curvature != 0.0) {
        gaitSelector->new_diff_step_length = gaitSelector->new_step_length * (gaitSelector->outer_radius - gaitSelector->inner_radius) / (gaitSelector->outer_radius + gaitSelector->inner_radius);    // apply new value when front leg swing
    }//end if 
}//end set_step_length

void Legged::set_step_height(double new_value){
    if (new_value <= 0.0) {
        throw std::runtime_error("Step height should be larger than zero.");
    }//end if
    gaitSelector->step_height = new_value;
}//end set_step_height

void Legged::set_curvature(double new_value){
    gaitSelector->curvature = new_value;    
    if (gaitSelector->curvature == 0.0) {
        gaitSelector->new_diff_step_length = 0.0;
        gaitSelector->diff_dS = 0.0;
    } else {
        double turn_radius = 1.0 / std::abs(gaitSelector->curvature);
        gaitSelector->outer_radius = turn_radius + gaitSelector->BW/2.0;
        gaitSelector->inner_radius = turn_radius - gaitSelector->BW/2.0;
        /*
        step_length + d : step_length - d  = outer_radius : inner_radius
        (step_length - d) * outer_radius = (step_length + d) * inner_radius
        d * (outer_radius + inner_radius) = step_length * (outer_radius - inner_radius)
        */
        gaitSelector->new_diff_step_length = gaitSelector->new_step_length * (gaitSelector->outer_radius - gaitSelector->inner_radius) / (gaitSelector->outer_radius + gaitSelector->inner_radius);    // apply new value when front leg swing
        gaitSelector->diff_dS = gaitSelector->dS * (gaitSelector->outer_radius - gaitSelector->inner_radius) / (gaitSelector->outer_radius + gaitSelector->inner_radius);  // apply new value immediately
        // determine increase/decrease of differential according to sign of curvature and left/right leg 
        if (gaitSelector->curvature > 0.0) { // turn left
            gaitSelector->sign_diff[0] = -1;
            gaitSelector->sign_diff[1] = 1;
            gaitSelector->sign_diff[2] = 1;
            gaitSelector->sign_diff[3] = -1;
        } else { // turn right
            gaitSelector->sign_diff[0] = 1;
            gaitSelector->sign_diff[1] = -1;
            gaitSelector->sign_diff[2] = -1;
            gaitSelector->sign_diff[3] = 1;
        }//end if else
    }// end if else
}//end set_curvature

