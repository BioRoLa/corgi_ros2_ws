#include "hybrid_gen.hpp"

const double PI = M_PI;


Hybrid::Hybrid(std::shared_ptr<GaitSelector> gait_selector_ptr)
    : gaitSelector(gait_selector_ptr) {
}

std::array<double, 2> Hybrid::find_pose(double height, float shift, float steplength, double duty, double slope){
    std::array<double, 2> pose;
    double pos[2] = {0, -height + gaitSelector->leg_model.r};
    pose = gaitSelector->leg_model.inverse(pos, "G");
    for (double i = 0; i < shift + steplength*(1-gaitSelector->swing_time)*0.5; i += 0.001) {
        // for (double i = 0; i < shift + steplength*(1-gaitSelector->swing_time*gaitSelector->step_length)*0.5; i += 0.001) {
        pose = gaitSelector->leg_model.move(pose[0], pose[1], {-0.001, 0},0); 
    }
    for (double t = 0.0; t <= duty; t += gaitSelector->incre_duty) {
        pose = gaitSelector->leg_model.move(pose[0], pose[1], {gaitSelector->dS, 0},slope);
        
    }
    return pose;
}
 
void Hybrid::Initialize(int swing_index, int set_type) {
    // set_type = set original pose or not
    if(set_type){
        // 1>3>0>2, swing_index = who swings first
        switch (swing_index) {
            case 0:
            {
                gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
                break;
            }
            case 1:
            {
                gaitSelector->duty = {0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.0, 0.5};
                break;
            }
            case 2:
            {
                gaitSelector->duty = {0.5 - 2 * gaitSelector->swing_time, 1 - 2 * gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time};
                break;
            }
            case 3:
            {
                gaitSelector->duty = {1 - 2 * gaitSelector->swing_time, 0.5 - 2 * gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time};
                break;
            }
            default:
                for (int k=0 ; k<4 ; k++){
                    gaitSelector->eta[k][0] = 0.0; 
                    gaitSelector->eta[k][1] = 0.0;
                }
                break;
        }
                
        for(int i =0; i<4;i++){
            // std::cout << "gaitSelector->duty[" << i << "] = " << gaitSelector->duty[i] << std::endl;
            auto tmp0 = find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], (gaitSelector->step_length) , gaitSelector->duty[i],0);
            // double height, float shift, float steplength, double duty, double slope
            gaitSelector->next_eta[i][0] = tmp0[0];
            gaitSelector->next_eta[i][1] = tmp0[1];
        }

    }
    else{
        std::cout << "Read current pose and try set a gaitSelector->duty from guess the leg" << std::endl;
        // read current pose
        // tune all the gaitSelector->eta[i][1] between -pi ~ PI
        for (int i=0; i<4; i++){
            if (gaitSelector->eta[i][1] > PI){
                gaitSelector->eta[i][1] -= 2*PI;
            }
            else if (gaitSelector->eta[i][1] < -PI){
                gaitSelector->eta[i][1] += 2*PI;
            }
        }
        // set gaitSelector->duty by checking beta -> choose the biggest and set swing_index to it
        int swing_index = 0;
        double max_beta = gaitSelector->eta[0][1];
        for (int i=1; i<4; i++){
            if (gaitSelector->eta[i][1] > max_beta){
                max_beta = gaitSelector->eta[i][1];
                swing_index = i;
            }
        }   
        // std::cout << "--------------------------" << std::endl;
        std::cout << "Swing leg index selected: " << swing_index << std::endl;
        // 1>3>0>2, swing_index = who swings first
        switch (swing_index) {
            case 0:
            {
                gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
                break;
            }
            case 1:
            {
                gaitSelector->duty = {0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.0, 0.5};
                break;
            }
            case 2:
            {
                gaitSelector->duty = {0.5 - 2 * gaitSelector->swing_time, 1 - 2 * gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time};
                break;
            }
            case 3:
            {
                gaitSelector->duty = {1 - 2 * gaitSelector->swing_time, 0.5 - 2 * gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time};
                break;
            }
            default:
                for (int k=0 ; k<4 ; k++){
                    gaitSelector->eta[k][0] = 0.0; 
                    gaitSelector->eta[k][1] = 0.0;
                }
                break;
        }
        
        gaitSelector->swing_phase = {0};  
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                gaitSelector->next_eta[i][j] = gaitSelector->eta[i][j];
            }
        }   
    }    

    
    
}

void Hybrid::Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
    std::cout << "Swing leg: " << swing_leg << std::endl;
    std::cout << "Swing target: " << target[0] << ", " << target[1] << std::endl;
    double startX = relative[swing_leg][0];
    double startY = relative[swing_leg][1];
    double endX   = target[0];
    double endY   = target[1];

    // 不一定都需要
    while(endY > startY) {
        endY -= 2.0 * PI;
    } 

    gaitSelector->leg_model.forward(endX, endY, false);
    gaitSelector->leg_model.contact_map(endX, endY);
    int rim_id = gaitSelector->leg_model.rim;
    double alpha0 = gaitSelector->leg_model.alpha;
    Eigen::Vector2d body_velocity = {gaitSelector->velocity,0};
    Eigen::Vector2d terrain = {cos(terrain_slope), sin(terrain_slope)};
    // Eigen::Vector2d terrain = {1,0};
    // 儲存一條完整 swing 軌跡
    swing_traj[swing_leg] = HybridSwing::generate(gaitSelector->leg_model, swing_type, startX, endX, startY, endY, rim_id, alpha0, body_velocity, terrain, 500);

    variation[0] = endX - startX;
    variation[1] = endY - startY;
    target[0] = startX;
    target[1] = startY;
}

void Hybrid::Swing_step(std::array<double, 2> target, std::array<double, 2> variation, int swing_leg, double duty_ratio){
    double ratio = (duty_ratio - (1-gaitSelector->swing_time)) / gaitSelector->swing_time;
    ratio = clamp(ratio, 0.0, 1.0); 

    int idx = static_cast<int>(ratio * 500);
    if (idx >= 500) idx = 500;

    gaitSelector->next_eta[swing_leg][0] = swing_traj[swing_leg][idx].theta;
    gaitSelector->next_eta[swing_leg][1] = swing_traj[swing_leg][idx].beta;
}

void Hybrid::Step(){
    for (int i=0; i<4; i++) {
        gaitSelector->next_hip[i][0] += gaitSelector->dS;
        gaitSelector->duty[i] += gaitSelector->incre_duty;    
    }

    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (gaitSelector->duty[i] < 0){ 
            gaitSelector->duty[i] += 1.0; 
        }

        /* Calculate next foothold if entering swing phase*/
        // Enter SW (calculate swing phase traj)
        if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
            gaitSelector->swing_phase[i] = 1;
            double total_step_length; 
            double swing_hip_move_d; 
            // change to new step length when front leg start to swing
            // front leg swing
            if ( (i==0 || i==1))  
            {  
                // apply new step length and differential
                gaitSelector->next_step_length[i] = gaitSelector->new_step_length;   
                double rest_time = (1.0 - 4*gaitSelector->swing_time) / 2;
                total_step_length = gaitSelector->step_length;
                swing_hip_move_d = gaitSelector->direction * gaitSelector->swing_time * total_step_length;
    
            }
            // hind leg swing
            else {    
                int last_leg = (i+2) % 4;   // Contralateral front leg 對側
                // apply hind step length corresponding to the front leg's.
                gaitSelector->step_length = gaitSelector->current_step_length[last_leg];
                gaitSelector->next_step_length[i] = gaitSelector->step_length;   
                total_step_length = gaitSelector->step_length ;
                swing_hip_move_d = gaitSelector->direction * gaitSelector->swing_time * total_step_length; 
                gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            }

            swing_pose = find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
            // swing_pose[1] += 0.0872664626;
            // swing_pose[1] += 0.1745329252;
            Swing(gaitSelector->eta, swing_pose, swing_variation, i);            
        } 
        // Enter TD
        else if ((gaitSelector->duty[i] > 1.0)) {                  
            gaitSelector->swing_phase[i] = 0;
            gaitSelector->duty[i] -= 1.0; 
            gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
        }
        /* Calculate next gaitSelector->eta */
        // calculate the nest Stance phase traj
        if (gaitSelector->swing_phase[i] == 0) { 
            gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
            std::array<double, 2> result_eta;
            result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), (gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2])}, 0);
            gaitSelector->next_eta[i][0] = result_eta[0];
            gaitSelector->next_eta[i][1] = result_eta[1];
        } 
        // read the next Swing phase traj
        else { 
            Swing_step(swing_pose, swing_variation, i, gaitSelector->duty[i]);
        }
        // update the gaitSelector->hip position
        gaitSelector->hip[i] = gaitSelector->next_hip[i];
    }

}

void Hybrid::change_Height(double new_value, int leg_index){
    // 0-3 specific leg, 4 all legs
    // real-change when in the stance phase
    // gaitSelector->new_step_length = new_value;
    if (leg_index == 4){
        for (int i=0; i<4; i++) {
            // gaitSelector->new_stand_height[leg_index] = new_value;
            gaitSelector->current_stand_height[leg_index] = new_value;
        }
    }
    else{
        // gaitSelector->new_stand_height[leg_index] = new_value;
        gaitSelector->current_stand_height[leg_index] = new_value;
    }
    // add limitation
}

void Hybrid::change_Height_all(double new_value){
    gaitSelector->stand_height  = new_value;
    for (int i=0; i<4; i++) {
        gaitSelector->next_hip[i][2] = gaitSelector->stand_height;
        gaitSelector->current_stand_height[i] = gaitSelector->stand_height;
    }
    // add limitation
}

void Hybrid::change_Step_length(double new_value){
    gaitSelector->new_step_length = new_value;
    // add limitation
}

void Hybrid::change_Velocity(double new_value){
    gaitSelector->velocity = new_value;
    gaitSelector->dS = gaitSelector->velocity / gaitSelector->pub_rate;
    gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;  
}

void Hybrid::update_nextFrame(){
    // use next eta 
    for (int i=0; i<4; i++) {
        if(i==1 || i==2) {
            gaitSelector->leg_model.contact_map(gaitSelector->next_eta[i][0], -gaitSelector->next_eta[i][1],0);
            gaitSelector->leg_model.forward(gaitSelector->next_eta[i][0], -gaitSelector->next_eta[i][1],true);
        }
        else{
            gaitSelector->leg_model.contact_map(gaitSelector->next_eta[i][0], gaitSelector->next_eta[i][1],0);
            gaitSelector->leg_model.forward(gaitSelector->next_eta[i][0], gaitSelector->next_eta[i][1],true);
        }
        
        // calculate gaitSelector->relative_foothold
        // in leg frame
        gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.contact_p[0];
        gaitSelector->relative_foothold[i][1] = gaitSelector->leg_model.contact_p[1];
        // std::cout << "relative_foothold: " << gaitSelector->relative_foothold[i][0] << ", " << gaitSelector->relative_foothold[i][1] << std::endl;
    }
    computeNextBody();

    // in robot frame
    gaitSelector->next_hip = {{{gaitSelector->next_body[0]+gaitSelector->BL/2, gaitSelector->next_body[1]+gaitSelector->BW/2, gaitSelector->next_body[2]},
                               {gaitSelector->next_body[0]+gaitSelector->BL/2, gaitSelector->next_body[1]-gaitSelector->BW/2, gaitSelector->next_body[2]},
                               {gaitSelector->next_body[0]-gaitSelector->BL/2, gaitSelector->next_body[1]-gaitSelector->BW/2, gaitSelector->next_body[2]},
                               {gaitSelector->next_body[0]-gaitSelector->BL/2, gaitSelector->next_body[1]+gaitSelector->BW/2, gaitSelector->next_body[2]}}};

    gaitSelector->next_foothold = {{{gaitSelector->next_hip[0][0] - gaitSelector->relative_foothold[0][1], gaitSelector->next_hip[0][1], gaitSelector->next_hip[0][2] + gaitSelector->relative_foothold[0][0]},
                                    {gaitSelector->next_hip[1][0] - gaitSelector->relative_foothold[1][1], gaitSelector->next_hip[1][1], gaitSelector->next_hip[1][2] + gaitSelector->relative_foothold[1][0]},
                                    {gaitSelector->next_hip[2][0] - gaitSelector->relative_foothold[2][1], gaitSelector->next_hip[2][1], gaitSelector->next_hip[2][2] + gaitSelector->relative_foothold[2][0]},
                                    {gaitSelector->next_hip[3][0] - gaitSelector->relative_foothold[3][1], gaitSelector->next_hip[3][1], gaitSelector->next_hip[3][2] + gaitSelector->relative_foothold[3][0]}}};
}

void Hybrid::computeNextBody() {
    gaitSelector->next_body[0] = gaitSelector->body[0] + gaitSelector->dS;  // move forward
    gaitSelector->next_body[1] = gaitSelector->body[1];       // lateral stays
    double sum_z = 0.0;

    for (int i = 0; i < 4; ++i) {
        double z_i = gaitSelector->relative_foothold[i][1];
        sum_z += -z_i;  // because z_i = hip_z - foot_z → foot_z = hip_z + z_i → hip_z = foot_z - z_i → body_z = foot_z - z_i
    }

    gaitSelector->next_body[2] = sum_z / 4.0;
    // std::cout << "height: " << gaitSelector->next_body[2] << std::endl;
}

double Hybrid::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}
void Hybrid::csv_title(std::ofstream &file) {
    // open the file and keep saving / in no file then open the new one to save
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }
    // write the header
    file << "step, "
         << "eta[0][0], eta[0][1], "
         << "eta[1][0], eta[1][1], "
         << "eta[2][0], eta[2][1], "
         << "eta[3][0], eta[3][1], "
         << "next_eta[0][0], next_eta[0][1], "
         << "next_eta[1][0], next_eta[1][1], "
         << "next_eta[2][0], next_eta[2][1], "
         << "next_eta[3][0], next_eta[3][1], "
         << "relative_foothold.x, relative_foothold.y, "
         << "body.x, body.y, body.z, "
         << "next_body.x, next_body.y, next_body.z, "
         << "hip.x, hip.y, hip.z, "
         << "next_hip.x, next_hip.y, next_hip.z, "
         << "swing_phase[0], swing_phase[1], swing_phase[2], swing_phase[3], "
         << "duty[0], duty[1], duty[2], duty[3]" << std::endl;

}

// save the step, eta, next_eta, relative_foothold, body, next_body, hip, next_hip, swing_phase, duty of each leg to csv "totall_steps.csv"
void Hybrid::save_to_csv(std::ofstream &file, int step) {
    // open the file and keep saving / in no file then open the new one to save
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }
    // // write the header
    // file << "step, "
    //      << "eta[0][0], eta[0][1], "
    //      << "eta[1][0], eta[1][1], "
    //      << "eta[2][0], eta[2][1], "
    //      << "eta[3][0], eta[3][1], "
    //      << "next_eta[0][0], next_eta[0][1], "
    //      << "next_eta[1][0], next_eta[1][1], "
    //      << "next_eta[2][0], next_eta[2][1], "
    //      << "next_eta[3][0], next_eta[3][1], "
    //      << "relative_foothold.x, relative_foothold.y, "
    //      << "body.x, body.y, body.z, "
    //      << "next_body.x, next_body.y, next_body.z, "
    //      << "hip.x, hip.y, hip.z, "
    //      << "next_hip.x, next_hip.y, next_hip.z, "
    //      << "swing_phase[0], swing_phase[1], swing_phase[2], swing_phase[3], "
    //      << "duty[0], duty[1], duty[2], duty[3]" << std::endl;
    // write the data
    file << step << ", ";
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->eta[i][0] << ", " << gaitSelector->eta[i][1] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->next_eta[i][0] << ", " << gaitSelector->next_eta[i][1] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->relative_foothold[i][0] << ", " << gaitSelector->relative_foothold[i][1] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->body[i] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->next_body[i] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->hip[i][0] << ", " << gaitSelector->hip[i][1] << ", " << gaitSelector->hip[i][2] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->next_hip[i][0] << ", " << gaitSelector->next_hip[i][1] << ", " << gaitSelector->next_hip[i][2] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->swing_phase[i] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->duty[i] << ", ";
    }   
    file << std::endl;
    // close the file
    
    // std::cout << "Data saved to file" << std::endl;
    std::cout << "step: " << step << std::endl;
}


void Hybrid::Step_wheel(){
    for (int i=0; i<4; i++) {
        gaitSelector->next_hip[i][0] += gaitSelector->dS;
        gaitSelector->duty[i] += gaitSelector->incre_duty;    
    }
    for (int i=0; i<2; i++) {
        double wheel_deltabeta = gaitSelector->dS/ gaitSelector->leg_model.radius; // wheel delta beta
        // for front wheel
        /* Keep duty in the range [0, 1] */
        if (gaitSelector->duty[i] < 0){ 
            gaitSelector->duty[i] += 1.0; 
        }
        // Enter TD
        if ((gaitSelector->duty[i] > 1.0)) {   
            gaitSelector->duty[i] -= 1.0; 
        }

        gaitSelector->next_eta[i][0] = 17*PI/180.0; // 17 degree
        gaitSelector->next_eta[i][1] -= wheel_deltabeta; // beta = beta - delta_beta
    }

    for (int i=2; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (gaitSelector->duty[i] < 0){ 
            gaitSelector->duty[i] += 1.0; 
        }

        if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
            gaitSelector->swing_phase[i] = 1;
            gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            std::cout << "lo_pose2: " << gaitSelector->next_eta[i][0] << ", " << gaitSelector->next_eta[i][1] << std::endl;
    
            
            swing_pose = find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0.1466077); 
            std::cout << "swing_pose: " << swing_pose[0] << ", " << swing_pose[1] << std::endl;
            swing_pose[1] += 0.1466077;
            
            Swing(gaitSelector->eta, swing_pose, swing_variation, i);            
        } 
        // Enter TD
        else if ((gaitSelector->duty[i] > 1.0)) {                  
            gaitSelector->swing_phase[i] = 0;
            gaitSelector->duty[i] -= 1.0; 
            gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
        }
        /* Calculate next gaitSelector->eta */
        // calculate the nest Stance phase traj
        if (gaitSelector->swing_phase[i] == 0) { 
            gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
            std::array<double, 2> result_eta;
            result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0.1466077);
            gaitSelector->next_eta[i][0] = result_eta[0];
            gaitSelector->next_eta[i][1] = result_eta[1];
        } 
        // read the next Swing phase traj
        else { 
            Swing_step(swing_pose, swing_variation, i, gaitSelector->duty[i]);
        }
        // update the gaitSelector->hip position
        gaitSelector->hip[i] = gaitSelector->next_hip[i];
    }

}