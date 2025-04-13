#include "hybrid_gen.hpp"

const double PI = M_PI;
Hybrid::Hybrid(ros::NodeHandle& nh) : GaitSelector(nh)
{
    
}

std::array<double, 2> Hybrid::find_pose(double height, float shift, float steplength, double slope){
    std::array<double, 2> pose;
    double pos[2] = {0, -height + leg_model.r};
    pose = leg_model.inverse(pos, "G");
    if (steplength >= 0) {
        for (double i = 0; i < shift + steplength; i += 0.001) {
            pose = leg_model.move(pose[0], pose[1], {0.001, 0}, slope);
        }
    } else {
        for (double i = 0; i > shift + steplength; i -= 0.001) {
            pose = leg_model.move(pose[0], pose[1], {-0.001, 0}, slope);
        }
    }
    return pose;
}
 
// add read current 硬走
void Hybrid::Initialize(int swing_index, int pub_time, int do_pub, int transfer_state, int transfer_sec, int wait_sec, double shift) {
    // 1>3>0>2, swing_index = who swings first
    switch (swing_index) {
        case 0:
        {
            duty = {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
            break;
        }
        case 1:
        {
            duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
            break;
        }
        case 2:
        {
            duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
            break;
        }
        case 3:
        {
            duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
            break;
        }
        default:
            for (int k=0 ; k<4 ; k++){
                eta[k][0] = 0.0; 
                eta[k][1] = 0.0;
            }
            break;
    }
    
    for(int i =0; i<4;i++){
            auto tmp0 = find_pose(stand_height, shift, (step_length/2) - (duty[i]/(1-swing_time)) * step_length, 0);
            next_eta[i][0] = tmp0[0];
            next_eta[i][1] = tmp0[1];
    }

    // Initial leg configuration
    for (int i=0; i<4; i++) {
        leg_model.contact_map(next_eta[i][0], next_eta[i][1],0);
        leg_model.forward(next_eta[i][0], next_eta[i][1],true);
        relative_foothold[i][0] = leg_model.contact_p[0];
        relative_foothold[i][1] = -stand_height;
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};

    }  

    if (transfer_state){
        Transfer(transfer_sec, wait_sec);
    }
    else{
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                eta[i][j] = next_eta[i][j];
            }
        }
        if(do_pub){
            Send(pub_time);
        }
    }
}

void Hybrid::Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
    double startX = relative[swing_leg][0];
    double startY = relative[swing_leg][1];

    double endX   = target[0];
    double endY   = target[1];
    while(endY < startY) {
        endY += 2.0 * PI;
    } 
    variation[0] = endX - startX;
    variation[1] = endY - startY;

    target[0] = startX;
    target[1] = startY;    
}

void Hybrid::Swing_step(std::array<double, 2> target, std::array<double, 2> variation, int swing_leg, double duty_ratio){
    double ratio = (duty_ratio-(1-swing_time))/swing_time;
    double currentX = target[0] + variation[0] * ratio;
    double currentY = target[1] + variation[1] * ratio;

    eta[swing_leg][0] = currentX;
    eta[swing_leg][1] = currentY;
}

void Hybrid::Step(int pub_time, int do_pub, double shift){
    for (int i=0; i<4; i++) {
        next_hip[i][0] += dS ;
        duty[i] += incre_duty;     
    }

    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (duty[i] < 0){ duty[i] += 1.0; }

        /* Calculate next foothold if entering swing phase(1) */
        // Enter SW (calculate swing phase traj)

        if ((duty[i] > (1 - swing_time)) && swing_phase[i] == 0) {
            swing_phase[i] = 1;
            // change to new step length when front leg start to swing
            // front leg swing (forward)
            if ( (i==0 || i==1)) {  
                // apply new step length and differential
                next_step_length[i] = new_step_length;   
                double rest_time = (1.0 - 4*swing_time) / 2;
                foothold[i] = {next_hip[i][0] + ((1-swing_time)/2)*(new_step_length ) + (swing_time)*(step_length ) + (rest_time*(step_length - new_step_length)), 0};   
                // half distance between leave and touch-down position (in hip coordinate) + distance hip traveled during swing phase + hip travel difference during rest time because different incre_duty caused by change of step length.
            } 
            // hind leg swing
            else {    
                int last_leg = (i+2) % 4;   // Contralateral front leg 對側
                // apply hind step length corresponding to the front leg's.
                step_length = current_step_length[last_leg];
                next_step_length[i] = step_length;    
                foothold[i] = {next_hip[i][0] + ((1-swing_time)/2+swing_time)*(step_length), 0};
                incre_duty = dS / step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            }
            swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
            Swing(eta, swing_pose, swing_variation, i);
            
        } 
        // Enter TD
        else if ((duty[i] > 1.0)) {                  
            swing_phase[i] = 0;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
            current_step_length[i] = next_step_length[i];  
        }

        /* Calculate next eta */
        // calculate the nest Stance phase traj
        if (swing_phase[i] == 0) { 
            leg_model.forward(eta[i][0], eta[i][1],true);
            std::array<double, 2> result_eta;
            // result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, 0); 
            result_eta = leg_model.move(eta[i][0], eta[i][1], {-(next_hip[i][0]-hip[i][0]), next_hip[i][1]-hip[i][1]}, 0);
            eta[i][0] = result_eta[0];
            eta[i][1] = result_eta[1];
        } 
        // read the next Swing phase traj
        else { 
            Swing_step(swing_pose, swing_variation, i, duty[i]);
        }
        // update the hip position
        hip[i] = next_hip[i];
    }
    
    // Send eta 
    if(do_pub){
        Send(pub_time);
    }

}

void Hybrid::change_Height(double new_value){
    stand_height = new_value;
    for (int i=0; i<4; i++) {
        next_hip[i][1] = stand_height;
    }
    // add limitation
}

void Hybrid::change_Step_length(double new_value){
    new_step_length = new_value;
    // add limitation
}

void Hybrid::change_Velocity(double new_value){
    velocity = new_value;
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;  
}


// hip foothold