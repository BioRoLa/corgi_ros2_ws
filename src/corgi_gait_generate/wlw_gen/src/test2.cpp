#include "test2.hpp"

using namespace std;
using namespace Eigen;

WLWGait::WLWGait(ros::NodeHandle& nh, bool sim, double CoM_bias, int pub_rate, double BL, double BW, double BH): 
    leg_model(sim), 
    CoM_bias(CoM_bias), 
    BL(BL), 
    BW(BW), 
    BH(BH), 
    pub_rate(pub_rate)
{
    motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", 1);
    rate_ptr = new ros::Rate(pub_rate);

    // Initialize dS & incre_duty
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;
}

WLWGait::~WLWGait() {
    delete rate_ptr;
    rate_ptr = nullptr;
}

void WLWGait::setCmd(std::array<double, 2> send, int index, bool dir) {
    if (dir==true){
        motor_cmd_modules[index]->beta  = -send[1];
    }
    else{
        motor_cmd_modules[index]->beta  = send[1];
    }
    motor_cmd_modules[index]->theta = send[0];
    motor_cmd_modules[index]->kp_r = 90;
    motor_cmd_modules[index]->ki_r = 0;
    motor_cmd_modules[index]->kd_r = 1.75;
    motor_cmd_modules[index]->kp_l = 90;
    motor_cmd_modules[index]->ki_l = 0;
    motor_cmd_modules[index]->kd_l = 1.75;
}

void WLWGait::publish(int freq) {
    for (int i = 0; i < freq; i++) {
        motor_pub.publish(motor_cmd);
        rate_ptr->sleep();
    }
}

std::array<double, 2> WLWGait::find_pose(double height, float shift, float steplength) {
    std::array<double, 2> pose;
    double pos[2] = {0, -height + 0.0125};
    pose = leg_model.inverse(pos, "G");

    if (steplength >= 0) {
        for (double i = 0; i < shift + steplength; i += 0.001) {
            pose = leg_model.move(pose[0], pose[1], {0.001, 0}, 0);
        }
    } else {
        for (double i = 0; i > shift + steplength; i -= 0.001) {
            pose = leg_model.move(pose[0], pose[1], {-0.001, 0}, 0);
        }
    }
    return pose;
}

void WLWGait::Send(int freq){
    for(int i =0; i<4; i++){
        // std::cout << i << ": " <<eta[i][0]*180.0/M_PI << ", "<< eta[i][1]*180.0/M_PI << std::endl;
        std::array<double, 2> tmp = { eta[i][0], eta[i][1] };
        if (i==1 || i==2) {
            setCmd(tmp, i, true);
        } else {
            setCmd(tmp, i, false);
        }     
    }
    publish(freq);
}

void WLWGait::Initialize(int index, int pub_time, int swing_index, int do_pub) {
    // 1>3>0>2, index = who swings first
    switch (index) {
        case 0:
        {
            duty= {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
            // LF: 0(-SL*3/6), 1(SL*1/6), 2(-SL*1/6), 3(SL*3/6)
            auto tmp0 = find_pose(stand_height, 0.00, (-step_length*3/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(stand_height, 0.00, ( step_length*1/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(stand_height, 0.00, (-step_length*1/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(stand_height, 0.00, ( step_length*3/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        case 1:
        {
            duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
            // RF: 0( SL*1/6), 1(-SL*3/6), 2( SL*3/6), 3(-SL*1/6)
            auto tmp0 = find_pose(stand_height, 0.00, ( step_length*1/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(stand_height, 0.00, (-step_length*3/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(stand_height, 0.00, ( step_length*3/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(stand_height, 0.00, (-step_length*1/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        case 2:
        {
            duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
            // RF: 0( SL*3/6), 1(-SL*1/6), 2(-SL*3/6), 3( SL*1/6)
            auto tmp0 = find_pose(stand_height, 0.00, ( step_length*3/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(stand_height, 0.00, (-step_length*1/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(stand_height, 0.00, (-step_length*3/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(stand_height, 0.00, ( step_length*1/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        case 3:
        {
            duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
            // RF: 0(-SL*1/6), 1( SL*3/6), 2( SL*1/6), 3(-SL*3/6)
            auto tmp0 = find_pose(stand_height, 0.00, (-step_length*1/6));
            eta[0][0] = tmp0[0];
            eta[0][1] = tmp0[1];

            auto tmp1 = find_pose(stand_height, 0.00, ( step_length*3/6));
            eta[1][0] = tmp1[0];
            eta[1][1] = tmp1[1];

            auto tmp2 = find_pose(stand_height, 0.00, ( step_length*1/6));
            eta[2][0] = tmp2[0];
            eta[2][1] = tmp2[1];

            auto tmp3 = find_pose(stand_height, 0.00, (-step_length*3/6));
            eta[3][0] = tmp3[0];
            eta[3][1] = tmp3[1];
            break;
        }
        default:
            eta[0][0] = 0.0; eta[0][1] = 0.0;
            eta[1][0] = 0.0; eta[1][1] = 0.0;
            eta[2][0] = 0.0; eta[2][1] = 0.0;
            eta[3][0] = 0.0; eta[3][1] = 0.0;
            break;
    }
    
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    
    // Get foothold in world coordinate
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;

    // Initial leg configuration
    for (int i=0; i<4; i++) {
        leg_model.contact_map(eta[i][0], eta[i][1],0);
        leg_model.forward(eta[i][0], eta[i][1],true);
        relative_foothold[i][0] = leg_model.contact_p[0];
        relative_foothold[i][1] = -stand_height;
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};

    }  
    if(do_pub){
        Send(pub_time);
    }
    
}

void WLWGait::Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
    double startX = relative[swing_leg][0];
    double startY = relative[swing_leg][1];

    double endX   = target[0];
    double endY   = target[1];
    if(endY < startY) {
        endY += 2.0 * M_PI;
    } 

    variation[0] = endX - startX;
    variation[1] = endY - startY;

    target[0] = startX;
    target[1] = startY;
}


void WLWGait::Step(int pub_time, int do_pub){
    for (int i=0; i<4; i++) {
        next_hip[i][0] += dS ;
        duty[i] += incre_duty;
        // cout<< "Leg: "<< i << ", duty: " << duty[i] << ", hip_x: " << next_hip[i][0] << endl;       
    }

    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (duty[i] < 0){ duty[i] += 1.0; }

        /* Calculate next foothold if entering swing phase(1) */
        // Enter SW (calculate swing phase traj)
        if ((duty[i] > (1 - swing_time)) && swing_phase[i] == 0) {
            swing_phase[i] = 1;
            std::array<double, 2> swing_pose;
            std::array<double, 2> swing_variation;
            swing_pose = find_pose(Height, 0.00, (SL*3/6));  
            Swing(relative_eta, swing_pose, swing_variation, swing_index);
            
        } 
        // Enter TD
        else if ((duty[i] > 1.0)) {                  
            swing_phase[i] = 0;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
        }

        /* Calculate next eta */
        // calculate the nest Stance phase traj
        if (swing_phase[i] == 0) { 
            
        } 
        // read the next Swing phase traj
        else { 

        }
        
        // update the hip position
        hip[i] = next_hip[i];
    }
    
    // Send eta 
    if(do_pub){
        Send(pub_time);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wlw_test");
    ros::NodeHandle nh;

    double CoM_bias = 0.0;
    int pub_rate = 1000;

    WLWGait wlw_gait(nh, true, CoM_bias, pub_rate);       
    wlw_gait.Initialize(0, 1000, 0, 1);

    // the loop decide the distance

    for (int step = 0;step<3000;step++) {
        wlw_gait.Step(1,1);
    }
    ros::shutdown();
    return 0;
}


