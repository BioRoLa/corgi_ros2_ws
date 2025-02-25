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
    motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", 1000);
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &WLWGait::motorsStateCallback, this);
    rate_ptr = new ros::Rate(pub_rate);
    
    // Initialize dS & incre_duty
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;
}

WLWGait::~WLWGait() {
    delete rate_ptr;
    rate_ptr = nullptr;
}

void WLWGait::motorsStateCallback(const corgi_msgs::MotorStateStamped::ConstPtr& msg)
{
    current_motor_state_ = *msg;
    // std::cout << "Received motor state" << std::endl;
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
    // cout<<"origin: "<<pose[0]*180/PI<<","<<pose[2]*180/PI<<endl;
    if (steplength >= 0) {
        for (double i = 0; i < shift + steplength; i += 0.001) {
            // cout<<"pose: "<<pose[0]*180/PI<<","<<pose[2]*180/PI<<endl;
            pose = leg_model.move(pose[0], pose[1], {0.001, 0}, 0);
        }
    } else {
        for (double i = 0; i > shift + steplength; i -= 0.001) {
            pose = leg_model.move(pose[0], pose[1], {-0.001, 0}, 0);
        }
    }
    // cout<<"end: "<<pose[0]*180/PI<<","<<pose[2]*180/PI<<endl;
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

void WLWGait::Initialize(int swing_index, int pub_time, int do_pub) {
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
            eta[0][0] = 0.0; eta[0][1] = 0.0;
            eta[1][0] = 0.0; eta[1][1] = 0.0;
            eta[2][0] = 0.0; eta[2][1] = 0.0;
            eta[3][0] = 0.0; eta[3][1] = 0.0;
            break;
    }
    
    for(int i =0; i<4;i++){
        if (i!=swing_index){
            auto tmp0 = find_pose(stand_height, 0.00, (step_length/2) - (duty[i]/(1-swing_time)) * step_length);
            eta[i][0] = tmp0[0];
            eta[i][1] = tmp0[1];
        }
        else{
            auto tmp0 = find_pose(stand_height, 0.00, -(step_length/2));
            eta[i][0] = tmp0[0];
            eta[i][1] = tmp0[1];
        }
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
    // for (int i =0; i<4; i++){
    //     cout<< "Pose" << endl;
    //     cout<< "Leg: "<<i<< ",theta: " << relative[i][0]*180/PI << ", beta: " << relative[i][1]*180/PI << endl;
    // }
    // cout<< " " << endl;

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

void WLWGait::Swing_step(std::array<double, 2> target, std::array<double, 2> variation, double eta[4][2], int swing_leg, double duty_ratio){
    double ratio = (duty_ratio-(1-swing_time))/swing_time;
    // cout<< "duty_ratio" << duty_ratio << ", ration in swing phase = " << ratio << endl;
    double currentX = target[0] + variation[0] * ratio;
    double currentY = target[1] + variation[1] * ratio;

    eta[swing_leg][0] = currentX;
    eta[swing_leg][1] = currentY;
}

void WLWGait::Step(int pub_time, int do_pub){
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
            swing_pose = find_pose(stand_height, 0.00, (step_length*3/6));  
            Swing(eta, swing_pose, swing_variation, i);
            
        } 
        // Enter TD
        else if ((duty[i] > 1.0)) {                  
            swing_phase[i] = 0;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
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
            Swing_step(swing_pose, swing_variation, eta, i, duty[i]);
        }
        // update the hip position
        hip[i] = next_hip[i];
    }
    
    // Send eta 
    if(do_pub){
        Send(pub_time);
    }

}

void WLWGait::Transform(int type, int do_pub){
    // assumption (wheel mode)
    std::random_device rd;  
    std::mt19937 gen(rd()); 
    std::uniform_int_distribution<int> dist(0, 359); // 0~359(含)
    for(int i = 0; i < 4; i++){
        int randomDeg = dist(gen); 
        eta[i][0] = 17.0 * PI / 180.0;
        eta[i][1] = randomDeg * PI / 180.0;
        
        std::cout << i << " = " << eta[i][0] * 180 / PI << " , " << eta[i][1]* 180 / PI  << std::endl;
    }
    if (do_pub){
        Send(100);
    }
    
    switch (type)
    {
    case 0:
        // wheel to wlw

        break;
    
    default:
        break;
    }    
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "wlw_test");
    ros::NodeHandle nh;

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double CoM_bias = 0.0;
    int pub_rate = 1000;
    
    /*  wlw functions  */
    // initialize
    WLWGait wlw_gait(nh, true, CoM_bias, pub_rate);    
    wlw_gait.Initialize(2, 300, 0);
    
    // transform
    wlw_gait.Transform(0,1);
    // real-time wlw
    // // the loop decide the distance
    // for (int step = 0;step<5000;step++) {
    //     wlw_gait.Step(1,1);
    // }
    ros::shutdown();
    return 0;
}

// stability
// collision
// slope
// print (draw)
// how to swing
// multi-loop
// variaty
// add reading initial pose (find the leg that should be the front)
// add transfer(like csv)
// transform
// turn


