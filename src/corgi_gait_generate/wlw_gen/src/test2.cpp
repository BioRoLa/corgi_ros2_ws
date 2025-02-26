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
}

void WLWGait::setCmd(std::array<double, 2> send, int index, bool dir) {
    if (dir==true){
        motor_cmd_modules[index]->beta  = -send[1];
    }
    else{
        motor_cmd_modules[index]->beta  = send[1];
    }
    motor_cmd_modules[index]->theta = send[0];
    motor_cmd_modules[index]->kp_r = 150;
    motor_cmd_modules[index]->ki_r = 0;
    motor_cmd_modules[index]->kd_r = 1.75;
    motor_cmd_modules[index]->kp_l = 150;
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
        // std::cout << i << ": " <<current_eta[i][0]*180.0/M_PI << ", "<< current_eta[i][1]*180.0/M_PI << std::endl;
        std::array<double, 2> tmp = { current_eta[i][0], current_eta[i][1] };
        if (i==1 || i==2) {
            setCmd(tmp, i, true);
        } else {
            setCmd(tmp, i, false);
        }     
    }
    publish(freq);
}

void WLWGait::Initialize(int swing_index, int pub_time, int do_pub, int transfer_state, int transfer_sec, int wait_sec) {
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
                current_eta[k][0] = 0.0; 
                current_eta[k][1] = 0.0;
            }
            break;
    }
    
    for(int i =0; i<4;i++){
        if (i!=swing_index){
            auto tmp0 = find_pose(stand_height, 0.00, (step_length/2) - (duty[i]/(1-swing_time)) * step_length);
            next_eta[i][0] = tmp0[0];
            next_eta[i][1] = tmp0[1];
        }
        else{
            auto tmp0 = find_pose(stand_height, 0.00, -(step_length/2));
            next_eta[i][0] = tmp0[0];
            next_eta[i][1] = tmp0[1];
        }
    }
    
    // Get foothold in world coordinate
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;

    // Initial leg configuration
    for (int i=0; i<4; i++) {
        leg_model.contact_map(next_eta[i][0], next_eta[i][1],0);
        leg_model.forward(next_eta[i][0], next_eta[i][1],true);
        relative_foothold[i][0] = leg_model.contact_p[0];
        relative_foothold[i][1] = -stand_height;
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};

    }  

    if (transfer_state){
        Transfer(transfer_sec, wait_sec, do_pub);
    }
    else{
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                current_eta[i][j] = next_eta[i][j];
            }
        }
        if(do_pub){
            Send(pub_time);
        }
    }
}

void WLWGait::Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
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
    double currentX = target[0] + variation[0] * ratio;
    double currentY = target[1] + variation[1] * ratio;

    current_eta[swing_leg][0] = currentX;
    current_eta[swing_leg][1] = currentY;
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
            Swing(current_eta, swing_pose, swing_variation, i);
            
        } 
        // Enter TD
        else if ((duty[i] > 1.0)) {                  
            swing_phase[i] = 0;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
        }

        /* Calculate next eta */
        // calculate the nest Stance phase traj
        if (swing_phase[i] == 0) { 
            leg_model.forward(current_eta[i][0], current_eta[i][1],true);
            std::array<double, 2> result_eta;
            result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, 0);
            current_eta[i][0] = result_eta[0];
            current_eta[i][1] = result_eta[1];
        } 
        // read the next Swing phase traj
        else { 
            Swing_step(swing_pose, swing_variation, current_eta, i, duty[i]);
        }
        // update the hip position
        hip[i] = next_hip[i];
    }
    
    // Send eta 
    if(do_pub){
        Send(pub_time);
    }

}

void WLWGait::Transform(int type, int do_pub, int transfer_state, int transfer_sec, int wait_sec){    
    // tranform according to type
    switch (type)
    {
    case 0:
        // wheel to wlw
        // assumption (wheel mode random)
        std::mt19937 gen(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, 359); 
        for(int i = 0; i < 4; i++){
            int randomDeg = dist(gen); 
            next_eta[i][0] = 17.0 * PI / 180.0;
            next_eta[i][1] = randomDeg * PI / 180.0;
            // std::cout << i << " = " << next_eta[i][0] * 180 / PI << " , " << next_eta[i][1]* 180 / PI  << std::endl;
        }

        // transfer to random wheel pose
        if (transfer_state){
            Transfer(transfer_sec, wait_sec, do_pub);
        }
        else{
            if(do_pub){
                Send(100);
            }
        } 
        break;
    
    default:
        break;
    }    
}

std::vector<double> WLWGait::linspace(double start, double end, int num_steps) {
    std::vector<double> result;
    if (num_steps < 1) return result; 
    
    result.resize(num_steps);
    if (num_steps == 1) {
        // Only one step -> just start
        result[0] = start;
        return result;
    }
    
    double step = (end - start) / (num_steps - 1);
    for (int i = 0; i < num_steps; ++i) {
        result[i] = start + step * i;
    }
    return result;
}

void WLWGait::Transfer(int transfer_sec, int wait_sec, int do_pub){
    // next_eta = target and grep current_motor_pose -> devided to step until current_eta
    // transfer
    std::vector<std::vector<double>> theta_steps(4), beta_steps(4);
    for (int i=0; i<4; i++){
        theta_steps[i] = linspace(motor_state_modules[i]->theta, next_eta[i][0], transfer_sec*pub_rate);
        beta_steps[i]  = linspace(motor_state_modules[i]->beta,  next_eta[i][1], transfer_sec*pub_rate);
    }
    for (int step_i = 0; step_i < transfer_sec*pub_rate; step_i++) {
        for  (int i=0; i<4; i++){
            current_eta[i][0] = theta_steps[i][step_i];
            current_eta[i][1] = beta_steps[i][step_i];
        }       
        // cout << step_i << ": " <<current_eta[1][0]*180.0/M_PI << ", "<< current_eta[1][1]*180.0/M_PI << std::endl;
        if (do_pub) {
            Send(1);
        }
    }

    // wait
    for (int step_i = 0; step_i < wait_sec*pub_rate; step_i++) {
        if (do_pub) {
            Send(1);
        }
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
    // transform
    wlw_gait.Transform(0, 1, 1, 3, 0);
    // wlw initial pose
    wlw_gait.Initialize(2, 300, 0, 0, 5, 2);
    
    

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
// multi-loop //
// variaty
// add transfer(like csv) //
// transform
// turn


