#include "Simple_fsm.hpp"

GaitSelector::GaitSelector( ros::NodeHandle& nh, 
                            bool sim, 
                            double CoM_bias, 
                            int pub_rate, 
                            double BL, 
                            double BW, 
                            double BH): 
    leg_model(sim), 
    CoM_bias(CoM_bias), 
    BL(BL), 
    BW(BW), 
    BH(BH), 
    pub_rate(pub_rate),
    rng(rd()), 
    dist(0, 359),
    currentGait(Gait::WHEELED)
{
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &GaitSelector::motor_state_cb, this);
    motor_cmd_pub_ = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", pub_rate);
    rate_ptr = new ros::Rate(pub_rate);

    // Initialize dS & incre_duty
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;  
    
    // Get foothold in world coordinate
    // assume the body center  is in 0,0
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;

    

}

GaitSelector::~GaitSelector() {
    delete rate_ptr;
    rate_ptr = nullptr;
}



void GaitSelector::setCmd(std::array<double, 2> send, int index, bool dir) {
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

void GaitSelector::publish(int freq) {
    for (int i = 0; i < freq; i++) {
        motor_cmd_pub_.publish(motor_cmd);
        rate_ptr->sleep();
    }
}

void GaitSelector::Send(int freq){
    for(int i =0; i<4; i++){
        // std::cout << i << ": " <<current_eta[i][0]*180.0/M_PI << ", "<< current_eta[i][1]*180.0/M_PI << std::endl;
        std::array<double, 2> tmp = { eta[i][0], eta[i][1] };
        if (i==1 || i==2) {
            setCmd(tmp, i, true);
        } else {
            setCmd(tmp, i, false);
        }     
    }
    publish(freq);
}

void GaitSelector::motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}  






void GaitSelector::Transfer(int transfer_sec, int wait_sec){
    // next_eta = target and grep current_motor_pose -> devided to step until current_eta
    // transfer
    std::vector<std::vector<double>> theta_steps(4), beta_steps(4);
    for (int i=0; i<4; i++){
        theta_steps[i] = linspace(motor_state_modules[i]->theta, next_eta[i][0], transfer_sec*pub_rate);
        beta_steps[i]  = linspace(motor_state_modules[i]->beta,  next_eta[i][1], transfer_sec*pub_rate);
    }
    for (int step_i = 0; step_i < transfer_sec*pub_rate; step_i++) {
        for  (int i=0; i<4; i++){
            eta[i][0] = theta_steps[i][step_i];
            eta[i][1] = beta_steps[i][step_i];
        }       
        Send(1);
    }

    // wait
    for (int step_i = 0; step_i < wait_sec*pub_rate; step_i++) {
        Send(1);
    }

}

void GaitSelector::Receive(){
    // eta store the current motor pose
    for (int i=0; i<4; i++){
        eta[i][0] = motor_state_modules[i]->theta;
        eta[i][1]  = motor_state_modules[i]->beta;
        if (i==1 || i==2) {
            eta[i][1] = -eta[i][1];
        }
    }
    // for (int i = 0; i < 4; i++){
    //     std::cout << i << ": " 
    //               << eta[i][0] * 180 / M_PI << ", " 
    //               << eta[i][1] * 180 / M_PI << std::endl;
    // }
}

std::vector<double> GaitSelector::linspace(double start, double end, int num_steps) {
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

// Initialize static variables
std::array<double, 4> GaitSelector::duty = {0.0};
std::array<int, 4> GaitSelector::swing_phase = {0};

double GaitSelector::swing_time = 0.2;   
double GaitSelector::velocity = 0.05;  
double GaitSelector::stand_height = 0.14;
double GaitSelector::step_length = 0.4; 
double GaitSelector::step_height = 0.03; 

double GaitSelector::curvature = 0.0; // +: turn left, -:turn right, 0: straight

std::array<double, 4> GaitSelector::current_step_length = {step_length, step_length, step_length, step_length};
std::array<double, 4> GaitSelector::next_step_length    = {step_length, step_length, step_length, step_length};
double GaitSelector::new_step_length = step_length;

double GaitSelector::relative_foothold[4][2] = {0.0};
double GaitSelector::eta[4][2] = {0.0};
double GaitSelector::next_eta[4][2]= {0.0};
std::array<std::array<double, 2>, 4> GaitSelector::foothold= {0.0};
std::array<std::array<double, 2>, 4> GaitSelector::next_foothold=foothold;
std::array<std::array<double, 2>, 4> GaitSelector::body= {0.0};
std::array<std::array<double, 2>, 4> GaitSelector::next_body=body;
std::array<std::array<double, 2>, 4> GaitSelector::hip = {0.0};
std::array<std::array<double, 2>, 4> GaitSelector::next_hip = hip;

// For turning 
double GaitSelector::outer_radius = 0.0;
double GaitSelector::inner_radius = 0.0;
double GaitSelector::diff_step_length = 0.0;  // Differential step length 
double GaitSelector::new_diff_step_length = 0.0;  // New differential step length
double GaitSelector::diff_dS = 0.0;   // Differential dS
int GaitSelector::sign_diff[4] = {0.0};   // Differential sign

corgi_msgs::MotorStateStamped GaitSelector::motor_state = corgi_msgs::MotorStateStamped();

std::vector<corgi_msgs::MotorState*> GaitSelector::motor_state_modules = {
    &GaitSelector::motor_state.module_a,
    &GaitSelector::motor_state.module_b,
    &GaitSelector::motor_state.module_c,
    &GaitSelector::motor_state.module_d
};

// add statecallback
//  bool sim=true,
// double CoM_bias=0.0,
// int pub_rate=1000,
// double BL=0.444,
// double BW=0.4,
// double BH=0.2


// wheel add pub rate
// add Transfer
