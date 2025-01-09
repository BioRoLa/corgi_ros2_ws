#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"

#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "sensor_msgs/Imu.h"

#include "KLD_estimation/InformationFilter.hpp"
#include "KLD_estimation/csv_reader.hpp"
using namespace estimation_model;

// Constants
const int SAMPLE_RATE = 200; //Hz
const float THRESHOLD = 0.08; //threshold of KLD
const int J = 10; //sample time (matrix size)
const float MOTOR_OFFSET[3] = {0.222, 0.193, 0};
const float WHEEL_RADIUS = 0.1;
const float WHEEL_WIDTH = 0.012;

//Classes
class Encoder{
    public:
        Encoder(corgi_msgs::MotorState* m, sensor_msgs::Imu* i): module(m), imu(i)
        {}

        void UpdateState(float dt){
            beta_d = (beta - beta_prev) / dt;
            theta_d = (theta - theta_prev) / dt;
            theta_prev = theta;
            beta_prev  = beta;
            theta = module->theta;
            beta  = module->beta;
            w_y = imu->angular_velocity.y;
            state << theta, beta, beta_d, w_y, theta_d;
        }

        void init(float dt){
            theta = module->theta;
            beta  = module->beta;
            theta_prev = theta;
            beta_prev  = beta;
            UpdateState(dt);
        }
        //const reference, prevent modified
        const Eigen::Matrix<float, 5, 1>& GetState() const{
            return state;
        }
    private:
        corgi_msgs::MotorState* module;
        sensor_msgs::Imu* imu;
        float theta;
        float beta;
        float theta_prev;
        float beta_prev;
        float theta_d;
        float beta_d;
        float w_y;
        Eigen::Vector<float, 5> state;
};

// Variables
bool trigger = false;
float dt = 1 / (float)SAMPLE_RATE;
bool initialized = false;
bool motor_state_initialized = false;
bool imu_initialized = false;

corgi_msgs::MotorStateStamped motor_state;
sensor_msgs::Imu imu;

// Callbacks
void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
    motor_state_initialized = true;
}

void imu_cb(const sensor_msgs::Imu msg){
    imu = msg;
    imu_initialized = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_odometry");

    ros::NodeHandle nh;

    // ROS Publishers
    /* TODO: add state publisher
    std::vector<std::string> cols = {
        "v_.x", "v_.y", "v_.z", 
        "p.x", "p.y", "p.z", 
        "zLF.x", "zLF.y", "zLF.z", 
        "zRF.x", "zRF.y", "zRF.z", 
        "zRH.x", "zRH.y", "zRH.z", 
        "zLH.x", "zLH.y", "zLH.z", 
        "zP.x", "zP.y", "zP.z", 
        "ba.x", "ba.y", "ba.z", 
        "lf.contact","rf.contact","rh.contact","lh.contact",
        "lf.cscore","rf.cscore","rh.cscore","lh.cscore",
        "lf.c","rf.c","rh.c","lh.c",
        "threshold",
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz"
    };*/
    // ROS Subscribers
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", SAMPLE_RATE, trigger_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", SAMPLE_RATE, motor_state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", SAMPLE_RATE, imu_cb);

    Eigen::initParallel();
    ros::Rate rate(SAMPLE_RATE);

    /* Estimate model initialization */

    //IMU data (Observation)
    U u(J, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), dt);

    //Legs model
    Leg lf_leg(Eigen::Vector3f( MOTOR_OFFSET[0],  MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg rf_leg(Eigen::Vector3f( MOTOR_OFFSET[0], -MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg rh_leg(Eigen::Vector3f(-MOTOR_OFFSET[0], -MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg lh_leg(Eigen::Vector3f(-MOTOR_OFFSET[0],  MOTOR_OFFSET[1], MOTOR_OFFSET[2]), WHEEL_RADIUS, WHEEL_WIDTH);
    
    //Legs encoder
    Encoder encoder_lf(&motor_state.module_a, &imu);
    Encoder encoder_rf(&motor_state.module_b, &imu);
    Encoder encoder_rh(&motor_state.module_c, &imu);
    Encoder encoder_lh(&motor_state.module_d, &imu);

    //Dynamic predictor
    DP lf(J + 1, lf_leg, &u);
    DP rf(J + 1, rf_leg, &u);
    DP rh(J + 1, rh_leg, &u);
    DP lh(J + 1, lh_leg, &u);

    while (ros::ok()){
        ros::spinOnce();
        if(trigger){
            
        }
        
        if (!initialized && motor_state_initialized && imu_initialized) {
            //Initialize encoder states
            encoder_lf.init(dt);
            encoder_rf.init(dt);
            encoder_rh.init(dt);
            encoder_lh.init(dt);
            //Dynamic predictor
            lf.init(encoder_lf.GetState(), 0);
            rf.init(encoder_rf.GetState(), 0);
            rh.init(encoder_rh.GetState(), 0);
            lh.init(encoder_lh.GetState(), 0);
            initialized = true;
        }
        else{
            //Update encoder states
            encoder_lf.UpdateState(dt);
            encoder_rf.UpdateState(dt);
            encoder_rh.UpdateState(dt);
            encoder_lh.UpdateState(dt);
        }
    }

    ros::shutdown();
    
    return 0;
}
