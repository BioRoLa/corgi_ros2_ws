#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"

#include "corgi_msgs/MotorStateStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "KLD_estimation/InformationFilter.hpp"
#include "KLD_estimation/csv_reader.hpp"
using namespace estimation_model;

// Constants
const int SAMPLE_RATE = 200; //Hz
const float THREHOLD = 0.08; //threshold of KLD
const int J = 10; //sample time (matrix size)
const float MOTOR_OFFSET[3] = {0.222, 0.193, 0};
const float WHEEL_RADIUS = 0.1;
const float WHEEL_WIDTH = 0.012;

//Classes
class Encoder{
    public:
        Encoder(corgi_msgs::MotorState* m, corgi_msgs::IMUStamped* i, float dt)//FIXME: undetermined IMU msg
        :   module(m), 
            imu(i),
            theta(m->theta),
            beta(m->beta),
            theta_prev(m->theta),
            beta_prev(m->beta),
            w_y(i->w_y) //FIXME: undetermined msg
        {
            updateState();
        }
            
        void updateState(float dt) {
            // TODO: update
            beta_d = 
            theta_d = 
            theta_prev = theta;
            beta_prev  = beta;
            theta = module->theta;
            beta  = module->beta;
            state << theta, beta, beta_d, w_y, theta_d;
        }
        //const reference, prevent modified
        const Eigen::Matrix<float, 5, 1>& getState() const {
            return state;
        }
    private:
        corgi_msgs::MotorState* module;
        corgi_msgs::IMUStamped* imu;
        float theta;
        float beta;
        float theta_prev;
        float beta_prev;
        float theta_d;
        float beta_d;
        float w_y;
        Eigen::Vector<float, 5> state
}

// Variables
bool trigger = false;
float dt = 1 / (float)SAMPLE_RATE;

corgi_msgs::MotorStateStamped motor_state;

// Callbacks
void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;

}
// Eigen::Vector<float, 5> encoder_lf(df.iloc("lf.theta", start_index), df.iloc("lf.beta", start_index), df.iloc("lf.beta_d", start_index), df.iloc("w.y", start_index), df.iloc("lf.theta_d", start_index));

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
    // TODO: Add IMU(ax,ay,az),IMU(wx,wy,wz),IMU(quaternion),leg encoder(theta,beta,(beta_d,theta_d))

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
    
    //Dynamic predictor
    Encoder encoder_lf;
    Encoder encoder_rf;
    Encoder encoder_rh;
    Encoder encoder_lh;
    Eigen::Vector<float, 5> encoder_lf();
    DP lf(J + 1, lf_leg, encoder_lf.state, 0, &u);
    DP rf(J + 1, rf_leg, encoder_rf.state, 0, &u);
    DP rh(J + 1, rh_leg, encoder_rh.state, 0, &u);
    DP lh(J + 1, lh_leg, encoder_lh.state, 0, &u);

    while (ros::ok()){
        ros::spinOnce();
        if(trigger){

        }
    }

    ros::shutdown();
    
    return 0;
}
