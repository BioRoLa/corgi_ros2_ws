#ifndef SIMPLE_FSM_HPP
#define SIMPLE_FSM_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/MotorStateStamped.h"

#include "leg_model.hpp"

enum class Gait {
    WHEEL = 0,
    LEG = 1,   
    WLW = 2,   
    TRANSFORM = 3
};
class GaitSelector {
    public:
        GaitSelector(   ros::NodeHandle& nh,
                        bool sim=true,
                        double CoM_bias=0.0,
                        int pub_rate=1000,
                        double BL=0.444,
                        double BW=0.4,
                        double BH=0.2);
        ~GaitSelector();

        ros::Subscriber motor_state_sub_;
        ros::Publisher motor_cmd_pub_; 
        ros::Rate* rate_ptr;
        std::random_device rd;                     
        std::mt19937 rng;                          
        std::uniform_int_distribution<int> dist;
    /*    State or cmd messages      */ 
        corgi_msgs::MotorCmdStamped motor_cmd;
        corgi_msgs::MotorStateStamped motor_state;
        std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
            &motor_cmd.module_a,
            &motor_cmd.module_b,
            &motor_cmd.module_c,
            &motor_cmd.module_d
        };
        std::vector<corgi_msgs::MotorState*> motor_state_modules = {
            &motor_state.module_a,
            &motor_state.module_b,
            &motor_state.module_c,
            &motor_state.module_d
        };
        void motor_state_cb(const corgi_msgs::MotorStateStamped state){
            motor_state = state;
        }               

        /*     Cooperate variables      */ 
        LegModel leg_model;
        int pub_rate;
        const double CoM_bias;
        const double BL;
        const double BW;
        const double BH;

        std::array<double, 4> duty;
        std::array<int, 4> swing_phase = {0, 0, 0, 0};
        double swing_time;   // ratio
        double velocity;     // m/s
        double stand_height; // m
        double step_length;  // m
        double step_height;  // m

        double eta[4][2];
        double next_eta[4][2];
        std::array<std::array<double, 2>, 4> foothold;
        std::array<std::array<double, 2>, 4> next_foothold;
        std::array<std::array<double, 2>, 4> body;
        std::array<std::array<double, 2>, 4> next_body;
        std::array<std::array<double, 2>, 4> hip;
        std::array<std::array<double, 2>, 4> next_hip;

        /*     Cooperate functions      */ 
        // ex.send, pub, 


    private:
        /*     Gait Selector     */ 
        Gait currentGait;
        Gait newGait;
        void changeGait(const std::string& command) {
            newGait = currentGait;

            if (command == "1") {
                newGait = Gait::WHEEL;
            } else if (command == "2") {
                newGait = Gait::LEG;
            } else if (command == "3") {
                newGait = Gait::WLW;
            } else if (command[0] == 'v') {
                std::string velocity = command.substr(1);
                std::cout << "Setting velocity to " << velocity << std::endl;
                return;
            } else if (command[0] == 'l') {
                std::string stepLength = command.substr(1);
                std::cout << "Setting step length to " << stepLength << std::endl;
                return;
            } else if (command[0] == 'h') {
                std::string standHeight = command.substr(1);
                std::cout << "Setting stand height to " << standHeight << std::endl;
                return;
            } else if (command[0] == 'j') {
                std::string liftHeight = command.substr(1);
                std::cout << "Setting lift height to " << liftHeight << std::endl;
                return;
            } else {
                std::cerr << "Unknown command: " << command << std::endl;
                return;
            }

            if (newGait != currentGait) {
                printCurrentGait();
                Transform();  
                currentGait = newGait; 
          }
        }
        void printCurrentGait() const {
            switch (currentGait) {
            case Gait::WHEEL:
                std::cout << "Current gait: WHEEL" << std::endl;
                break;
            case Gait::LEG:
                std::cout << "Current gait: LEG" << std::endl;
                break;
            case Gait::WLW:
                std::cout << "Current gait: WLW" << std::endl;
                break;
            case Gait::TRANSFORM:
                std::cout << "Current gait: TRANSFORM" << std::endl;
                break;
            default:
                std::cerr << "Unknown gait" << std::endl;
                break;
            }
        }
        void Transform() {
            switch (currentGait)
            {
            case Gait::WHEEL:
                currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(newGait == Gait::LEG) {
                    std::cout << "Transforming from WHEEL to LEG" << std::endl;
                } 
                else if(newGait == Gait::WLW) {
                    std::cout << "Transforming from WHEEL to WLW" << std::endl;
                }
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
            break;
            case Gait::LEG:
                currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(newGait == Gait::WHEEL) {
                    std::cout << "Transforming from LEG to WHEEL" << std::endl;
                } 
                else if(newGait == Gait::WLW) {
                    std::cout << "Transforming from LEG to WLW" << std::endl;
                } 
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
                break;
            case Gait::WLW:
                currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(newGait == Gait::WHEEL) {
                    std::cout << "Transforming from WLW to WHEEL" << std::endl;
                } 
                else if(newGait == Gait::LEG) {
                    std::cout << "Transforming from WLW to LEG" << std::endl;
                } 
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
                break;
            
            default:
                break;
            }
            
            std::cout << "Transforming..." << std::endl;
        }
    };
#endif

// gaitselextor setup add nodehandler and all initialize
// CoM_bias to 2D array(current 1D)