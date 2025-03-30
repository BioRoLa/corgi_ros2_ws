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
    currentGait(Gait::WHEEL)
{
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &GaitSelector::motor_state_cb, this);
    motor_cmd_pub_ = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", pub_rate);
    rate_ptr = new ros::Rate(pub_rate);
}

GaitSelector::~GaitSelector() {
    delete rate_ptr;
    rate_ptr = nullptr;
}

void GaitSelector::keyboardInputThread() {
    while (ros::ok()) {
        char input_char;
        std::cout << "[l] Legged; [w] Wheeled; [h] Hybrid; [p] Pause; [r] Resume; [q] Quit: ";
        std::cin >> input_char;
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            switch (input_char) {
                case 'l':
                    std::cout << "Legged" << std::endl;
                    changeGait("2");
                    break;
                case 'w':
                    std::cout << "Wheeled" << std::endl;
                    changeGait("1");
                    break;
                case 'h':
                    std::cout << "Hybrid" << std::endl;
                    changeGait("3");
                    break;
                case 'p':
                    std::cout << "Paused" << std::endl;
                    break;
                case 'r': {
                    std::cout << "Resume" << std::endl;
                    break;
                }
                case 'q':
                    std::cout << "Quitting..." << std::endl;
                    ros::shutdown();
                    break;
                default:
                    std::cout << "Unrecognized input, keeping previous settings." << std::endl;
                    break;
            }
        }
    }
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

void GaitSelector::changeGait(const std::string& command) {
    newGait = currentGait;

    if (command == "1") {
        newGait = Gait::WHEEL;
    } else if (command == "2") {
        newGait = Gait::LEG;
    } else if (command == "3") {
        newGait = Gait::WLW;
    } 
    else if (command[0] == 'v') {
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
    } 
    else {
        std::cerr << "Unknown command: " << command << std::endl;
        return;
    }

    if (newGait != currentGait) {
        printCurrentGait();
        Transform();  
        currentGait = newGait; 
        printCurrentGait();

  }
}

void GaitSelector::printCurrentGait() const {
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

void GaitSelector::Transform() {
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


