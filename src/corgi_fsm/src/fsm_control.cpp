#include "fsm_control.hpp"

#include "walk_gait.hpp"


corgi_msgs::MotorCmdStamped motor_cmd;
corgi_msgs::MotorStateStamped motor_state;
corgi_msgs::FsmCmdStamped fsm_cmd;
corgi_msgs::FsmStateStamped fsm_state;

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void fsm_cmd_cb(const corgi_msgs::FsmCmdStamped cmd){
    fsm_cmd = cmd;
}


int main(int argc, char **argv) {

    ROS_INFO("FSM Starts\n");

    ros::init(argc, argv, "corgi_fsm");

    ros::NodeHandle nh;
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    ros::Subscriber fsm_cmd_sub = nh.subscribe<corgi_msgs::FsmCmdStamped>("fsm/command", 1000, fsm_cmd_cb);
    ros::Publisher motor_cmd_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1000);
    ros::Publisher fsm_state_pub = nh.advertise<corgi_msgs::FsmStateStamped>("fsm/state", 1000);
    ros::Rate rate(1000);

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };


    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0;
        cmd->kp = 90;
        cmd->ki = 0;
        cmd->kd = 1.75;
    }


    int current_mode = REST_MODE;
    int next_mode = REST_MODE;
    bool switch_mode = false;

    bool to_walk_finished = true;
    bool to_wheel_finished = true;

    double CoM_bias = 0.0;
    int sampling_rate = 1000;
    double init_eta[8] = {18/180.0*M_PI, 0, 18/180.0*M_PI, 0, 18/180.0*M_PI, 0, 18/180.0*M_PI, 0};
    WalkGait walk_gait(init_eta, true, CoM_bias, sampling_rate);
    std::array<std::array<double, 4>, 2> walk_eta_list;
    double init_theta[4], init_beta[4];


    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        if (fsm_cmd.next_mode != current_mode && current_mode != TO_WALK_MODE && current_mode != TO_WHEEL_MODE) {
            next_mode = fsm_cmd.next_mode;
            switch_mode = true;
        }

        if (fsm_cmd.pause) {
            if (current_mode != REST_MODE) {
                ROS_INFO("FSM: PAUSE!");
                next_mode = REST_MODE;
                switch_mode = true;
            }
            else {
                switch_mode = false;
            }
        }

        if (fsm_cmd.stop) {
            ROS_INFO("FSM: STOP!");
            ros::shutdown();
            return 0;
        }

        // switch mode
        if (switch_mode){
            switch (next_mode) {
                case REST_MODE:
                    ROS_INFO("FSM: Entering REST MODE");
                    break;
                case CSV_MODE:
                    ROS_INFO("FSM: Entering CSV MODE");
                    break;
                case WHEEL_MODE:
                    ROS_INFO("FSM: Entering WHEEL MODE");
                    break;
                case WALK_MODE:
                    for (int i=0; i<4; i++){
                        init_theta[i] = motor_state_modules[i]->theta;
                        init_beta[i] = motor_state_modules[i]->beta;
                    }
                    walk_gait.initialize(init_theta, init_beta);
                    ROS_INFO("FSM: Entering WALK MODE");
                    break;
                case WLW_MODE:
                    ROS_INFO("FSM: Entering WLW MODE");
                    break;
                case TO_WALK_MODE:
                    to_walk_finished = false;
                    ROS_INFO("FSM: Entering TO WALK MODE");
                    break;
                case TO_WHEEL_MODE:
                    to_wheel_finished = false;
                    ROS_INFO("FSM: Entering TO WHEEL MODE");
                    break;
                default:
                    ROS_WARN("FSM: Unknown command received!");
                    break;
            }

            current_mode = next_mode;
            switch_mode = false;
        }
        
        // update
        switch (current_mode) {
            case REST_MODE:
                break;
            case CSV_MODE:
                break;
            case WHEEL_MODE:
                break;
            case WALK_MODE:
                walk_eta_list = walk_gait.step();
                for (int i=0; i<4; i++) {
                    if (walk_eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (walk_eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = walk_eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? walk_eta_list[1][i] : -walk_eta_list[1][i];
                }
                break;
            case WLW_MODE:
                break;
            case TO_WALK_MODE:




                if (to_walk_finished) {
                    switch_mode = true;
                    next_mode = WALK_MODE;
                }

                break;
            case TO_WHEEL_MODE:

                if (to_wheel_finished) {
                    switch_mode = true;
                    next_mode = WHEEL_MODE;
                }

                break;
            default:
                break;
        }


        // std::cout << "= = = = =" << std::endl << std::endl;

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = ros::Time::now();

        fsm_state.header.seq = loop_count;
        fsm_state.header.stamp = ros::Time::now();

        motor_cmd_pub.publish(motor_cmd);
        fsm_state_pub.publish(fsm_state);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}