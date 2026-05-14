#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "Steering.pb.h"


std::mutex mutex_motor_state;
std::mutex mutex_power_state;
std::mutex mutex_steer_state;

motor_msg::MotorCmdStamped          motor_cmd;
steering_msg::SteeringCmdStamped    steer_cmd;
motor_msg::MotorStateStamped        motor_state;
power_msg::PowerStateStamped        power_state;
steering_msg::SteeringStateStamped  steer_state;


void motor_cmd_cb(const motor_msg::MotorCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_motor_state);

    std::vector<motor_msg::MotorState*> motor_states = {
        motor_state.mutable_module_a(),
        motor_state.mutable_module_b(),
        motor_state.mutable_module_c(),
        motor_state.mutable_module_d()
    };

    std::vector<const motor_msg::MotorCmd*> motor_cmds = {
        &cmd.module_a(),
        &cmd.module_b(),
        &cmd.module_c(),
        &cmd.module_d()
    };

    std::cout << "TB_A: (" << motor_cmds[0]->theta() << ", " << motor_cmds[0]->beta() << ", " << motor_cmds[0]->gamma() << "); " << std::endl
              << "TB_B: (" << motor_cmds[1]->theta() << ", " << motor_cmds[1]->beta() << ", " << motor_cmds[1]->gamma() << "); " << std::endl
              << "TB_C: (" << motor_cmds[2]->theta() << ", " << motor_cmds[2]->beta() << ", " << motor_cmds[2]->gamma() << "); " << std::endl
              << "TB_D: (" << motor_cmds[3]->theta() << ", " << motor_cmds[3]->beta() << ", " << motor_cmds[3]->gamma() << "); " << std::endl << std::endl;

    for (int i = 0; i < 4; i++) {
        motor_states[i]->set_theta(motor_cmds[i]->theta());
        motor_states[i]->set_beta(motor_cmds[i]->beta());
        motor_states[i]->set_gamma(motor_cmds[i]->gamma());
        motor_states[i]->set_velocity_r(1);
        motor_states[i]->set_velocity_l(1);
        motor_states[i]->set_velocity_h(1);
        motor_states[i]->set_torque_r(1);
        motor_states[i]->set_torque_l(1);
        motor_states[i]->set_torque_h(1);
    }

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    motor_state.mutable_header()->set_seq(cmd.header().seq());
    motor_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    motor_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}


void steer_cmd_cb(const steering_msg::SteeringCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_steer_state);

    steer_state.set_current_angle(cmd.angle());
    steer_state.set_current_state(cmd.voltage());

    std::cout << cmd.angle() << std::endl;

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    steer_state.mutable_header()->set_seq(cmd.header().seq());
    steer_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    steer_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

void update_power_state() {
    static int seq = 0;

    power_state.mutable_header()->set_seq(seq++);
    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    power_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    power_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);

    power_state.set_pb1_digital(true);
    power_state.set_pb1_signal(false);
    power_state.set_pb1_power(true);
    power_state.set_pb2_digital(false);
    power_state.set_pb2_signal(true);
    power_state.set_pb2_power(false);
    power_state.set_clean(true);

    power_state.set_pb1_v_0(11.1);
    power_state.set_pb1_i_0(1.1);
    power_state.set_pb1_v_1(11.2);
    power_state.set_pb1_i_1(1.2);
    power_state.set_pb1_v_2(11.3);
    power_state.set_pb1_i_2(1.3);
    power_state.set_pb1_v_3(11.4);
    power_state.set_pb1_i_3(1.4);
    power_state.set_pb1_v_4(11.5);
    power_state.set_pb1_i_4(1.5);
    power_state.set_pb1_v_5(11.6);
    power_state.set_pb1_i_5(1.6);
    power_state.set_pb1_v_6(11.7);
    power_state.set_pb1_i_6(1.7);
    power_state.set_pb1_v_7(11.8);
    power_state.set_pb1_i_7(1.8);

    power_state.set_pb2_v_0(22.1);
    power_state.set_pb2_i_0(2.1);
    power_state.set_pb2_v_1(22.2);
    power_state.set_pb2_i_1(2.2);
    power_state.set_pb2_v_2(22.3);
    power_state.set_pb2_i_2(2.3);
    power_state.set_pb2_v_3(22.4);
    power_state.set_pb2_i_3(2.4);
    power_state.set_pb2_v_4(22.5);
    power_state.set_pb2_i_4(2.5);
    power_state.set_pb2_v_5(22.6);
    power_state.set_pb2_i_5(2.6);
    power_state.set_pb2_v_6(22.7);
    power_state.set_pb2_i_6(2.7);
    power_state.set_pb2_v_7(22.8);
    power_state.set_pb2_i_7(2.8);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_virtual_agent");

    core::NodeHandler nh_;
    core::Publisher<motor_msg::MotorStateStamped> &motor_state_pub = nh_.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Publisher<power_msg::PowerStateStamped> &power_state_pub = nh_.advertise<power_msg::PowerStateStamped>("power/state");
    core::Publisher<steering_msg::SteeringStateStamped> &steer_state_pub = nh_.advertise<steering_msg::SteeringStateStamped>("steer/state");
    core::Subscriber<motor_msg::MotorCmdStamped> &motor_cmd_sub = nh_.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    core::Subscriber<steering_msg::SteeringCmdStamped> &steer_cmd_sub = nh_.subscribe<steering_msg::SteeringCmdStamped>("steer/command", 1000, steer_cmd_cb);

    core::Rate rate(1000);

    while (rclcpp::ok()) {
        core::spinOnce();

        {
            std::lock_guard<std::mutex> lock(mutex_motor_state);
            motor_state_pub.publish(motor_state);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_power_state);
            update_power_state();
            power_state_pub.publish(power_state);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_steer_state);
            steer_state_pub.publish(steer_state);
        }

        rate.sleep();
    }

    rclcpp::shutdown();


    return 0;
}
