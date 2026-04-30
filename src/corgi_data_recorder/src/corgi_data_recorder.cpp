// Because of the ROS2 update, the seq field of the imu has been removed. (2025-11-13)
// Updated to use custom ImuStamped message to preserve seq field. (2025-12-08)

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"
#include "corgi_msgs/msg/power_cmd_stamped.hpp"
#include "corgi_msgs/msg/power_state_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/imu_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/force_state_stamped.hpp"
#include "corgi_msgs/msg/sim_leg_contact_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


bool trigger = false;
corgi_msgs::msg::MotorCmdStamped motor_cmd;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::PowerCmdStamped power_cmd;
corgi_msgs::msg::PowerStateStamped power_state;
corgi_msgs::msg::ImuStamped imu;
sensor_msgs::msg::Range range_1;
sensor_msgs::msg::Range range_2;
sensor_msgs::msg::Range range_3;
sensor_msgs::msg::Range range_4;
corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::SimLegContactStamped sim_leg_contact_state;
geometry_msgs::msg::TransformStamped tf_data;
geometry_msgs::msg::WrenchStamped force_plate_1;
geometry_msgs::msg::WrenchStamped force_plate_2;
geometry_msgs::msg::WrenchStamped force_plate_3;
geometry_msgs::msg::WrenchStamped force_plate_4;
bool tf_data_valid = false;

std::ofstream output_file;
std::string output_file_name = "";
std::string output_file_path = "";

std_msgs::msg::Float32MultiArray camera;

// Check if file exists
bool file_exists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

// Signal handler for graceful shutdown
void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("corgi_data_recorder"), "Interrupt received.");

    if (output_file.is_open()) {
        output_file.close();
        RCLCPP_INFO(rclcpp::get_logger("corgi_data_recorder"), "Data successfully saved.");
    }

    rclcpp::shutdown();
    exit(signum);
}

// Callback function for trigger topic
void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
    
    output_file_name = msg.output_filename;
    
    if (trigger && msg.output_filename != "") { //triggered and output filename assigned
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros2_ws/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path + ".csv";
        while (file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + ".csv";
            index++;
        }
        if (index != 1) output_file_name += "_" + std::to_string(index-1);
        output_file_name += ".csv";

        output_file_path = file_path_with_extension;

        if (!output_file.is_open()) {
            output_file.open(output_file_path);
            output_file << "Time" << ","
                        << "cmd_seq" << "," << "cmd_sec" << "," << "cmd_nsec" << ","
                        << "cmd_theta_a" << "," << "cmd_beta_a" << "," << "cmd_gamma_a" << ","
                        << "cmd_trq_r_a" << "," << "cmd_trq_l_a" << "," << "cmd_trq_h_a" << ","
                        << "cmd_theta_b" << "," << "cmd_beta_b"  << "," << "cmd_gamma_b" << ","
                        << "cmd_trq_r_b" << "," << "cmd_trq_l_b" << "," << "cmd_trq_h_b" << ","
                        << "cmd_theta_c" << "," << "cmd_beta_c"  << "," << "cmd_gamma_c" << ","
                        << "cmd_trq_r_c" << "," << "cmd_trq_l_c" << "," << "cmd_trq_h_c" << ","
                        << "cmd_theta_d" << "," << "cmd_beta_d"  << "," << "cmd_gamma_d" << ","
                        << "cmd_trq_r_d" << "," << "cmd_trq_l_d" << "," << "cmd_trq_h_d" << ","

                        << "state_seq" << "," << "state_sec" << "," << "state_nsec" << ","
                        << "state_theta_a" << "," << "state_beta_a"  << "," << "state_gamma_a" << ","
                        << "state_vel_r_a" << "," << "state_vel_l_a" << "," << "state_vel_h_a" << ","
                        << "state_trq_r_a" << "," << "state_trq_l_a" << "," << "state_trq_h_a" << ","
                        << "state_theta_b" << "," << "state_beta_b"  << "," << "state_gamma_b" << ","
                        << "state_vel_r_b" << "," << "state_vel_l_b" << "," << "state_vel_h_b" << ","
                        << "state_trq_r_b" << "," << "state_trq_l_b" << "," << "state_trq_h_b" << ","
                        << "state_theta_c" << "," << "state_beta_c"  << "," << "state_gamma_c" << ","
                        << "state_vel_r_c" << "," << "state_vel_l_c" << "," << "state_vel_h_c" << ","
                        << "state_trq_r_c" << "," << "state_trq_l_c" << "," << "state_trq_h_c" << ","
                        << "state_theta_d" << "," << "state_beta_d"  << "," << "state_gamma_d" << ","
                        << "state_vel_r_d" << "," << "state_vel_l_d" << "," << "state_vel_h_d" << ","
                        << "state_trq_r_d" << "," << "state_trq_l_d" << "," << "state_trq_h_d" << ","

                        << "imu_seq" << "," << "imu_sec" << "," << "imu_nsec" << ","
                        << "imu_orien_x" << "," << "imu_orien_y" << "," << "imu_orien_z" << "," << "imu_orien_w" << ","
                        << "imu_ang_vel_x" << "," << "imu_ang_vel_y" << "," << "imu_ang_vel_z" << ","
                        << "imu_lin_acc_x" << "," << "imu_lin_acc_y" << "," << "imu_lin_acc_z" << ","

                        << "imp_seq" << "," << "imp_sec" << "," << "imp_nsec" << ","
                        << "imp_cmd_theta_a" << "," << "imp_cmd_beta_a" << ","
                        << "imp_cmd_Fx_a"    << "," << "imp_cmd_Fy_a"   << "," << "imp_cmd_Fz_a"   << ","
                        << "imp_cmd_kx_a"    << "," << "imp_cmd_ky_a"   << "," << "imp_cmd_kz_a"   << ","
                        << "imp_cmd_bx_a"    << "," << "imp_cmd_by_a"   << "," << "imp_cmd_bz_a"   << ","
                        << "imp_cmd_theta_b" << "," << "imp_cmd_beta_b" << ","
                        << "imp_cmd_Fx_b"    << "," << "imp_cmd_Fy_b"   << "," << "imp_cmd_Fz_b"   << ","
                        << "imp_cmd_kx_b"    << "," << "imp_cmd_ky_b"   << "," << "imp_cmd_kz_b"   << ","
                        << "imp_cmd_bx_b"    << "," << "imp_cmd_by_b"   << "," << "imp_cmd_bz_b"   << ","
                        << "imp_cmd_theta_c" << "," << "imp_cmd_beta_c" << ","
                        << "imp_cmd_Fx_c"    << "," << "imp_cmd_Fy_c"   << "," << "imp_cmd_Fz_c"   << ","
                        << "imp_cmd_kx_c"    << "," << "imp_cmd_ky_c"   << "," << "imp_cmd_kz_c"   << ","
                        << "imp_cmd_bx_c"    << "," << "imp_cmd_by_c"   << "," << "imp_cmd_bz_c"   << ","
                        << "imp_cmd_theta_d" << "," << "imp_cmd_beta_d" << ","
                        << "imp_cmd_Fx_d"    << "," << "imp_cmd_Fy_d"   << "," << "imp_cmd_Fz_d"   << ","
                        << "imp_cmd_kx_d"    << "," << "imp_cmd_ky_d"   << "," << "imp_cmd_kz_d"   << ","
                        << "imp_cmd_bx_d"    << "," << "imp_cmd_by_d"   << "," << "imp_cmd_bz_d"   << ","

                        << "force_seq" << "," << "force_sec" << "," << "force_nsec" << ","
                        << "force_Fx_a" << "," << "force_Fy_a" << "," << "force_Fz_a" << ","
                        << "force_Fx_b" << "," << "force_Fy_b" << "," << "force_Fz_b" << ","
                        << "force_Fx_c" << "," << "force_Fy_c" << "," << "force_Fz_c" << ","
                        << "force_Fx_d" << "," << "force_Fy_d" << "," << "force_Fz_d" << ","

                        << "sim_sec" << "," << "sim_nsec" << ","
                        << "sim_pos_x" << "," << "sim_pos_y" << "," << "sim_pos_z" << ","
                        << "sim_orien_x" << "," << "sim_orien_y" << "," << "sim_orien_z" << "," << "sim_orien_w" << ","
                        << "sim_contact_state_a" << "," << "sim_contact_state_b" << "," << "sim_contact_state_c" << "," << "sim_contact_state_d" << ","
                        // << "sim_dst_lf" << "," << "sim_dst_lh" << "," << "sim_dst_rf" << "," << "sim_dst_rh" << ","

                        << "power_seq" << "," << "power_sec" << "," << "power_nsec" << ","
                        << "v_0" << "," << "i_0" << ","
                        << "v_1" << "," << "i_1" << ","
                        << "v_2" << "," << "i_2" << ","
                        << "v_3" << "," << "i_3" << ","
                        << "v_4" << "," << "i_4" << ","
                        << "v_5" << "," << "i_5" << ","
                        << "v_6" << "," << "i_6" << ","
                        << "v_7" << "," << "i_7" << ","
                        << "v_8" << "," << "i_8" << ","
                        << "v_9" << "," << "i_9" << ","
                        << "v_10" << "," << "i_10" << ","
                        << "v_11" << "," << "i_11"<< ","

                        << "fp1_sec" << "," << "fp1_nsec" << ","
                        << "fp1_fx" << "," << "fp1_fy" << "," << "fp1_fz" << ","
                        << "fp1_tx" << "," << "fp1_ty" << "," << "fp1_tz" << ","
                        << "fp2_sec" << "," << "fp2_nsec" << ","
                        << "fp2_fx" << "," << "fp2_fy" << "," << "fp2_fz" << ","
                        << "fp2_tx" << "," << "fp2_ty" << "," << "fp2_tz" << ","
                        << "fp3_sec" << "," << "fp3_nsec" << ","
                        << "fp3_fx" << "," << "fp3_fy" << "," << "fp3_fz" << ","
                        << "fp3_tx" << "," << "fp3_ty" << "," << "fp3_tz" << ","
                        << "fp4_sec" << "," << "fp4_nsec" << ","
                        << "fp4_fx" << "," << "fp4_fy" << "," << "fp4_fz" << ","
                        << "fp4_tx" << "," << "fp4_ty" << "," << "fp4_tz" << ","

                        << "dst_lf" << "," << "dst_rf" << "," << "dst_rr" << "," << "dst_lr" << ","
                        << "camera_dist" << "," << "camera_yaw" 
                        << "\n";

            RCLCPP_INFO(rclcpp::get_logger("corgi_data_recorder"), "Recording data to %s", output_file_name.c_str());
        }
    }
    else {
        if (output_file.is_open()) {
            output_file.close();
            RCLCPP_INFO(rclcpp::get_logger("corgi_data_recorder"), "Stopped recording data");
        }
    }
}


void motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped::SharedPtr cmd){
    motor_cmd = *cmd;
}


void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr state){
    motor_state = *state;
}


void power_cmd_cb(const corgi_msgs::msg::PowerCmdStamped::SharedPtr cmd){
    power_cmd = *cmd;
}


void power_state_cb(const corgi_msgs::msg::PowerStateStamped::SharedPtr state){
    power_state = *state;
}

void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr values){
    imu = *values;
}

void imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr cmd){
    imp_cmd = *cmd;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr state){
    force_state = *state;
}

void sim_leg_contact_state_cb(const corgi_msgs::msg::SimLegContactStamped::SharedPtr state){
    sim_leg_contact_state = *state;
}

void stair_info_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    // Process stair information if needed
    // camera
    // camera.data.resize(msg->data.size());
    camera.data = msg->data;
    // float dist = msg->data[0];
    // float yaw  = msg->data[1];
}

void range_1_cb(const sensor_msgs::msg::Range::SharedPtr msg){
    range_1 = *msg;
}

void range_2_cb(const sensor_msgs::msg::Range::SharedPtr msg){
    range_2 = *msg;
}

void range_3_cb(const sensor_msgs::msg::Range::SharedPtr msg){
    range_3 = *msg;
}

void range_4_cb(const sensor_msgs::msg::Range::SharedPtr msg){
    range_4 = *msg;
}

void force_plate_1_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    force_plate_1 = *msg;
}

void force_plate_2_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    force_plate_2 = *msg;
}

void force_plate_3_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    force_plate_3 = *msg;
}

void force_plate_4_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    force_plate_4 = *msg;
}

// write data to CSV file
void write_data(rclcpp::Node::SharedPtr node) {
    if (!output_file.is_open()){
        if (output_file_name != "") 
            RCLCPP_INFO(rclcpp::get_logger("corgi_data_recorder"), "Output file is not opened");
        return;
    }

    output_file << node->now().nanoseconds() / 1e9 << ","
                << motor_cmd.header.seq << "," << motor_cmd.header.stamp.sec << "," << motor_cmd.header.stamp.nanosec << ","
                << motor_cmd.module_a.theta << "," << motor_cmd.module_a.beta << "," << motor_cmd.module_a.gamma << ","
                << motor_cmd.module_a.torque_r << "," << motor_cmd.module_a.torque_l << "," << motor_cmd.module_a.torque_h << ","
                << motor_cmd.module_b.theta << "," << motor_cmd.module_b.beta << "," << motor_cmd.module_b.gamma << ","
                << motor_cmd.module_b.torque_r << "," << motor_cmd.module_b.torque_l << "," << motor_cmd.module_b.torque_h << ","
                << motor_cmd.module_c.theta << "," << motor_cmd.module_c.beta << "," << motor_cmd.module_c.gamma << ","
                << motor_cmd.module_c.torque_r << "," << motor_cmd.module_c.torque_l << "," << motor_cmd.module_c.torque_h << ","
                << motor_cmd.module_d.theta << "," << motor_cmd.module_d.beta << "," << motor_cmd.module_d.gamma << ","
                << motor_cmd.module_d.torque_r << "," << motor_cmd.module_d.torque_l << "," << motor_cmd.module_d.torque_h << ","

                << motor_state.header.seq << "," << motor_state.header.stamp.sec << "," << motor_state.header.stamp.nanosec << ","
                << motor_state.module_a.theta      << "," << motor_state.module_a.beta << "," << motor_state.module_a.gamma << ","
                << motor_state.module_a.velocity_r << "," << motor_state.module_a.velocity_l << "," << motor_state.module_a.velocity_h << ","
                << motor_state.module_a.torque_r   << "," << motor_state.module_a.torque_l << "," << motor_state.module_a.torque_h << ","
                << motor_state.module_b.theta      << "," << motor_state.module_b.beta << "," << motor_state.module_b.gamma << ","
                << motor_state.module_b.velocity_r << "," << motor_state.module_b.velocity_l << "," << motor_state.module_b.velocity_h << ","
                << motor_state.module_b.torque_r   << "," << motor_state.module_b.torque_l << "," << motor_state.module_b.torque_h << ","
                << motor_state.module_c.theta      << "," << motor_state.module_c.beta << "," << motor_state.module_c.gamma << ","
                << motor_state.module_c.velocity_r << "," << motor_state.module_c.velocity_l << "," << motor_state.module_c.velocity_h << ","
                << motor_state.module_c.torque_r   << "," << motor_state.module_c.torque_l << "," << motor_state.module_c.torque_h << ","
                << motor_state.module_d.theta      << "," << motor_state.module_d.beta << "," << motor_state.module_d.gamma << ","
                << motor_state.module_d.velocity_r << "," << motor_state.module_d.velocity_l << "," << motor_state.module_d.velocity_h << ","
                << motor_state.module_d.torque_r   << "," << motor_state.module_d.torque_l << "," << motor_state.module_d.torque_h << ","

                << imu.header.seq << "," << imu.header.stamp.sec << "," << imu.header.stamp.nanosec << ","
                << imu.orientation.x << "," << imu.orientation.y << "," << imu.orientation.z << "," << imu.orientation.w << ","
                << imu.angular_velocity.x << "," << imu.angular_velocity.y << "," << imu.angular_velocity.z << ","
                << imu.linear_acceleration.x << "," << imu.linear_acceleration.y << "," << imu.linear_acceleration.z << ","

                << imp_cmd.header.seq << "," << imp_cmd.header.stamp.sec << "," << imp_cmd.header.stamp.nanosec << ","
                << imp_cmd.module_a.theta << "," << imp_cmd.module_a.beta << ","
                << imp_cmd.module_a.fx    << "," << imp_cmd.module_a.fy << "," << imp_cmd.module_a.fz << ","
                << imp_cmd.module_a.kx    << "," << imp_cmd.module_a.ky << "," << imp_cmd.module_a.kz << ","
                << imp_cmd.module_a.bx    << "," << imp_cmd.module_a.by << "," << imp_cmd.module_a.bz << ","
                << imp_cmd.module_b.theta << "," << imp_cmd.module_b.beta << ","
                << imp_cmd.module_b.fx    << "," << imp_cmd.module_b.fy << "," << imp_cmd.module_b.fz << ","
                << imp_cmd.module_b.kx    << "," << imp_cmd.module_b.ky << "," << imp_cmd.module_b.kz << ","
                << imp_cmd.module_b.bx    << "," << imp_cmd.module_b.by << "," << imp_cmd.module_b.bz << ","
                << imp_cmd.module_c.theta << "," << imp_cmd.module_c.beta << ","
                << imp_cmd.module_c.fx    << "," << imp_cmd.module_c.fy << "," << imp_cmd.module_c.fz << ","
                << imp_cmd.module_c.kx    << "," << imp_cmd.module_c.ky << "," << imp_cmd.module_c.kz << ","
                << imp_cmd.module_c.bx    << "," << imp_cmd.module_c.by << "," << imp_cmd.module_c.bz << ","
                << imp_cmd.module_d.theta << "," << imp_cmd.module_d.beta << ","
                << imp_cmd.module_d.fx    << "," << imp_cmd.module_d.fy << "," << imp_cmd.module_d.fz << ","
                << imp_cmd.module_d.kx    << "," << imp_cmd.module_d.ky << "," << imp_cmd.module_d.kz << ","
                << imp_cmd.module_d.bx    << "," << imp_cmd.module_d.by << "," << imp_cmd.module_d.bz << ","

                << force_state.header.seq << "," << force_state.header.stamp.sec << "," << force_state.header.stamp.nanosec << ","
                << force_state.module_a.fx    << "," << force_state.module_a.fy << "," << force_state.module_a.fz << ","
                << force_state.module_b.fx    << "," << force_state.module_b.fy << "," << force_state.module_b.fz << ","
                << force_state.module_c.fx    << "," << force_state.module_c.fy << "," << force_state.module_c.fz << ","
                << force_state.module_d.fx    << "," << force_state.module_d.fy << "," << force_state.module_d.fz << ","

                << tf_data.header.stamp.sec << "," << tf_data.header.stamp.nanosec << ","
                << tf_data.transform.translation.x << "," << tf_data.transform.translation.y << "," << tf_data.transform.translation.z << ","
                << tf_data.transform.rotation.x << "," << tf_data.transform.rotation.y << "," << tf_data.transform.rotation.z << "," << tf_data.transform.rotation.w << ","

                << sim_leg_contact_state.module_a.contact << ","
                << sim_leg_contact_state.module_b.contact << ","
                << sim_leg_contact_state.module_c.contact << ","
                << sim_leg_contact_state.module_d.contact << ","

                << power_state.header.seq << "," << power_state.header.stamp.sec << "," << power_state.header.stamp.nanosec << ","
                << power_state.v_0 << "," << power_state.i_0 << ","
                << power_state.v_1 << "," << power_state.i_1 << ","
                << power_state.v_2 << "," << power_state.i_2 << ","
                << power_state.v_3 << "," << power_state.i_3 << ","
                << power_state.v_4 << "," << power_state.i_4 << ","
                << power_state.v_5 << "," << power_state.i_5 << ","
                << power_state.v_6 << "," << power_state.i_6 << ","
                << power_state.v_7 << "," << power_state.i_7 << ","
                << power_state.v_8 << "," << power_state.i_8 << ","
                << power_state.v_9 << "," << power_state.i_9 << ","
                << power_state.v_10 << "," << power_state.i_10 << ","
                << power_state.v_11 << "," << power_state.i_11 << ","

                << force_plate_1.header.stamp.sec << "," << force_plate_1.header.stamp.nanosec << ","
                << force_plate_1.wrench.force.x << "," << force_plate_1.wrench.force.y << "," << force_plate_1.wrench.force.z << ","
                << force_plate_1.wrench.torque.x << "," << force_plate_1.wrench.torque.y << "," << force_plate_1.wrench.torque.z << ","
                << force_plate_2.header.stamp.sec << "," << force_plate_2.header.stamp.nanosec << ","
                << force_plate_2.wrench.force.x << "," << force_plate_2.wrench.force.y << "," << force_plate_2.wrench.force.z << ","
                << force_plate_2.wrench.torque.x << "," << force_plate_2.wrench.torque.y << "," << force_plate_2.wrench.torque.z << ","
                << force_plate_3.header.stamp.sec << "," << force_plate_3.header.stamp.nanosec << ","
                << force_plate_3.wrench.force.x << "," << force_plate_3.wrench.force.y << "," << force_plate_3.wrench.force.z << ","
                << force_plate_3.wrench.torque.x << "," << force_plate_3.wrench.torque.y << "," << force_plate_3.wrench.torque.z << ","
                << force_plate_4.header.stamp.sec << "," << force_plate_4.header.stamp.nanosec << ","
                << force_plate_4.wrench.force.x << "," << force_plate_4.wrench.force.y << "," << force_plate_4.wrench.force.z << ","
                << force_plate_4.wrench.torque.x << "," << force_plate_4.wrench.torque.y << "," << force_plate_4.wrench.torque.z << ","

                << range_1.range << "," << range_2.range << "," << range_3.range << "," << range_4.range ;

                // camera 資訊
                if (camera.data.size() >= 2) {
                    output_file << "," << camera.data[0] << "," << camera.data[1];
                } else {
                    output_file << ",NaN,NaN";
                }

                // 換行 & flush
                output_file << "\n";
                output_file.flush();
                
    // output_file.flush();
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("corgi_data_recorder"), "Data Recorder Starts\n");

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("corgi_data_recorder");

    // Only wait for Webots /clock when running in simulation mode.
    // In real-hardware mode (use_sim_time=false) the system wall clock is used
    // and node->now() is already valid — no need to wait.
    bool use_sim_time = false;
    node->get_parameter_or("use_sim_time", use_sim_time, false);

    if (use_sim_time) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (node->now().seconds() > 0.0) {
                RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_INFO(node->get_logger(), "Real hardware mode: using system wall clock.");
    }
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 100, trigger_cb);
    auto motor_cmd_sub = node->create_subscription<corgi_msgs::msg::MotorCmdStamped>("motor/command", 100, motor_cmd_cb);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 100, motor_state_cb);
    auto power_cmd_sub = node->create_subscription<corgi_msgs::msg::PowerCmdStamped>("power/command", 100, power_cmd_cb);
    auto power_state_sub = node->create_subscription<corgi_msgs::msg::PowerStateStamped>("power/state", 100, power_state_cb);
    auto imu_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>("imu", 100, imu_cb);
    auto imp_cmd_sub = node->create_subscription<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 100, imp_cmd_cb);
    auto force_state_sub = node->create_subscription<corgi_msgs::msg::ForceStateStamped>("force/state", 100, force_state_cb);
    auto sim_leg_contact_state_sub = node->create_subscription<corgi_msgs::msg::SimLegContactStamped>("sim/leg_contact", 100, sim_leg_contact_state_cb);
    // TF listener for odom -> base_link transform
    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto stair_info_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>("stair_plane_info", 100, stair_info_cb);
    auto range_sub_1 = node->create_subscription<sensor_msgs::msg::Range>("range_1", 100, range_1_cb);
    auto range_sub_2 = node->create_subscription<sensor_msgs::msg::Range>("range_2", 100, range_2_cb);
    auto range_sub_3 = node->create_subscription<sensor_msgs::msg::Range>("range_3", 100, range_3_cb);
    auto range_sub_4 = node->create_subscription<sensor_msgs::msg::Range>("range_4", 100, range_4_cb);
    auto force_plate_sub_1 = node->create_subscription<geometry_msgs::msg::WrenchStamped>("sensor/force_plate_1", 100, force_plate_1_cb);
    auto force_plate_sub_2 = node->create_subscription<geometry_msgs::msg::WrenchStamped>("sensor/force_plate_2", 100, force_plate_2_cb);
    auto force_plate_sub_3 = node->create_subscription<geometry_msgs::msg::WrenchStamped>("sensor/force_plate_3", 100, force_plate_3_cb);
    auto force_plate_sub_4 = node->create_subscription<geometry_msgs::msg::WrenchStamped>("sensor/force_plate_4", 100, force_plate_4_cb);

    // rclcpp::Rate rate(1000);
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = node->now();//timer init

    signal(SIGINT, signal_handler);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Try to get TF data
        try {
            tf_data = tfBuffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
            tf_data_valid = true;
        } catch (const tf2::TransformException &ex) {
            // TF not available yet, keep previous value
        }

        if (trigger) {
            write_data(node);
        }
        next_time += period;
        if(!node->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
            break;
        }
        // rate.sleep();
    }

    if (output_file.is_open()) {
        output_file.close();
    }

    rclcpp::shutdown();

    return 0;
}