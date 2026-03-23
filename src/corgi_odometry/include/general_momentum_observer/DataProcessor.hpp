#ifndef DATA_PROCESSOR_HPP
#define DATA_PROCESSOR_HPP

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include <corgi_msgs/msg/imu_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>

/**
 * @brief ROS-free input record for offline CSV playback.
 *
 * Mirrors the fields of CSVReader::RobotData so that the same
 * processing logic can be shared between online and offline modes.
 */
struct RawRecord {
    // Base position and orientation (Ground Truth)
    double sim_pos_x = 0, sim_pos_y = 0, sim_pos_z = 0;
    double sim_orien_x = 0, sim_orien_y = 0, sim_orien_z = 0, sim_orien_w = 0;

    // IMU data
    double imu_orien_x = 0, imu_orien_y = 0, imu_orien_z = 0, imu_orien_w = 0;
    double imu_ang_vel_x = 0, imu_ang_vel_y = 0, imu_ang_vel_z = 0;
    double imu_lin_acc_x = 0, imu_lin_acc_y = 0, imu_lin_acc_z = 0;

    // Leg data (LF=a, RF=b, RH=c, LH=d)
    double state_theta_a = 0, state_beta_a = 0, state_vel_r_a = 0, state_vel_l_a = 0, state_trq_r_a = 0, state_trq_l_a = 0;
    double state_theta_b = 0, state_beta_b = 0, state_vel_r_b = 0, state_vel_l_b = 0, state_trq_r_b = 0, state_trq_l_b = 0;
    double state_theta_c = 0, state_beta_c = 0, state_vel_r_c = 0, state_vel_l_c = 0, state_trq_r_c = 0, state_trq_l_c = 0;
    double state_theta_d = 0, state_beta_d = 0, state_vel_r_d = 0, state_vel_l_d = 0, state_trq_r_d = 0, state_trq_l_d = 0;
};

/**
 * @brief Data Processor for both real-time and offline processing.
 *
 * Converts raw sensor data to the generalized-coordinate representation
 * used by the disturbance observer and ES-EKF.
 */
class DataProcessor {
public:
    DataProcessor(double dt);
    
    struct ProcessedData {
        Eigen::VectorXd q;      // (12,) - generalized coordinates
        Eigen::VectorXd q_dot;  // (12,) - generalized velocities
        Eigen::VectorXd tau;    // (8,) - joint torques
        Eigen::VectorXd I_c;    // (4,) - leg inertias
    };

    /**
     * @brief Process offline CSV record (with stateful velocity LPF).
     */
    ProcessedData process_record(const RawRecord& record);

    /**
     * @brief Process real-time data from ROS messages.
     */
    ProcessedData process_realtime_data(
        const geometry_msgs::msg::Vector3& position,
        const geometry_msgs::msg::Vector3& velocity,
        const corgi_msgs::msg::ImuStamped& imu,
        const corgi_msgs::msg::MotorStateStamped& motor_state
    );
    
private:
    double dt_;
    double low_pass_alpha_;
    bool first_call_ = true;

    // Stateful LPF for offline base velocity
    double last_sim_pos_x_ = 0.0, last_sim_pos_z_ = 0.0;
    double last_x_dot_ = 0.0, last_z_dot_ = 0.0;

    void cal_quaternion_to_euler(double x, double y, double z, double w, double& roll, double& pitch);
    double cal_theta_to_Rm(double theta);
    double cal_theta_dot_to_Rm_dot(double theta, double theta_dot);
    double cal_theta_to_Ic(double theta);
    void cal_motor_to_joint_torque(double theta, double torque_right, double torque_left,
                                    double& torque_beta, double& force_Rm);
};

#endif // DATA_PROCESSOR_HPP
