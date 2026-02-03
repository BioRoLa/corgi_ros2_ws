#ifndef DATA_PROCESSOR_HPP
#define DATA_PROCESSOR_HPP

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include <corgi_msgs/msg/imu_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>

/**
 * @brief Data Processor for real-time processing
 * Converts ROS messages to observer input format
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
     * @brief Process real-time data from ROS messages
     */
    ProcessedData process_realtime_data(
        const geometry_msgs::msg::Vector3& position,
        const geometry_msgs::msg::Vector3& velocity,
        const corgi_msgs::msg::ImuStamped& imu,
        const corgi_msgs::msg::MotorStateStamped& motor_state
    );
    
private:
    double dt_;
    
    void cal_quaternion_to_euler(double x, double y, double z, double w, double& roll, double& pitch);
    double cal_theta_to_Rm(double theta);
    double cal_theta_dot_to_Rm_dot(double theta, double theta_dot);
    double cal_theta_to_Ic(double theta);
    void cal_motor_to_joint_torque(double theta, double torque_right, double torque_left,
                                    double& torque_beta, double& force_Rm);
};

#endif // DATA_PROCESSOR_HPP
