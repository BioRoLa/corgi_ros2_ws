#pragma once

#include <array>

namespace corgi {

class Config {
public:
    // ============================================================
    // COMMON PARAMETERS
    // ============================================================
    
    // Degrees of freedom
    static constexpr int DOF = 12;
    
    // Physical constants
    static constexpr double G = 9.81;                // Gravitational acceleration [m/s^2]
    
    // Robot physical parameters
    static constexpr double BASE_WEIGHT = 19.0;      // Base mass [kg]
    static constexpr double LEG_WEIGHT = 0.65;       // Single leg mass [kg]
    static constexpr double BASE_LENGTH = 0.5;       // Base length [m]
    static constexpr double BASE_WIDTH = 0.2;        // Base width [m]
    static constexpr double BASE_HEIGHT = 0.1;       // Base height [m]
    static constexpr double WHEEL_RADIUS = 0.10;     // [m]
    static constexpr double TIRE_SKIN_RADIUS = 0.019;   // [m]
    static constexpr double WHEEL_WIDTH = 0.019;     // [m]
    
    // Geometric parameters
    static constexpr double LEG_X_OFFSET = 0.222;    // Leg offset in x-axis [m]
    static constexpr double LEG_Y_OFFSET = 0.193;    // Leg offset in y-axis [m]
    static constexpr double LEG_Z_OFFSET = 0.0;      // Leg offset in z-axis [m]
    
    // Base inertias (computed from dimensions)
    static constexpr double BODY_I_XX = BASE_WEIGHT * (BASE_LENGTH * BASE_LENGTH + BASE_HEIGHT * BASE_HEIGHT) / 12.0;
    static constexpr double BODY_I_YY = BASE_WEIGHT * (BASE_WIDTH * BASE_WIDTH + BASE_HEIGHT * BASE_HEIGHT) / 12.0;
    
    // Observer parameters
    
    // Sample rate and time step
    static constexpr double SAMPLE_RATE = 1000.0;         // Sampling frequency [Hz]
    static constexpr double DT = 1.0 / SAMPLE_RATE;       // Time step [s]
    
    // ES-EKF rate (runs at half the main loop rate)
    static constexpr double ESEKF_RATE = 500.0;            // ESEKF prediction rate [Hz]
    static constexpr double ESEKF_DT = 1.0 / ESEKF_RATE;  // Nominal ESEKF time step [s]
    static constexpr int    ESEKF_DECIMATION = static_cast<int>(SAMPLE_RATE / ESEKF_RATE);  // =2
    
    // Polynomial coefficients for Rm calculation
    // Rm(theta) = A[0]*theta^4 + A[1]*theta^3 + A[2]*theta^2 + A[3]*theta + A[4]
    static constexpr std::array<double, 5> RM_COEFF = {
        -0.0035, 0.0110, 0.0030, 0.0500, -0.0132
    };
    
    // Polynomial coefficients for leg inertia calculation
    // Ic(theta) = B[0]*theta^6 + B[1]*theta^5 + ... + B[5]*theta + B[6]
    static constexpr std::array<double, 7> IC_COEFF = {
        1e-06, -1e-05, 0.0001, -0.0002, -0.0012, 0.0042, 0.0041
    };
    
    // ============================================================
    // ONLINE PARAMETERS (used only in online ROS2 mode)
    // ============================================================
    
    // Node name and rate
    static constexpr const char* NODE_NAME = "contact_leg_estimator";
    static constexpr double ONLINE_LOOP_RATE = SAMPLE_RATE;  // Online loop rate [Hz]
    
    // Topic names for subscriptions
    static constexpr const char* TOPIC_MOTOR_STATE = "motor/state";
    static constexpr const char* TOPIC_IMU = "imu";
    static constexpr const char* TOPIC_ODOMETRY_POSITION = "odometry/position";
    static constexpr const char* TOPIC_ODOMETRY_VELOCITY = "odometry/velocity";
    static constexpr const char* TOPIC_TRIGGER = "trigger";
    
    // Topic names for publications
    static constexpr const char* TOPIC_CONTACT_STATE = "contact_state";
    
    // Queue sizes
    static constexpr int QUEUE_SIZE_SUB = 1;          // Subscriber queue size (use latest value)
    static constexpr int QUEUE_SIZE_PUB = 10;         // Publisher queue size
};

} // namespace corgi
