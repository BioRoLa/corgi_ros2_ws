#pragma once

#include <array>

namespace quadruped {

class Config {
public:
    // File configuration
    static constexpr const char* CSV_FILENAME = "cpp_test";
    static constexpr int START_INDEX = 5000;
    static constexpr int DOF = 12;  // Degrees of freedom
    static constexpr bool ENABLE_LOGGING = true;
    
    // Physical constants
    static constexpr double G = 9.81;                // m/s^2
    
    // Robot physical parameters (SIM mode)
    static constexpr double BASE_WEIGHT = 19.0;      // kg
    static constexpr double LEG_WEIGHT = 0.65;       // kg
    static constexpr double BASE_LENGTH = 0.5;       // m
    static constexpr double BASE_WIDTH = 0.2;        // m
    static constexpr double BASE_HEIGHT = 0.1;       // m
    
    // Geometric parameters
    static constexpr double LEG_X_OFFSET = 0.222;   // m
    static constexpr double LEG_Y_OFFSET = 0.193;   // m
    
    // Base inertias (computed from dimensions)
    static constexpr double BODY_I_XX = BASE_WEIGHT * (BASE_LENGTH * BASE_LENGTH + BASE_HEIGHT * BASE_HEIGHT) / 12.0;
    static constexpr double BODY_I_YY = BASE_WEIGHT * (BASE_WIDTH * BASE_WIDTH + BASE_HEIGHT * BASE_HEIGHT) / 12.0;
    
    // Observer parameters
    static constexpr double OBSERVER_CUTOFF_FREQ = 15.0;  // Hz
    static constexpr double SAMPLE_RATE = 1000.0;         // Hz
    static constexpr double DT = 1.0 / SAMPLE_RATE;       // s
    
    // Encoder low pass filter
    static constexpr double ENCODER_CUTOFF_FREQ = 30.0;   // Hz
    
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
};

} // namespace quadruped
