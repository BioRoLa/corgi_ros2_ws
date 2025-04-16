#ifndef CONFIG_HPP
#define CONFIG_HPP

struct CenterOfMass {
    double x;
    double y;
    double z;
};

struct Motor {
    double kt;
};

struct MotorModule {
    Motor motor_l;
    Motor motor_r;
};

struct RobotConfig {
    const bool sim;
    const CenterOfMass CoM;
    MotorModule module_a, module_b, module_c, module_d;

    RobotConfig(bool sim, CenterOfMass CoM, MotorModule a, MotorModule b, MotorModule c, MotorModule d)
    : sim(sim), CenterOfMass(CoM), module_a(a), module_b(b), module_c(c), module_d(d) {}
};

#endif // CONFIG_HPP