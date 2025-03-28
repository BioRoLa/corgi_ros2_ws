#include <yaml.h>
#include <cstdlib> 
#include <stdexcept>
#include <string>
#include "config.hpp"

RobotConfig load_config()
{
    const char* home_path = std::getenv("HOME");
    if (!home_path) {
        throw std::runtime_error("HOME environment variable not set");
    }
    std::string config_file_path = std::string(home) + "/corgi_ws/corgi_ros_ws/config/config.yaml";
    YAML::Node yaml_node = YAML::LoadFile(config_file_path);

    const bool sim = yaml_node["sim"].as<bool>();
    double CoM_x = yaml_node["CoM"]["x"].as<double>();
    double CoM_y = yaml_node["CoM"]["y"].as<double>();
    double CoM_z = yaml_node["CoM"]["z"].as<double>();
    const CoM CoM{CoM_x, CoM_y, CoM_z};

    auto load_motor_module = [&](const YAML::Node& module_node) {
        double motor_l_kt = module_node["motor_l"]["kt"].as<double>();
        double motor_r_kt = module_node["motor_r"]["kt"].as<double>();
        return MotorModule{Motor{motor_l_kt}, Motor{motor_r_kt}};
    };
    MotorModule module_a = load_motor_module(yaml_config["module_a"]);
    MotorModule module_b = load_motor_module(yaml_config["module_b"]);
    MotorModule module_c = load_motor_module(yaml_config["module_c"]);
    MotorModule module_d = load_motor_module(yaml_config["module_d"]);

    // double module_a_motor_l_kt = yaml_node["module_a"]["motor_l"]["kt"].as<double>();
    // double module_a_motor_r_kt = yaml_node["module_a"]["motor_r"]["kt"].as<double>();
    // double module_b_motor_l_kt = yaml_node["module_b"]["motor_l"]["kt"].as<double>();
    // double module_b_motor_r_kt = yaml_node["module_b"]["motor_r"]["kt"].as<double>();
    // double module_c_motor_l_kt = yaml_node["module_c"]["motor_l"]["kt"].as<double>();
    // double module_c_motor_r_kt = yaml_node["module_c"]["motor_r"]["kt"].as<double>();
    // double module_d_motor_l_kt = yaml_node["module_d"]["motor_l"]["kt"].as<double>();
    // double module_d_motor_r_kt = yaml_node["module_d"]["motor_r"]["kt"].as<double>();
    // MotorModule module_a{Motor{module_a_motor_l_kt}, Motor{module_a_motor_r_kt}};
    // MotorModule module_b{Motor{module_b_motor_l_kt}, Motor{module_b_motor_r_kt}};
    // MotorModule module_c{Motor{module_c_motor_l_kt}, Motor{module_c_motor_r_kt}};
    // MotorModule module_d{Motor{module_d_motor_l_kt}, Motor{module_d_motor_r_kt}};
    
    return RobotConfig(sim, CoM, module_a, module_b, module_c, module_d);
}//end load_config
