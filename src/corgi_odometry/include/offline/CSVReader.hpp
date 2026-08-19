#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>
#include <stdexcept>
#include <iostream>

namespace corgi {

/**
 * @brief CSV Reader with column name support for offline test data.
 */
class CSVReader {
public:
    struct RobotData {
        // Base position and orientation (Ground Truth)
        double sim_pos_x = 0, sim_pos_y = 0, sim_pos_z = 0;
        double sim_orien_x = 0, sim_orien_y = 0, sim_orien_z = 0, sim_orien_w = 0;

        // IMU timestamp
        int32_t imu_seq = 0;
        int32_t imu_sec = 0;
        int32_t imu_nsec = 0;

        // IMU data
        double imu_orien_x = 0, imu_orien_y = 0, imu_orien_z = 0, imu_orien_w = 0;
        double imu_ang_vel_x = 0, imu_ang_vel_y = 0, imu_ang_vel_z = 0;
        double imu_lin_acc_x = 0, imu_lin_acc_y = 0, imu_lin_acc_z = 0;

        // Leg data (LF, RF, RH, LH)
        double state_theta_a = 0, state_beta_a = 0, state_vel_r_a = 0, state_vel_l_a = 0, state_trq_r_a = 0, state_trq_l_a = 0;
        double state_theta_b = 0, state_beta_b = 0, state_vel_r_b = 0, state_vel_l_b = 0, state_trq_r_b = 0, state_trq_l_b = 0;
        double state_theta_c = 0, state_beta_c = 0, state_vel_r_c = 0, state_vel_l_c = 0, state_trq_r_c = 0, state_trq_l_c = 0;
        double state_theta_d = 0, state_beta_d = 0, state_vel_r_d = 0, state_vel_l_d = 0, state_trq_r_d = 0, state_trq_l_d = 0;

        // Stage 1.5 camber: ABAD tilt gamma [rad] and hip motor velocity
        // (gamma rate) [rad/s]. Optional columns — default 0 when absent.
        double state_gamma_a = 0, state_gamma_b = 0, state_gamma_c = 0, state_gamma_d = 0;
        double state_vel_h_a = 0, state_vel_h_b = 0, state_vel_h_c = 0, state_vel_h_d = 0;
    };

    std::vector<RobotData> read_csv(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }

        // Count lines for pre-allocation (skip header)
        size_t line_count = 0;
        {
            std::string tmp;
            while (std::getline(file, tmp)) ++line_count;
            if (line_count > 0) --line_count; // subtract header
        }
        file.clear();
        file.seekg(0);

        std::vector<RobotData> data;
        data.reserve(line_count);

        // Read header line and parse column names
        std::string header_line;
        std::getline(file, header_line);
        auto column_indices = parse_header(header_line);

        // Read data lines
        std::string line;
        while (std::getline(file, line)) {
            auto values = parse_line(line);

            if (values.size() <= column_indices.at("sim_pos_z")) {
                std::cerr << "Warning: Skipping malformed line\n";
                continue;
            }

            RobotData row;

            row.sim_pos_x   = values[column_indices.at("sim_pos_x")];
            row.sim_pos_y   = values[column_indices.at("sim_pos_y")];
            row.sim_pos_z   = values[column_indices.at("sim_pos_z")];
            row.sim_orien_x = values[column_indices.at("sim_orien_x")];
            row.sim_orien_y = values[column_indices.at("sim_orien_y")];
            row.sim_orien_z = values[column_indices.at("sim_orien_z")];
            row.sim_orien_w = values[column_indices.at("sim_orien_w")];

            // IMU timestamp (optional — absent in simulation CSVs)
            if (column_indices.count("imu_seq"))
                row.imu_seq  = static_cast<int32_t>(values[column_indices.at("imu_seq")]);
            if (column_indices.count("imu_sec"))
                row.imu_sec  = static_cast<int32_t>(values[column_indices.at("imu_sec")]);
            if (column_indices.count("imu_nsec"))
                row.imu_nsec = static_cast<int32_t>(values[column_indices.at("imu_nsec")]);

            row.imu_orien_x   = values[column_indices.at("imu_orien_x")];
            row.imu_orien_y   = values[column_indices.at("imu_orien_y")];
            row.imu_orien_z   = values[column_indices.at("imu_orien_z")];
            row.imu_orien_w   = values[column_indices.at("imu_orien_w")];
            row.imu_ang_vel_x = values[column_indices.at("imu_ang_vel_x")];
            row.imu_ang_vel_y = values[column_indices.at("imu_ang_vel_y")];
            row.imu_ang_vel_z = values[column_indices.at("imu_ang_vel_z")];
            row.imu_lin_acc_x = values[column_indices.at("imu_lin_acc_x")];
            row.imu_lin_acc_y = values[column_indices.at("imu_lin_acc_y")];
            row.imu_lin_acc_z = values[column_indices.at("imu_lin_acc_z")];

            row.state_theta_a = values[column_indices.at("state_theta_a")];
            row.state_beta_a  = values[column_indices.at("state_beta_a")];
            row.state_vel_r_a = values[column_indices.at("state_vel_r_a")];
            row.state_vel_l_a = values[column_indices.at("state_vel_l_a")];
            row.state_trq_r_a = values[column_indices.at("state_trq_r_a")];
            row.state_trq_l_a = values[column_indices.at("state_trq_l_a")];

            row.state_theta_b = values[column_indices.at("state_theta_b")];
            row.state_beta_b  = values[column_indices.at("state_beta_b")];
            row.state_vel_r_b = values[column_indices.at("state_vel_r_b")];
            row.state_vel_l_b = values[column_indices.at("state_vel_l_b")];
            row.state_trq_r_b = values[column_indices.at("state_trq_r_b")];
            row.state_trq_l_b = values[column_indices.at("state_trq_l_b")];

            row.state_theta_c = values[column_indices.at("state_theta_c")];
            row.state_beta_c  = values[column_indices.at("state_beta_c")];
            row.state_vel_r_c = values[column_indices.at("state_vel_r_c")];
            row.state_vel_l_c = values[column_indices.at("state_vel_l_c")];
            row.state_trq_r_c = values[column_indices.at("state_trq_r_c")];
            row.state_trq_l_c = values[column_indices.at("state_trq_l_c")];

            row.state_theta_d = values[column_indices.at("state_theta_d")];
            row.state_beta_d  = values[column_indices.at("state_beta_d")];
            row.state_vel_r_d = values[column_indices.at("state_vel_r_d")];
            row.state_vel_l_d = values[column_indices.at("state_vel_l_d")];
            row.state_trq_r_d = values[column_indices.at("state_trq_r_d")];
            row.state_trq_l_d = values[column_indices.at("state_trq_l_d")];

            // Stage 1.5 camber columns (optional — absent in legacy CSVs)
            auto opt = [&](const char* name, double& dst) {
                auto it = column_indices.find(name);
                if (it != column_indices.end() && it->second < values.size())
                    dst = values[it->second];
            };
            opt("state_gamma_a", row.state_gamma_a);
            opt("state_gamma_b", row.state_gamma_b);
            opt("state_gamma_c", row.state_gamma_c);
            opt("state_gamma_d", row.state_gamma_d);
            opt("state_vel_h_a", row.state_vel_h_a);
            opt("state_vel_h_b", row.state_vel_h_b);
            opt("state_vel_h_c", row.state_vel_h_c);
            opt("state_vel_h_d", row.state_vel_h_d);

            data.push_back(row);
        }

        return data;
    }

private:
    std::map<std::string, size_t> parse_header(const std::string& header) {
        std::map<std::string, size_t> indices;
        std::stringstream ss(header);
        std::string column_name;
        size_t index = 0;
        while (std::getline(ss, column_name, ',')) {
            indices[column_name] = index++;
        }
        return indices;
    }

    std::vector<double> parse_line(const std::string& line) {
        std::vector<double> values;
        values.reserve(42);
        std::stringstream ss(line);
        std::string value_str;
        while (std::getline(ss, value_str, ',')) {
            try {
                values.push_back(std::stod(value_str));
            } catch (...) {
                values.push_back(0.0);
            }
        }
        return values;
    }
};

}  // namespace corgi
