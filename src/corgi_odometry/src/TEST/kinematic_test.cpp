/**
 * @file kinematic_test.cpp
 *
 * Compare two body-velocity estimation methods over an entire CSV run:
 *
 *   Method 1 — Leg::PointVelocity (instantaneous no-slip constraint)
 *     v_body = -PointVelocity(v=0, w_imu)
 *
 *   Method 2 — DP::z()-style sliding-window displacement
 *     d = travel2(window) + compensate2(window, theta_d, dt)
 *         + first_contact_point - last_contact_point_rotated
 *     v = d / (window_size * dt)
 *
 * Usage:
 *   ros2 run corgi_odometry kinematic_test [csv_path] [leg_id] [start_idx] [window_n]
 *   leg_id  : a=LF  b=RF  c=RH  d=LH   (default: a)
 *   start   : CSV row to begin from       (default: 0)
 *   window_n: sliding-window size          (default: 10)
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Config.hpp"
#include "kinematic/ContactMap.hpp"

// ============================================================
// CSV row & reader
// ============================================================
namespace {

struct CsvRow {
    double theta = 0.0;
    double beta  = 0.0;
    double vel_r = 0.0;
    double vel_l = 0.0;
    double w_x   = 0.0;
    double w_y   = 0.0;
    double w_z   = 0.0;
};

class CsvReader {
public:
    std::vector<CsvRow> read(const std::string& filename, char leg_id) {
        std::ifstream file(filename);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file: " + filename);

        std::string header_line;
        if (!std::getline(file, header_line))
            throw std::runtime_error("Empty CSV: " + filename);

        auto header = parse_header(header_line);

        const std::string theta_col = std::string("state_theta_") + leg_id;
        const std::string beta_col  = std::string("state_beta_")  + leg_id;
        const std::string vel_r_col = std::string("state_vel_r_") + leg_id;
        const std::string vel_l_col = std::string("state_vel_l_") + leg_id;

        std::vector<CsvRow> rows;
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            auto values = parse_line(line);

            CsvRow row;
            row.theta = get_value(values, header, theta_col);
            row.beta  = get_value(values, header, beta_col);
            row.vel_r = get_value(values, header, vel_r_col);
            row.vel_l = get_value(values, header, vel_l_col);
            row.w_x   = get_value(values, header, "imu_ang_vel_x");
            row.w_y   = get_value(values, header, "imu_ang_vel_y");
            row.w_z   = get_value(values, header, "imu_ang_vel_z");
            rows.push_back(row);
        }
        return rows;
    }

private:
    std::map<std::string, size_t> parse_header(const std::string& h) {
        std::map<std::string, size_t> idx;
        std::stringstream ss(h);
        std::string col;
        size_t i = 0;
        while (std::getline(ss, col, ',')) idx[col] = i++;
        return idx;
    }

    std::vector<double> parse_line(const std::string& line) {
        std::vector<double> vals;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            try   { vals.push_back(std::stod(tok)); }
            catch (...) { vals.push_back(0.0); }
        }
        return vals;
    }

    double get_value(const std::vector<double>& vals,
                     const std::map<std::string, size_t>& header,
                     const std::string& col) {
        auto it = header.find(col);
        if (it == header.end()) throw std::runtime_error("Missing column: " + col);
        return (it->second < vals.size()) ? vals[it->second] : 0.0;
    }
};

// ============================================================
// Helpers
// ============================================================

Leg create_leg(char leg_id) {
    using corgi::Config;
    double x_sign = 1.0, y_sign = 1.0;
    switch (leg_id) {
        case 'a': x_sign =  1; y_sign =  1; break;  // LF
        case 'b': x_sign =  1; y_sign = -1; break;  // RF
        case 'c': x_sign = -1; y_sign = -1; break;  // RH
        case 'd': x_sign = -1; y_sign =  1; break;  // LH
        default: throw std::runtime_error("Invalid leg_id (use a/b/c/d)");
    }
    return Leg{
        Eigen::Vector3f(x_sign * Config::LEG_X_OFFSET,
                        y_sign * Config::LEG_Y_OFFSET,
                        Config::LEG_Z_OFFSET),
        static_cast<float>(Config::WHEEL_RADIUS),
        static_cast<float>(Config::TIRE_SKIN_RADIUS)
    };
}

bool is_right_side(char leg_id) { return (leg_id == 'b' || leg_id == 'c'); }

const char* rim_name(RIM r) {
    switch (r) {
        case G_POINT:      return "G_PNT";
        case UPPER_RIM_R:  return "UP_R ";
        case LOWER_RIM_R:  return "LO_R ";
        case LOWER_RIM_L:  return "LO_L ";
        case UPPER_RIM_L:  return "UP_L ";
        case NO_CONTACT:   return "NO_CT";
        default:           return "???? ";
    }
}

// ============================================================
// DP::z()-style computation over a sliding window
// d = travel2 + compensate2 + first_point - last_point_rotated
// ============================================================
Eigen::Vector3f compute_dp_z(
    estimation_model::ContactMap& cm,
    std::deque<trajectory>& trajectories,
    std::deque<float>& theta_d_deque,
    Leg& leg,
    float dt)
{
    const int n = static_cast<int>(trajectories.size());

    // Geometric rolling displacement
    Eigen::Vector3f t = cm.travel2(trajectories, leg);

    // Rolling compensation (uses theta_d for RimRoll)
    Eigen::Vector3f c = cm.compensate2(trajectories, leg, theta_d_deque, dt);

    // First & last trajectory entries
    const trajectory& first = trajectories.front();
    const trajectory& last  = trajectories.back();

    RIM first_rim = cm.lookup(std::get<0>(first), std::get<2>(first));
    RIM last_rim  = cm.lookup(std::get<0>(last),  std::get<2>(last));

    // last contact point, rotated into first-frame coordinates
    leg.Calculate(std::get<0>(last), 0, 0, std::get<1>(last), 0, 0);
    leg.PointContact(last_rim, std::get<2>(last) - std::get<1>(last));
    Eigen::Matrix3f last_rot = Eigen::Matrix3f::Identity();
    for (int i = 0; i < n - 1; i++) {
        last_rot = last_rot * (std::get<3>(trajectories[i])).transpose();
    }
    Eigen::Vector3f last_point = last_rot * leg.contact_point;

    // first contact point (already in its own body frame)
    leg.Calculate(std::get<0>(first), 0, 0, std::get<1>(first), 0, 0);
    leg.PointContact(first_rim, std::get<2>(first) - std::get<1>(first));
    Eigen::Vector3f first_point = leg.contact_point;

    // Displacement (without u->dz control term)
    Eigen::Vector3f d = t + first_point - last_point + c;
    return d;
}

}  // anonymous namespace

// ============================================================
// main
// ============================================================
int main(int argc, char** argv) {
    const std::string default_csv =
        "/home/hiho817/corgi_ws/corgi_ros2_ws/output_data/walk_5m_1.csv";
    const std::string csv_path = (argc >= 2) ? argv[1] : default_csv;
    const char leg_id          = (argc >= 3) ? argv[2][0] : 'a';
    const int  start_idx       = (argc >= 4) ? std::atoi(argv[3]) : 0;
    const int  window_n        = (argc >= 5) ? std::atoi(argv[4]) : 10;

    // Output CSV path: same directory as input, named kinematic_result_<leg>.csv
    const std::string out_csv = [&]() {
        auto pos = csv_path.find_last_of('/');
        std::string dir = (pos != std::string::npos) ? csv_path.substr(0, pos + 1) : "./";
        return dir + "kinematic_result_" + leg_id + ".csv";
    }();

    const float dt    = static_cast<float>(corgi::Config::DT);
    const bool  right = is_right_side(leg_id);

    try {
        CsvReader reader;
        auto rows = reader.read(csv_path, leg_id);
        const int total_rows = static_cast<int>(rows.size());
        if (start_idx >= total_rows) {
            std::cerr << "start_idx (" << start_idx
                      << ") >= total rows (" << total_rows << ")\n";
            return 1;
        }

        Leg leg = create_leg(leg_id);
        estimation_model::ContactMap contact_map;

        // Open output CSV
        std::ofstream ofs(out_csv);
        if (!ofs.is_open()) {
            std::cerr << "Cannot open output file: " << out_csv << "\n";
            return 1;
        }
        ofs << std::fixed << std::setprecision(8);
        ofs << "row,theta,beta,theta_d,beta_d,w_y,rim,"
            << "pv_x,pv_y,pv_z,dpz_x,dpz_y,dpz_z\n";

        // State accumulators
        float contact_beta = 0.0f;
        std::deque<trajectory> trajectories;
        std::deque<float> theta_d_deque;

        // Running sums for final averaging
        Eigen::Vector3f sum_pv   = Eigen::Vector3f::Zero();   // PointVelocity
        Eigen::Vector3f sum_dpz  = Eigen::Vector3f::Zero();   // DP::z()-style
        int valid_count = 0;

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "============================================\n";
        std::cout << "CSV       : " << csv_path << "\n";
        std::cout << "Leg       : " << leg_id
                  << " (a=LF, b=RF, c=RH, d=LH)"
                  << (right ? "  [right-side]" : "") << "\n";
        std::cout << "Rows      : " << total_rows
                  << "  start_idx: " << start_idx << "\n";
        std::cout << "Window    : " << window_n
                  << "  dt: " << dt << " s\n";
        std::cout << "============================================\n\n";

        // Column header
        std::cout << std::setw(7)  << "row"
                  << std::setw(10) << "theta"
                  << std::setw(10) << "beta"
                  << std::setw(10) << "theta_d"
                  << std::setw(10) << "beta_d"
                  << std::setw(10) << "w_y"
                  << std::setw(6)  << "rim"
                  << "   PV_vel(x,y,z)"
                  << "                DPz_vel(x,y,z)\n";
        std::cout << std::string(120, '-') << "\n";

        for (int idx = start_idx; idx < total_rows; ++idx) {
            const auto& row = rows[idx];

            float theta = static_cast<float>(row.theta);
            float beta  = static_cast<float>(row.beta);
            float w_x   = static_cast<float>(row.w_x);
            float w_y   = static_cast<float>(row.w_y);
            float w_z   = static_cast<float>(row.w_z);

            // Motor velocities → joint velocities
            float theta_d = static_cast<float>((-row.vel_r + row.vel_l) * 0.5);
            float beta_d  = static_cast<float>(( row.vel_r + row.vel_l) * 0.5);

            if (right) {
                beta   = -beta;
                beta_d = -beta_d;
            }

            // Accumulate contact angle
            contact_beta += (beta_d + w_y) * dt;
            RIM rim = contact_map.lookup(theta, contact_beta);
            float alpha = contact_beta - beta;

            // ----- Method 1: PointVelocity (instantaneous) -----
            leg.Calculate(theta, theta_d, 0.0f, beta, beta_d, 0.0f);
            leg.PointContact(rim, alpha);

            Eigen::Vector3f v_zero = Eigen::Vector3f::Zero();
            Eigen::Vector3f w_vec(w_x, w_y, w_z);
            leg.PointVelocity(v_zero, w_vec, rim, alpha, true);
            Eigen::Vector3f pv_vel = -leg.contact_velocity;

            // ----- Build trajectory for sliding window -----
            Eigen::Matrix3f dR =
                Eigen::AngleAxisf(w_y * dt, Eigen::Vector3f::UnitY())
                    .toRotationMatrix();
            trajectories.emplace_back(theta, beta, contact_beta, dR);
            theta_d_deque.push_back(theta_d);

            // Keep window at size n
            if (static_cast<int>(trajectories.size()) > window_n) {
                trajectories.pop_front();
                theta_d_deque.pop_front();
            }

            // ----- Method 2: DP::z()-style (needs full window) -----
            Eigen::Vector3f dpz_vel = Eigen::Vector3f::Zero();
            bool window_ready = (static_cast<int>(trajectories.size()) == window_n);

            if (window_ready) {
                Eigen::Vector3f d = compute_dp_z(
                    contact_map, trajectories, theta_d_deque, leg, dt);

                float window_time = static_cast<float>(window_n - 1) * dt;
                if (window_time > 0.0f) {
                    dpz_vel = d / window_time;
                }

                sum_pv  += pv_vel;
                sum_dpz += dpz_vel;
                valid_count++;
            }

            // ----- Write to CSV & console -----
            if (window_ready) {
                ofs << idx << ","
                    << theta << "," << beta << ","
                    << theta_d << "," << beta_d << ","
                    << w_y << "," << static_cast<int>(rim) << ","
                    << pv_vel.x() << "," << pv_vel.y() << "," << pv_vel.z() << ","
                    << dpz_vel.x() << "," << dpz_vel.y() << "," << dpz_vel.z() << "\n";
            }

            int row_offset = idx - start_idx;
            bool should_print = (row_offset < 5) ||
                                (row_offset % 1000 == 0) ||
                                (idx == total_rows - 1);
            if (should_print && window_ready) {
                std::cout << std::setw(7)  << idx
                          << std::setw(10) << theta
                          << std::setw(10) << beta
                          << std::setw(10) << theta_d
                          << std::setw(10) << beta_d
                          << std::setw(10) << w_y
                          << std::setw(6)  << rim_name(rim)
                          << "   " << std::setw(10) << pv_vel.x()
                          << " " << std::setw(10) << pv_vel.y()
                          << " " << std::setw(10) << pv_vel.z()
                          << "   " << std::setw(10) << dpz_vel.x()
                          << " " << std::setw(10) << dpz_vel.y()
                          << " " << std::setw(10) << dpz_vel.z()
                          << "\n";
            }
        }

        // ============================================================
        // Summary
        // ============================================================
        if (valid_count > 0) {
            Eigen::Vector3f avg_pv  = sum_pv  / static_cast<float>(valid_count);
            Eigen::Vector3f avg_dpz = sum_dpz / static_cast<float>(valid_count);
            Eigen::Vector3f diff    = avg_pv - avg_dpz;

            std::cout << "\n============================================\n";
            std::cout << "Valid steps compared : " << valid_count << "\n";
            std::cout << "--------------------------------------------\n";
            std::cout << "Avg PointVelocity vel: " << avg_pv.transpose()  << "\n";
            std::cout << "Avg DP::z() velocity : " << avg_dpz.transpose() << "\n";
            std::cout << "Difference (PV - DPz): " << diff.transpose()    << "\n";
            std::cout << "============================================\n";
            std::cout << "Result CSV: " << out_csv << "\n";
        } else {
            std::cout << "\nNo valid windows computed "
                      << "(need at least " << window_n << " rows).\n";
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
