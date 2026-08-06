#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "corgi_hybrid/hybrid_gen.hpp"

namespace
{
constexpr double kInitialTheta = 17.0 * M_PI / 180.0;

struct ExportOptions
{
    std::string output = "/home/chang/corgi_ws/corgi_ros2_ws/input_csv/hybrid_corgi.csv";
    double seconds = 60.0;
    double velocity = 0.08;
    double step_length = 0.30;
    double stand_height = 0.17;
    double gamma = 0.0;
    int swing_index = 1;
    int pub_rate = 1000;
};

double motor_beta_from_eta(int leg_index, double eta_beta)
{
    (void)leg_index;
    return eta_beta;
}

void write_row(std::ofstream &file, const double eta[4][2], double gamma)
{
    file << std::setprecision(17);
    for (int i = 0; i < 4; ++i) {
        if (i > 0) {
            file << ",";
        }
        file << eta[i][0] << "," << motor_beta_from_eta(i, eta[i][1]);
    }
    for (int i = 0; i < 4; ++i) {
        file << "," << gamma;
    }
    file << "\n";
}

void copy_next_eta_to_eta(const std::shared_ptr<GaitSelector> &gait_selector)
{
    for (int i = 0; i < 4; ++i) {
        gait_selector->eta[i][0] = gait_selector->next_eta[i][0];
        gait_selector->eta[i][1] = gait_selector->next_eta[i][1];
    }
}

void print_usage()
{
    std::cout
        << "Usage: ros2 run corgi_hybrid hybrid_csv_export [options]\n"
        << "\n"
        << "Options:\n"
        << "  --output PATH          Output CSV path\n"
        << "  --seconds SEC          Trajectory duration after transform (default 60)\n"
        << "  --velocity MPS         Hybrid body velocity (default 0.08)\n"
        << "  --step-length M        Step length (default 0.30)\n"
        << "  --stand-height M       Stand height for all legs (default 0.17)\n"
        << "  --gamma RAD            Fixed ABAD gamma command (default 0.0)\n"
        << "  --swing-index INDEX    Initial swing index 0..3 (default 1)\n"
        << "  --pub-rate HZ          Sample rate (default 1000)\n";
}

bool parse_args(int argc, char **argv, ExportOptions &options)
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto need_value = [&](const std::string &name) -> char * {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            print_usage();
            return false;
        } else if (arg == "--output") {
            char *v = need_value(arg);
            if (!v) return false;
            options.output = v;
        } else if (arg == "--seconds") {
            char *v = need_value(arg);
            if (!v) return false;
            options.seconds = std::stod(v);
        } else if (arg == "--velocity") {
            char *v = need_value(arg);
            if (!v) return false;
            options.velocity = std::stod(v);
        } else if (arg == "--step-length") {
            char *v = need_value(arg);
            if (!v) return false;
            options.step_length = std::stod(v);
        } else if (arg == "--stand-height") {
            char *v = need_value(arg);
            if (!v) return false;
            options.stand_height = std::stod(v);
        } else if (arg == "--gamma") {
            char *v = need_value(arg);
            if (!v) return false;
            options.gamma = std::stod(v);
        } else if (arg == "--swing-index") {
            char *v = need_value(arg);
            if (!v) return false;
            options.swing_index = std::stoi(v);
        } else if (arg == "--pub-rate") {
            char *v = need_value(arg);
            if (!v) return false;
            options.pub_rate = std::stoi(v);
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage();
            return false;
        }
    }
    return true;
}
}  // namespace

int main(int argc, char **argv)
{
    ExportOptions options;
    if (!parse_args(argc, argv, options)) {
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hybrid_csv_export");

    const bool sim = true;
    const double com_bias = 0.0;
    auto gait_selector = std::make_shared<GaitSelector>(node, sim, com_bias, options.pub_rate);
    Hybrid hybrid(gait_selector);

    gait_selector->velocity = options.velocity;
    gait_selector->step_length = options.step_length;
    gait_selector->new_step_length = options.step_length;
    gait_selector->dS = gait_selector->velocity / gait_selector->pub_rate;
    gait_selector->incre_duty = gait_selector->dS / gait_selector->step_length;

    for (int i = 0; i < 4; ++i) {
        gait_selector->current_stand_height[i] = options.stand_height;
        gait_selector->next_stand_height[i] = options.stand_height;
        gait_selector->current_step_length[i] = options.step_length;
        gait_selector->next_step_length[i] = options.step_length;
        gait_selector->current_shift[i] = 0.0;
    }

    hybrid.Initialize(options.swing_index, 1);

    std::ofstream csv(options.output);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output CSV: " << options.output << "\n";
        rclcpp::shutdown();
        return 1;
    }

    double row_eta[4][2] = {};
    for (int k = 0; k < 5000; ++k) {
        const double ratio = static_cast<double>(k) / 4999.0;
        for (int leg = 0; leg < 4; ++leg) {
            row_eta[leg][0] = kInitialTheta + ratio * (gait_selector->next_eta[leg][0] - kInitialTheta);
            row_eta[leg][1] = ratio * gait_selector->next_eta[leg][1];
        }
        write_row(csv, row_eta, options.gamma);
    }

    copy_next_eta_to_eta(gait_selector);
    hybrid.update_nextFrame();
    gait_selector->body = gait_selector->next_body;
    gait_selector->hip = gait_selector->next_hip;
    gait_selector->foothold = gait_selector->next_foothold;

    const int steps = static_cast<int>(std::round(options.seconds * options.pub_rate));
    for (int k = 0; k < steps; ++k) {
        hybrid.Step();
        write_row(csv, gait_selector->next_eta, options.gamma);
        copy_next_eta_to_eta(gait_selector);
    }

    csv.close();
    std::cout << "Saved " << (5000 + steps) << " rows to " << options.output << "\n"
              << "Transform rows: 5000, trajectory seconds: " << options.seconds << "\n"
              << "velocity=" << options.velocity
              << ", step_length=" << options.step_length
              << ", stand_height=" << options.stand_height
              << ", gamma=" << options.gamma << "\n";

    rclcpp::shutdown();
    return 0;
}
