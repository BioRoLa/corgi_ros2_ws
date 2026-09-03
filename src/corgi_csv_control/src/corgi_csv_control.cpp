#include <iostream>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

bool trigger = false;

namespace {
constexpr int kCsvColumns = 12;
constexpr int kTransformRows = 5000;
constexpr int64_t kControllerPeriodNanoseconds = 1000000;  // 1 ms / 1 kHz
constexpr int kLegCount = 4;

// Baseline gains. Unchanged from the original controller and used verbatim
// whenever touchdown softening is inactive.
constexpr double kBaseKp = 90.0;
constexpr double kBaseKi = 0.0;
constexpr double kBaseKd = 1.75;

bool parse_csv_row(
    const std::string &line,
    std::array<double, kCsvColumns> &values,
    std::string &error) {
    std::stringstream stream(line);
    std::string item;
    for (int column = 0; column < kCsvColumns; ++column) {
        if (!std::getline(stream, item, ',') || item.empty()) {
            error = "expected 12 numeric columns; missing column " + std::to_string(column);
            return false;
        }
        char *end = nullptr;
        errno = 0;
        const double value = std::strtod(item.c_str(), &end);
        if (errno != 0 || end == item.c_str() || *end != '\0' || !std::isfinite(value)) {
            error = "invalid numeric value in column " + std::to_string(column) + ": '" + item + "'";
            return false;
        }
        values[column] = value;
    }
    if (std::getline(stream, item, ',')) {
        error = "expected exactly 12 columns; found extra data";
        return false;
    }
    return true;
}

// Per-leg gain softening around each touchdown.
//
// Why this exists: the trajectory CSV is an open-loop stream of joint positions
// planned for a perfectly level body. When the body rolls or pitches (a walk with
// a small static stability margin does exactly that as each leg lifts), the
// commanded foot position for the landing leg ends up BELOW the real ground -- a
// 2 deg tilt is already ~7.6 mm of commanded penetration at this robot's geometry.
// Against a stiff position loop that error becomes a large impulsive push on the
// chassis at every touchdown. Briefly lowering kp on the leg that just landed lets
// the error bleed off instead of being converted straight into force.
//
// Timing comes from the "<name>_phase.csv" sidecar written next to the trajectory
// CSV: one row per trajectory row, four columns [FL, FR, RR, RL], 0 = stance,
// 1 = swing. A 1 -> 0 transition is a touchdown. If that file is absent the whole
// mechanism stays off and gains are byte-for-byte the original constants, so every
// CSV that predates this feature behaves exactly as before.
//
// Crucially the sidecar carries the PLANNED schedule, not a measurement: it says
// when the planner intended the foot to land on a level body. The very tilt this
// feature exists to absorb makes the real ground arrive early -- the planned swing
// is still ~11 mm above its landing height 20 ms out, so a 3 deg roll puts first
// contact a full 20 ms before the scheduled instant. Softening only from the
// scheduled row would therefore start after the impact it is meant to absorb, so
// the window opens `lead_ms` early.
class TouchdownSoftener {
public:
    // Blend factor per (row, leg): 1.0 = fully softened, 0.0 = baseline gains.
    // Precomputed at startup so the 1 kHz loop only does an array lookup.
    bool load(
        const std::string &phase_path,
        double lead_ms,
        double hold_ms,
        double ramp_ms,
        std::string &error) {
        std::ifstream file(phase_path);
        if (!file.is_open()) {
            error = "not found";
            return false;
        }

        std::vector<std::array<int, kLegCount>> phase;
        std::string line;
        bool first = true;
        int line_number = 0;
        while (std::getline(file, line)) {
            ++line_number;
            if (line.empty()) {
                continue;
            }
            std::array<int, kLegCount> row{};
            if (!parse_phase_row(line, row)) {
                // The generator writes a "FL_Phase,FR_Phase,..." header; skip it once.
                if (first) {
                    first = false;
                    continue;
                }
                error = "unparseable phase row at line " + std::to_string(line_number);
                return false;
            }
            first = false;
            phase.push_back(row);
        }

        if (phase.empty()) {
            error = "phase file has no data rows";
            return false;
        }

        const int lead_frames = ms_to_frames(lead_ms);
        const int hold_frames = ms_to_frames(hold_ms);
        const int ramp_frames = ms_to_frames(ramp_ms);

        blend_.assign(phase.size(), std::array<float, kLegCount>{});
        touchdown_count_ = 0;
        for (int leg = 0; leg < kLegCount; ++leg) {
            for (size_t row = 1; row < phase.size(); ++row) {
                if (!(phase[row - 1][leg] == 1 && phase[row][leg] == 0)) {
                    continue;
                }
                ++touchdown_count_;
                const int64_t touchdown = static_cast<int64_t>(row);
                // Windows of neighbouring touchdowns can overlap at high duty; keep
                // the softest value rather than letting a later ramp undo an earlier one.
                const int64_t first = touchdown - lead_frames;
                const int64_t last = touchdown + hold_frames + ramp_frames;
                for (int64_t r = std::max<int64_t>(0, first); r < last; ++r) {
                    if (static_cast<size_t>(r) >= phase.size()) {
                        break;
                    }
                    const double value = blend_at(
                        static_cast<int>(r - touchdown), hold_frames, ramp_frames);
                    blend_[r][leg] = std::max(blend_[r][leg], static_cast<float>(value));
                }
            }
        }
        rows_ = phase.size();
        return true;
    }

    bool active() const { return !blend_.empty(); }
    size_t rows() const { return rows_; }
    int touchdown_count() const { return touchdown_count_; }

    // Rows past the end of the phase table fall back to baseline gains rather than
    // clamping, so a short or stale sidecar degrades to the original behaviour.
    std::array<float, kLegCount> blend_at_row(size_t row) const {
        if (row < blend_.size()) {
            return blend_[row];
        }
        return std::array<float, kLegCount>{};
    }

private:
    static bool parse_phase_row(const std::string &line, std::array<int, kLegCount> &row) {
        std::stringstream stream(line);
        std::string item;
        for (int column = 0; column < kLegCount; ++column) {
            if (!std::getline(stream, item, ',') || item.empty()) {
                return false;
            }
            char *end = nullptr;
            errno = 0;
            const double value = std::strtod(item.c_str(), &end);
            if (errno != 0 || end == item.c_str() || *end != '\0' || !std::isfinite(value)) {
                return false;
            }
            row[column] = static_cast<int>(std::lround(value));
        }
        return true;
    }

    static int ms_to_frames(double ms) {
        return static_cast<int>(
            std::lround(std::max(0.0, ms) * 1e6 / kControllerPeriodNanoseconds));
    }

    // offset is relative to the scheduled touchdown row: negative = still swinging
    // (the lead), 0 .. hold = fully soft, then a linear ramp back to the baseline.
    static double blend_at(int offset, int hold_frames, int ramp_frames) {
        if (offset < hold_frames) {
            return 1.0;
        }
        if (ramp_frames <= 0) {
            return 0.0;
        }
        const int into_ramp = offset - hold_frames;
        if (into_ramp >= ramp_frames) {
            return 0.0;
        }
        return 1.0 - static_cast<double>(into_ramp) / static_cast<double>(ramp_frames);
    }

    std::vector<std::array<float, kLegCount>> blend_;
    size_t rows_ = 0;
    int touchdown_count_ = 0;
};

struct SofteningGains {
    double kp_scale = 1.0;    // kp_r / kp_l multiplier at full softening
    double kp_h_scale = 1.0;  // kp_h (ABAD) multiplier at full softening
    double kd_scale = 1.0;    // kd_r / kd_l multiplier at full softening
};

inline double lerp_to_one(double scale_at_full, float blend) {
    return 1.0 + static_cast<double>(blend) * (scale_at_full - 1.0);
}

// blend == nullptr (or all-zero entries) reproduces the original constant gains exactly.
void assign_motor_commands(
    const std::array<double, kCsvColumns> &values,
    const std::vector<corgi_msgs::msg::MotorCmd*> &motor_cmds,
    const std::array<float, kLegCount> *blend,
    const SofteningGains &soft) {
    for (size_t module = 0; module < motor_cmds.size(); ++module) {
        auto *cmd = motor_cmds[module];
        cmd->theta = values[2 * module];
        cmd->beta = values[2 * module + 1];
        cmd->gamma = values[8 + module];

        const float b = (blend != nullptr && module < kLegCount) ? (*blend)[module] : 0.0f;
        const double kp_rl = kBaseKp * lerp_to_one(soft.kp_scale, b);
        const double kp_h = kBaseKp * lerp_to_one(soft.kp_h_scale, b);
        const double kd_rl = kBaseKd * lerp_to_one(soft.kd_scale, b);

        cmd->kp_r = kp_rl;
        cmd->kp_l = kp_rl;
        cmd->kp_h = kp_h;
        cmd->ki_r = kBaseKi;
        cmd->ki_l = kBaseKi;
        cmd->ki_h = kBaseKi;
        cmd->kd_r = kd_rl;
        cmd->kd_l = kd_rl;
        cmd->kd_h = kBaseKd;
    }
}

std::string phase_path_for(const std::string &csv_path) {
    if (csv_path.size() >= 4 && csv_path.substr(csv_path.size() - 4) == ".csv") {
        return csv_path.substr(0, csv_path.size() - 4) + "_phase.csv";
    }
    return csv_path + "_phase.csv";
}
}  // namespace

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_csv_control");

    // Only wait for Webots /clock when running in simulation mode.
    // In real-hardware mode (use_sim_time=false) the system wall clock is used
    // and node->now() is already valid — no need to wait.
    bool use_sim_time = false;
    node->get_parameter_or("use_sim_time", use_sim_time, false);

    // Touchdown softening. Inert unless a "<name>_phase.csv" sidecar sits next to
    // the trajectory CSV, so existing inputs keep the original constant gains.
    const bool softening_enabled = node->declare_parameter<bool>("touchdown_softening", true);
    SofteningGains soft;
    soft.kp_scale = node->declare_parameter<double>("td_kp_scale", 0.35);
    soft.kp_h_scale = node->declare_parameter<double>("td_kp_h_scale", 1.0);
    soft.kd_scale = node->declare_parameter<double>("td_kd_scale", 1.0);
    // Opens the window before the SCHEDULED touchdown, because body tilt makes the
    // real ground arrive early (~20 ms at 3 deg roll for this robot's swing profile).
    const double td_lead_ms = node->declare_parameter<double>("td_lead_ms", 25.0);
    const double td_hold_ms = node->declare_parameter<double>("td_hold_ms", 20.0);
    const double td_ramp_ms = node->declare_parameter<double>("td_ramp_ms", 60.0);

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
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);
    // rclcpp::Rate rate(1000);
    // use_sim_time setting
    rclcpp::Duration period(0, kControllerPeriodNanoseconds);
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmds = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    if (argc < 2){
        RCLCPP_INFO(node->get_logger(), "Please input csv file path\n");
        return 1;
    }
    
    std::string csv_file_path;
    std::string input_arg = argv[1];

    // If absolute/relative path is provided, use it directly
    if (input_arg.find('/') != std::string::npos) {
        csv_file_path = input_arg;
        if (csv_file_path.size() < 4 || csv_file_path.substr(csv_file_path.size() - 4) != ".csv") {
            csv_file_path += ".csv";
        }
    } else {
        csv_file_path = std::getenv("HOME");
        csv_file_path += "/corgi_ws/corgi_ros2_ws/input_csv/";
        csv_file_path += input_arg;
        if (csv_file_path.size() < 4 || csv_file_path.substr(csv_file_path.size() - 4) != ".csv") {
            csv_file_path += ".csv";
        }
    }
    

    std::ifstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        RCLCPP_INFO(node->get_logger(), "Failed to open the CSV file\n");
        return 1;
    }

    TouchdownSoftener softener;
    if (softening_enabled) {
        const std::string phase_path = phase_path_for(csv_file_path);
        std::string phase_error;
        if (softener.load(phase_path, td_lead_ms, td_hold_ms, td_ramp_ms, phase_error)) {
            RCLCPP_INFO(
                node->get_logger(),
                "Touchdown softening ON: %s (%zu rows, %d touchdowns), "
                "kp x%.2f / kp_h x%.2f / kd x%.2f, lead %.0f ms + hold %.0f ms + ramp %.0f ms",
                phase_path.c_str(),
                softener.rows(),
                softener.touchdown_count(),
                soft.kp_scale,
                soft.kp_h_scale,
                soft.kd_scale,
                td_lead_ms,
                td_hold_ms,
                td_ramp_ms);
        } else {
            RCLCPP_INFO(
                node->get_logger(),
                "Touchdown softening OFF (%s: %s); using constant kp=%.0f ki=%.0f kd=%.2f",
                phase_path.c_str(),
                phase_error.c_str(),
                kBaseKp,
                kBaseKi,
                kBaseKd);
        }
    } else {
        RCLCPP_INFO(
            node->get_logger(),
            "Touchdown softening disabled by parameter; using constant kp=%.0f ki=%.0f kd=%.2f",
            kBaseKp,
            kBaseKi,
            kBaseKd);
    }

    std::string line;
    

    RCLCPP_INFO(
        node->get_logger(),
        "Leg Transform Starts: consuming exactly %d rows at 1 kHz before trigger",
        kTransformRows);
    
    int global_row = 0;
    for (int i = 0; i < kTransformRows; ++i) {
        if (!std::getline(csv_file, line)) {
            RCLCPP_ERROR(
                node->get_logger(),
                "CSV ended at row %d during transform; controller requires at least %d transform rows plus trajectory rows",
                i,
                kTransformRows);
            rclcpp::shutdown();
            return 1;
        }
        std::array<double, kCsvColumns> values{};
        std::string parse_error;
        if (!parse_csv_row(line, values, parse_error)) {
            RCLCPP_ERROR(
                node->get_logger(), "Invalid CSV transform row %d: %s", i, parse_error.c_str());
            rclcpp::shutdown();
            return 1;
        }
        const std::array<float, kLegCount> blend = softener.blend_at_row(global_row);
        assign_motor_commands(values, motor_cmds, softener.active() ? &blend : nullptr, soft);
        motor_cmd.header.stamp = node->now();
        motor_cmd.header.seq = 1 - kTransformRows + i;

        motor_cmd_pub->publish(motor_cmd);
        ++global_row;

        next_time += period;
        if(!node->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
            break;
        }
    }
    

    RCLCPP_INFO(node->get_logger(), "Leg Transform Finished\n");

    // A CSV holding only the transform rows (e.g. transform_pose.csv) is a valid
    // pose-only input: warn, then fall through so the trajectory loop simply reads
    // nothing. Treating this as a fatal error would reject files that worked before.
    if (csv_file.peek() == std::ifstream::traits_type::eof()) {
        RCLCPP_WARN(
            node->get_logger(),
            "CSV contains exactly the %d transform rows and no post-trigger trajectory",
            kTransformRows);
    }

    
    while (rclcpp::ok() && !trigger) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "CSV Trajectory Starts\n");

        int seq = 0;
        next_time = node->now();
        int csv_row = kTransformRows;
        bool phase_overrun_warned = false;
        while (rclcpp::ok() && std::getline(csv_file, line)) {
            rclcpp::spin_some(node);

            if (!trigger) {
                RCLCPP_INFO(node->get_logger(), "Trigger False detect, CSV Trajectory pause\n");
                while (rclcpp::ok() && !trigger) {
                    rclcpp::spin_some(node);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
                if (!rclcpp::ok()) {
                    break;
                }
            }

            std::array<double, kCsvColumns> values{};
            std::string parse_error;
            if (!parse_csv_row(line, values, parse_error)) {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "Invalid CSV trajectory row %d: %s",
                    csv_row,
                    parse_error.c_str());
                rclcpp::shutdown();
                return 1;
            }
            if (softener.active() && !phase_overrun_warned &&
                static_cast<size_t>(global_row) >= softener.rows()) {
                RCLCPP_WARN(
                    node->get_logger(),
                    "Phase sidecar ended at row %zu but trajectory continues; "
                    "remaining rows use baseline gains",
                    softener.rows());
                phase_overrun_warned = true;
            }
            const std::array<float, kLegCount> blend = softener.blend_at_row(global_row);
            assign_motor_commands(values, motor_cmds, softener.active() ? &blend : nullptr, soft);

            motor_cmd.header.seq = seq;
            motor_cmd.header.stamp = node->now();

            motor_cmd_pub->publish(motor_cmd);

            seq++;
            csv_row++;
            ++global_row;

            // rate.sleep();
            next_time += period;
            if(!node->get_clock()->sleep_until(next_time)){
                RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                break;
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "CSV Trajectory Finished\n");

    rclcpp::shutdown();
    
    return 0;
}
