#include <iostream>
#include <array>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_force_control/force_control.hpp"

class ImpedanceCmdPublisherNode : public rclcpp::Node {
public:
    ImpedanceCmdPublisherNode();
    void run();

private:
    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg);
    void initialize_impedance_command();
    void execute_initialization_phase();
    void execute_control_phase();
    void execute_rock_phase();
    
    // ROS2 interfaces
    rclcpp::Publisher<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_pub_;
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    
    // Kinematics helper
    std::unique_ptr<KinematicsHelper> kinematics_;
    
    // Messages
    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules_;

    // Physical parameters
    bool sim_;
    
    // State variables
    bool trigger_;
    double mg_;
    double s_;
    double h_;
    double b_value_;
    double k_value_;
    double b_lateral_;
    double k_lateral_;

    // Force-tracking rock (calibration). "legacy" = the original walk + 6-10 s
    // diagonal rock, unchanged. Any other mode stands still at the init pose
    // and adds rock_amp * sin(2 pi rock_hz t) * pattern[leg] to one force axis.
    std::string rock_mode_;
    double rock_amp_;
    double rock_hz_;
    int rock_cycles_;
    double rock_ramp_s_;
    double rock_hold_s_;
    double stance_pitch_dz_;
};

ImpedanceCmdPublisherNode::ImpedanceCmdPublisherNode()
    : Node("exp_h24"),
    sim_(false),
    trigger_(false),
    mg_(0.0),
    s_(0.0),
    h_(0.0),
    b_value_(0.0),
    k_value_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Impedance Command Publisher Starts");

    this->get_parameter_or("use_sim_time", sim_, false);
    kinematics_ = std::make_unique<KinematicsHelper>(sim_);

    if (sim_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Webots clock...");
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (this->now().seconds() > 0.0) {
                RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Real hardware mode: using system wall clock.");
    }

    const double default_b = sim_ ? 200.0 : 300.0;
    const double default_k = sim_ ? 2000.0 : 4000.0;
    b_value_ = this->declare_parameter<double>("b", default_b);
    k_value_ = this->declare_parameter<double>("k", default_k);
    // Lateral (body y, the ABAD direction) gains on their own; default = b / k.
    // A soft lateral spring lets a commanded fy reach the ground instead of
    // being cancelled by the spring as the leg yields sideways.
    b_lateral_ = this->declare_parameter<double>("b_lateral", b_value_);
    k_lateral_ = this->declare_parameter<double>("k_lateral", k_value_);
    RCLCPP_INFO(this->get_logger(), "Using impedance params: b=%.2f, k=%.2f, lateral b=%.2f, k=%.2f",
                b_value_, k_value_, b_lateral_, k_lateral_);

    // 30.00 kg is the ABAD robot on scales (gslip_pronk.cpp; the proto sums to
    // 30.84 and the sim force plates read 31.0). The old 23.05 was pre-ABAD.
    const double mass_kg = this->declare_parameter<double>("mass_kg", 30.0);
    mg_ = mass_kg * 9.81;
    RCLCPP_INFO(this->get_logger(), "Robot mass %.2f kg (weight %.1f N split over the legs)", mass_kg, mg_);

    rock_mode_ = this->declare_parameter<std::string>("rock_mode", "legacy");
    rock_amp_ = this->declare_parameter<double>("rock_amp", 10.0);
    rock_hz_ = this->declare_parameter<double>("rock_hz", 1.0);
    rock_cycles_ = this->declare_parameter<int>("rock_cycles", 10);
    rock_ramp_s_ = this->declare_parameter<double>("rock_ramp_s", 1.0);
    // sim: the trigger also removes the support box, so the robot settles
    // onto the plates during the first hold -- give it longer there
    rock_hold_s_ = this->declare_parameter<double>("rock_hold_s", 2.0);
    // Pitch the stance for the rock: front feet extend by +dz/2, hind by -dz/2
    // (m, + = nose up), ramped over 1 s starting halfway through the first
    // hold, so the first half is a level reference (in sim, after the support
    // box has gone).
    // 0.045 m over the 0.51 m wheelbase is ~5 deg. For checking anything that
    // uses the body pitch (force_estimation's per-leg pitch sign).
    stance_pitch_dz_ = this->declare_parameter<double>("stance_pitch_dz", 0.0);
    if (rock_mode_ != "legacy" && rock_mode_ != "diag_fz" && rock_mode_ != "pitch_fz" &&
        rock_mode_ != "roll_fz" && rock_mode_ != "fore_fx" && rock_mode_ != "lateral_fy") {
        RCLCPP_ERROR(this->get_logger(), "Unknown rock_mode '%s', using legacy", rock_mode_.c_str());
        rock_mode_ = "legacy";
    }
    if (rock_mode_ != "legacy" && (rock_amp_ < 0.0 || rock_hz_ <= 0.0 || rock_cycles_ < 1 || rock_ramp_s_ < 0.0 || rock_hold_s_ < 0.0)) {
        RCLCPP_ERROR(this->get_logger(), "Bad rock params (amp >= 0, hz > 0, cycles >= 1, ramp >= 0, hold >= 0), using legacy");
        rock_mode_ = "legacy";
    }
    RCLCPP_INFO(this->get_logger(), "rock_mode=%s amp=%.1f N hz=%.2f cycles=%d ramp=%.1f s hold=%.1f s",
                rock_mode_.c_str(), rock_amp_, rock_hz_, rock_cycles_, rock_ramp_s_, rock_hold_s_);
    if (rock_mode_ != "legacy" && stance_pitch_dz_ != 0.0) {
        if (rock_hold_s_ < 3.0) {
            RCLCPP_WARN(this->get_logger(), "stance_pitch_dz needs rock_hold_s >= 3 (level half, 1 s ramp, settle); raising it to 3");
            rock_hold_s_ = 3.0;
        }
        RCLCPP_INFO(this->get_logger(), "stance_pitch_dz=%.3f m (front +dz/2, hind -dz/2)", stance_pitch_dz_);
    }

    imp_cmd_pub_ = this->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000);
    trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 1000, 
        std::bind(&ImpedanceCmdPublisherNode::trigger_cb, this, std::placeholders::_1));
    
    // Initialize module pointers
    imp_cmd_modules_ = {
        &imp_cmd_.module_a,
        &imp_cmd_.module_b,
        &imp_cmd_.module_c,
        &imp_cmd_.module_d
    };
    
    initialize_impedance_command();
}

void ImpedanceCmdPublisherNode::trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    if (msg->enable && !trigger_) {
        RCLCPP_INFO(this->get_logger(), "Trigger received: enable=true");
    }
    trigger_ = msg->enable;
}

void ImpedanceCmdPublisherNode::initialize_impedance_command() {
    for (auto& cmd : imp_cmd_modules_){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->gamma = 0.0;
        cmd->fx = 0;
        cmd->fy = 0;
        cmd->fz = 0;
        cmd->mx = 0;
        cmd->my = 0;
        cmd->mz = 0;
        cmd->bx = b_value_;
        cmd->by = b_lateral_;
        cmd->bz = b_value_;
        cmd->kx = k_value_;
        cmd->ky = k_lateral_;
        cmd->kz = k_value_;
    }
}

void ImpedanceCmdPublisherNode::execute_initialization_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();
    std::array<double, 3> eta;
    bool init_completed = true;

    RCLCPP_INFO(this->get_logger(), "Initialization phase started");

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    for (int i=0; i<2000; i++){
        s_ = 0.12;
        h_ = 0.12;

        eta = legmodel.move_3d(
            imp_cmd_modules_[1]->theta,
            imp_cmd_modules_[1]->beta,
            imp_cmd_modules_[1]->gamma,
            {-s_ / 2000.0, 0.0, h_ / 2000.0}
        );

        imp_cmd_modules_[0]->theta = eta[0];
        imp_cmd_modules_[1]->theta = eta[0];
        imp_cmd_modules_[2]->theta = eta[0];
        imp_cmd_modules_[3]->theta = eta[0];

        // Same beta on all four legs: motor/state and the command are frame-
        // consistent (force_control's 2026-09-02 note; gslip_pronk sends one
        // beta to every leg). The old A/D = -beta mirror was the 2D-era
        // convention and canted the left pair forward and the right pair back
        // (a +/-20 N static fore-aft twist on the force plates).
        imp_cmd_modules_[0]->beta = eta[1];
        imp_cmd_modules_[1]->beta = eta[1];
        imp_cmd_modules_[2]->beta = eta[1];
        imp_cmd_modules_[3]->beta = eta[1];

        imp_cmd_modules_[0]->gamma = eta[2];
        imp_cmd_modules_[1]->gamma = eta[2];
        imp_cmd_modules_[2]->gamma = eta[2];
        imp_cmd_modules_[3]->gamma = eta[2];

        // Front lever arm to the centre of mass. contact_p_3d[0] is positive
        // when the foot is BEHIND the hip for this beta (the force plates read
        // the front legs heavier with the feet back); with the old A/D beta
        // mirror the sign was never exercised.
        legmodel.contact_map_3d(eta[0], eta[1], eta[2]);
        double s_front = 0.255 - legmodel.contact_p_3d[0];
        double f_hind = -mg_/2.0*(s_front/0.510);
        imp_cmd_modules_[0]->fz = -mg_/2.0 - f_hind;
        imp_cmd_modules_[1]->fz = -mg_/2.0 - f_hind;
        imp_cmd_modules_[2]->fz = f_hind;
        imp_cmd_modules_[3]->fz = f_hind;

        imp_cmd_.header.seq = -1;
        imp_cmd_.header.stamp = this->now();

        imp_cmd_pub_->publish(imp_cmd_);

        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            init_completed = false;
            break;
        }
    }

    if (init_completed) {
        RCLCPP_INFO(this->get_logger(), "Initialization phase completed");
    } else {
        RCLCPP_WARN(this->get_logger(), "Initialization phase ended early");
    }
}

void ImpedanceCmdPublisherNode::execute_control_phase() {
    LegModel& legmodel = kinematics_->get_leg_model();
    std::array<double, 3> eta;
    double ds = 0.0;
    
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    int loop_count = 0;
    while (rclcpp::ok()) {
        if (loop_count < 2000) {
            ds = 0.0;
        }
        else if (loop_count < 10000) {
            if (loop_count < 2200) { ds += 2*s_/2000.0/200.0; }
            else if (loop_count < 3800) { ds =   2*s_/2000.0; }
            else if (loop_count < 4000) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 4200) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 5800) { ds =  -2*s_/2000.0; }
            else if (loop_count < 6000) { ds +=  2*s_/2000.0/200.0; }
            else if (loop_count < 6200) { ds +=  2*s_/2000.0/200.0; }
            else if (loop_count < 7800) { ds =   2*s_/2000.0; }
            else if (loop_count < 8000) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 8200) { ds -=  2*s_/2000.0/200.0; }
            else if (loop_count < 9800) { ds =  -2*s_/2000.0; }
            else if (loop_count < 10000) { ds += 2*s_/2000.0/200.0; }

            eta = legmodel.move_3d(
                imp_cmd_modules_[1]->theta,
                imp_cmd_modules_[1]->beta,
                imp_cmd_modules_[1]->gamma,
                {ds, 0.0, 0.0}
            );

            imp_cmd_modules_[0]->theta = eta[0];
            imp_cmd_modules_[1]->theta = eta[0];
            imp_cmd_modules_[2]->theta = eta[0];
            imp_cmd_modules_[3]->theta = eta[0];

            imp_cmd_modules_[0]->beta = eta[1];
            imp_cmd_modules_[1]->beta = eta[1];
            imp_cmd_modules_[2]->beta = eta[1];
            imp_cmd_modules_[3]->beta = eta[1];

            imp_cmd_modules_[0]->gamma = eta[2];
            imp_cmd_modules_[1]->gamma = eta[2];
            imp_cmd_modules_[2]->gamma = eta[2];
            imp_cmd_modules_[3]->gamma = eta[2];

            legmodel.contact_map_3d(eta[0], eta[1], eta[2]);
            double s_front = 0.255 - legmodel.contact_p_3d[0];
            double f_hind = -mg_/2.0*(s_front/0.510);
            imp_cmd_modules_[0]->fz = -mg_/2.0 - f_hind;
            imp_cmd_modules_[1]->fz = -mg_/2.0 - f_hind;
            imp_cmd_modules_[2]->fz = f_hind;
            imp_cmd_modules_[3]->fz = f_hind;

            if (loop_count > 6000 && loop_count < 10000) {
                imp_cmd_modules_[0]->fz += 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[1]->fz -= 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[2]->fz += 10 * sin((loop_count-2000)/500.0*M_PI);
                imp_cmd_modules_[3]->fz -= 10 * sin((loop_count-2000)/500.0*M_PI);
            }
        }
        else {
            break;
        }

        imp_cmd_.header.seq = loop_count;
        imp_cmd_.header.stamp = this->now();

        imp_cmd_pub_->publish(imp_cmd_);

        loop_count++;

        next_time += period;
        if(!this->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
}

// Stand still at the init pose and rock one force axis with a known sinusoid,
// for scoring commanded vs estimated vs force-plate force (force_plate_track.py).
// Patterns are per leg A, B, C, D; each sums to zero, so the body is not pushed
// off its stance, only rocked:
//   diag_fz    + - + -  diagonal load shift (Eason 2025-03, exp_h24 legacy)
//   pitch_fz   + + - -  front vs hind
//   roll_fz    + - - +  A+D side vs B+C side
//   fore_fx    + + - -  front vs hind fore-aft (exp_sim_stay's rock, in 3D axes)
//   lateral_fy + - - +  A+D side vs B+C side lateral, loads the ABAD
// Timeline: rock_hold_s hold, ramp in, rock_cycles full cycles, ramp out, rock_hold_s hold.
void ImpedanceCmdPublisherNode::execute_rock_phase() {
    double pattern[4] = {1.0, 1.0, -1.0, -1.0};
    if (rock_mode_ == "diag_fz") { pattern[1] = -1.0; pattern[2] = 1.0; }
    else if (rock_mode_ == "roll_fz" || rock_mode_ == "lateral_fy") { pattern[1] = -1.0; pattern[3] = 1.0; }

    const int hold_ticks = static_cast<int>(rock_hold_s_ * 1000.0);
    const int ramp_ticks = static_cast<int>(rock_ramp_s_ * 1000.0);
    const int rock_ticks = static_cast<int>(rock_cycles_ * 1000.0 / rock_hz_);
    const int total_ticks = 2 * hold_ticks + 2 * ramp_ticks + rock_ticks;

    // Base fz is the init phase's front/hind split; the pose does not move.
    double base_fz[4];
    for (int i = 0; i < 4; i++) base_fz[i] = imp_cmd_modules_[i]->fz;

    RCLCPP_INFO(this->get_logger(), "Rock phase: %s, %.1f s", rock_mode_.c_str(), total_ticks / 1000.0);
    LegModel& legmodel = kinematics_->get_leg_model();

    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();

    for (int loop_count = 0; loop_count < total_ticks && rclcpp::ok(); loop_count++) {
        // envelope: 0 during holds, raised-cosine ramps, 1 while rocking
        const int t = loop_count - hold_ticks;
        double env = 0.0;
        if (t >= 0 && t < ramp_ticks) env = 0.5 - 0.5 * cos(M_PI * t / ramp_ticks);
        else if (t >= ramp_ticks && t < ramp_ticks + rock_ticks) env = 1.0;
        else if (t >= ramp_ticks + rock_ticks && t < 2 * ramp_ticks + rock_ticks)
            env = 0.5 + 0.5 * cos(M_PI * (t - ramp_ticks - rock_ticks) / ramp_ticks);
        const double u = (t >= 0) ? rock_amp_ * env * sin(2.0 * M_PI * rock_hz_ * t / 1000.0) : 0.0;

        // stance pitch ramp: 1000 equal steps starting halfway through the hold.
        // B carries the front pose and C the hind; same beta on every leg, as
        // in the init phase.
        if (stance_pitch_dz_ != 0.0 && loop_count >= hold_ticks / 2 && loop_count < hold_ticks / 2 + 1000) {
            const double step = stance_pitch_dz_ / 2.0 / 1000.0;
            const std::array<double, 3> front = legmodel.move_3d(
                imp_cmd_modules_[1]->theta, imp_cmd_modules_[1]->beta, imp_cmd_modules_[1]->gamma, {0.0, 0.0, step});
            const std::array<double, 3> hind = legmodel.move_3d(
                imp_cmd_modules_[2]->theta, imp_cmd_modules_[2]->beta, imp_cmd_modules_[2]->gamma, {0.0, 0.0, -step});
            imp_cmd_modules_[0]->theta = front[0]; imp_cmd_modules_[0]->beta =  front[1]; imp_cmd_modules_[0]->gamma = front[2];
            imp_cmd_modules_[1]->theta = front[0]; imp_cmd_modules_[1]->beta =  front[1]; imp_cmd_modules_[1]->gamma = front[2];
            imp_cmd_modules_[2]->theta = hind[0];  imp_cmd_modules_[2]->beta =  hind[1];  imp_cmd_modules_[2]->gamma = hind[2];
            imp_cmd_modules_[3]->theta = hind[0];  imp_cmd_modules_[3]->beta =  hind[1];  imp_cmd_modules_[3]->gamma = hind[2];
        }

        for (int i = 0; i < 4; i++) {
            if (rock_mode_ == "fore_fx") imp_cmd_modules_[i]->fx = pattern[i] * u;
            else if (rock_mode_ == "lateral_fy") imp_cmd_modules_[i]->fy = pattern[i] * u;
            else imp_cmd_modules_[i]->fz = base_fz[i] + pattern[i] * u;
        }

        imp_cmd_.header.seq = loop_count;
        imp_cmd_.header.stamp = this->now();
        imp_cmd_pub_->publish(imp_cmd_);

        next_time += period;
        if (!this->get_clock()->sleep_until(next_time)) {
            RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
            break;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Rock phase finished");
}

void ImpedanceCmdPublisherNode::run() {
    execute_initialization_phase();
    
    rclcpp::Duration period(0, 1000000); // 1ms
    rclcpp::Time next_time = this->now();
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        if (trigger_){
            RCLCPP_INFO(this->get_logger(), "Trigger latched, entering control phase");
            if (rock_mode_ == "legacy") execute_control_phase();
            else execute_rock_phase();
            break;
        }

        next_time += period;
        this->get_clock()->sleep_until(next_time);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImpedanceCmdPublisherNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
