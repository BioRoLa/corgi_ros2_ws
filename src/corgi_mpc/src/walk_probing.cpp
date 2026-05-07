/**
 * walk_probing.cpp
 *
 * MPC walking controller with per-leg probing state machine.
 *
 * Per-leg state machine (independent for each of the 4 legs):
 *   STANCE ↔ SWING ↔ PROBING → RECOVERY → STANCE
 *
 * STANCE → SWING:   Gait planner commands swing; leg lifts off.
 * SWING  → STANCE:  Swing ends AND contact detected (Fz > F_threshold).
 * SWING  → PROBING: Swing ends BUT no contact (step-miss). Constant-velocity
 *                   downward IK extension with reduced stiffness.
 * PROBING → RECOVERY: Contact detected during probing. Freeze probe position,
 *                     ramp ky from 0 → Ky_stance so MPC warm-starts smoothly.
 * RECOVERY → STANCE: ky ramp complete + short hold for MPC convergence.
 *
 * When any leg is in PROBING or RECOVERY the entire gait freezes (velocity=0)
 * to maintain 3-leg stability. Velocity is ramped back after all legs return
 * to STANCE.
 *
 * Ground truth:  Subscribe to /tf (odom→base_link) for sim velocity estimation.
 */

#include "walk_utils.hpp"
#include "mpc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "corgi_msgs/msg/sim_leg_contact_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// ─── Per-leg state machine ───
enum LegState {
    LEG_STANCE,
    LEG_SWING,
    LEG_PROBING,
    LEG_RECOVERY
};

// ─── Global sensor state (updated via callbacks) ───
bool trigger = false;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::MotorStateStamped motor_state;
geometry_msgs::msg::Vector3 odom_pos;
geometry_msgs::msg::Vector3 odom_vel;
double odom_z;
corgi_msgs::msg::ImuStamped imu;
corgi_msgs::msg::SimLegContactStamped sim_leg_contact;

// Ground-truth pose from /tf
double gt_x = 0.0, gt_y = 0.0, gt_z = 0.0;
double gt_qx = 0.0, gt_qy = 0.0, gt_qz = 0.0, gt_qw = 1.0;

// Sim-only velocity estimate from differentiated GT position (with low-pass filter)
double gt_vx_filt = 0.0, gt_vy_filt = 0.0, gt_vz_filt = 0.0;
double gt_prev_x = 0.0, gt_prev_y = 0.0, gt_prev_z = 0.0;
bool gt_prev_valid = false;

// ─── Callbacks ───
void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg) {
    trigger = msg->enable;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg) {
    force_state = *msg;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
    motor_state = *msg;
}

void odom_pos_cb(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    odom_vel = *msg;
}

void odom_z_cb(const std_msgs::msg::Float64::SharedPtr msg) {
    odom_z = msg->data;
}

void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg) {
    imu = *msg;
}

void sim_leg_contact_cb(const corgi_msgs::msg::SimLegContactStamped::SharedPtr msg) {
    sim_leg_contact = *msg;
}

void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& t : msg->transforms) {
        if (t.header.frame_id == "odom" && t.child_frame_id == "base_link") {
            gt_x = t.transform.translation.x;
            gt_y = t.transform.translation.y;
            gt_z = t.transform.translation.z;
            gt_qx = t.transform.rotation.x;
            gt_qy = t.transform.rotation.y;
            gt_qz = t.transform.rotation.z;
            gt_qw = t.transform.rotation.w;
        }
    }
}

void convert_force_to_local(double *f_global, const Eigen::Matrix3d& R_T) {
    Eigen::Vector3d f_global_vec(f_global[0], f_global[1], f_global[2]);
    Eigen::Vector3d f_local = R_T * f_global_vec;
    f_global[0] = f_local(0);
    f_global[1] = f_local(1);
    f_global[2] = f_local(2);
}

// ─── Contact detection helpers ───
std::vector<corgi_msgs::msg::SimLegContact*> g_sim_contact_modules;
std::vector<corgi_msgs::msg::ForceState*>    g_force_state_modules;
double g_F_threshold = 40.0;

bool detect_contact(int leg_idx, bool use_sim) {
    bool force_contact = std::abs(g_force_state_modules[leg_idx]->fy) > g_F_threshold;
    if (use_sim) {
        bool rim_contact = g_sim_contact_modules[leg_idx]->rim_ll ||
                           g_sim_contact_modules[leg_idx]->rim_lr;
        return rim_contact || force_contact;
    } else {
        return force_contact;
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_mpc");

    node->declare_parameter<std::string>("config_profile", "sim");
    std::string config_profile = node->get_parameter("config_profile").as_string();
    if (config_profile != "sim" && config_profile != "real") {
        RCLCPP_WARN(node->get_logger(),
                    "Invalid config_profile='%s', fallback to 'sim'",
                    config_profile.c_str());
        config_profile = "sim";
    }
    sim = (config_profile == "sim");

    RCLCPP_INFO(node->get_logger(), "Corgi MPC (walk_probing) Starts");
    RCLCPP_INFO(node->get_logger(), "Config profile: %s", config_profile.c_str());

    // Wait for clock synchronization
    RCLCPP_INFO(node->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ModelPredictiveController mpc;
    mpc.load_config(config_profile);
    mpc.target_loop = 2250;

    // ─── Publishers ───
    auto imp_cmd_pub = node->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 10);

    // ─── Subscribers ───
    auto trigger_sub     = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 10, trigger_cb);
    auto force_state_sub = node->create_subscription<corgi_msgs::msg::ForceStateStamped>("force/state", 10, force_state_cb);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 10, motor_state_cb);
    auto odom_pos_sub    = node->create_subscription<geometry_msgs::msg::Vector3>("odometry/legacy/position", 10, odom_pos_cb);
    auto odom_vel_sub    = node->create_subscription<geometry_msgs::msg::Vector3>("odometry/legacy/velocity", 10, odom_vel_cb);
    auto odom_z_sub      = node->create_subscription<std_msgs::msg::Float64>("odometry/legacy/z_position_hip", 10, odom_z_cb);
    auto imu_sub         = node->create_subscription<corgi_msgs::msg::ImuStamped>("imu", 10, imu_cb);
    auto sim_contact_sub = node->create_subscription<corgi_msgs::msg::SimLegContactStamped>("sim/leg_contact", 10, sim_leg_contact_cb);
    auto tf_sub          = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, tf_cb);

    rclcpp::Duration period(0, static_cast<uint32_t>(1000000000.0 / mpc.freq));
    rclcpp::Time next_time = node->now();

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a, &imp_cmd.module_b,
        &imp_cmd.module_c, &imp_cmd.module_d
    };
    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a, &force_state.module_b,
        &force_state.module_c, &force_state.module_d
    };
    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a, &motor_state.module_b,
        &motor_state.module_c, &motor_state.module_d
    };
    std::vector<corgi_msgs::msg::SimLegContact*> sim_contact_modules = {
        &sim_leg_contact.module_a, &sim_leg_contact.module_b,
        &sim_leg_contact.module_c, &sim_leg_contact.module_d
    };

    // Wire up contact detection globals
    g_sim_contact_modules = sim_contact_modules;
    g_force_state_modules = force_state_modules;

    const char* leg_names[4] = {"A", "B", "C", "D"};

    double init_eta[8];
    if (sim) {
        double tmp[8] = {1.3313651941315507, 0.4032814817188362,
                         1.1847611807810603, 0.10626486289107877,
                         1.1847611807810603, -0.10626486289107877,
                         1.3313651941315507, -0.4032814817188362};
        for (int i = 0; i < 8; ++i) init_eta[i] = tmp[i];
    } else {
        double tmp[8] = {1.2744470401482761, 0.4161719979302237,
                         1.1222141023936798, 0.11005079310996896,
                         1.1222141023936798, -0.11005079310996896,
                         1.2744470401482761, -0.4161719979302237};
        for (int i = 0; i < 8; ++i) init_eta[i] = tmp[i];
    }

    mpc.target_pos_z = 0.2;

    WalkGait walk_gait(sim, 0, mpc.freq);
    double velocity = 0.1;

    walk_gait.stand_height = mpc.target_pos_z;
    walk_gait.velocity = velocity;
    walk_gait.step_length = 0.2;
    walk_gait.step_height = 0.06;

    walk_gait.initialize(init_eta, walk_gait.step_length);
    walk_gait.set_velocity(mpc.target_vel_x);

    bool selection_matrix[4] = {true, true, true, true};

    // ─── Per-leg probing state ───
    LegState leg_state[4] = {LEG_STANCE, LEG_STANCE, LEG_STANCE, LEG_STANCE};

    // Probing parameters
    const double probe_velocity    = 0.05;                                  // m/s downward extension
    const double probe_dy          = probe_velocity * mpc.dt;               // m per cycle (≈0.5mm at 100Hz)
    const double Ky_probing        = 200.0;                                 // N/m, reduced stiffness
    const double By_probing        = 100.0;                                 // N·s/m, reduced damping
    const double F_threshold       = 50.0;                                   // N, contact force threshold
    const int    probe_timeout     = static_cast<int>(3.0 * mpc.freq);      // 3s max probing
    const int    recovery_ramp_cycles  = static_cast<int>(0.3 * mpc.freq);  // 300ms ky + force ramp
    const int    recovery_hold_cycles  = static_cast<int>(0.1 * mpc.freq);  // 100ms MPC warm-start hold
    const int    gait_resume_ramp_cycles = static_cast<int>(1.0 * mpc.freq); // 1s velocity ramp

    // Per-leg state variables
    double probe_theta[4] = {0, 0, 0, 0};
    double probe_beta[4]  = {0, 0, 0, 0};
    int    probe_count[4]    = {0, 0, 0, 0};
    int    recovery_count_leg[4] = {0, 0, 0, 0};

    // Grace period for touchdown detection (avoid false step-miss on sensor delay)
    bool pending_touchdown[4] = {false, false, false, false};
    int  pending_countdown[4] = {0, 0, 0, 0};
    const int touchdown_grace_cycles = 5;  // 50ms grace at 100Hz

    // Gait freeze / resume
    double saved_velocity = 0.0;
    bool   gait_frozen    = false;
    int    resume_count   = 0;

    // Initialize impedance command
    for (auto& cmd : imp_cmd_modules) {
        cmd->theta = 17 / 180.0 * M_PI;
        cmd->beta  = 0 / 180.0 * M_PI;
        cmd->fy    = -mpc.m * mpc.gravity / 4.0;

        cmd->mx = mpc.Mx;
        cmd->my = mpc.My;
        cmd->bx = mpc.Bx_swing;
        cmd->by = mpc.By_swing;
        cmd->kx = mpc.Kx_swing;
        cmd->ky = mpc.Ky_swing;
    }

    RCLCPP_INFO(node->get_logger(), "Wait For Force Control Node ...");

    if (!sim) {
        for (int i = 0; i < int(3 * mpc.freq); i++) {
            next_time += period;
            node->get_clock()->sleep_until(next_time);
        }
    }

    // ══════════════ Transform to initial stance ══════════════
    RCLCPP_INFO(node->get_logger(), "Transform Starts");

    for (int i = 0; i < int(3 * mpc.freq); i++) {
        for (int j = 0; j < 4; j++) {
            imp_cmd_modules[j]->theta += (init_eta[2 * j] - 17 / 180.0 * M_PI) / (3 * mpc.freq);
            imp_cmd_modules[j]->beta  += init_eta[2 * j + 1] / (3 * mpc.freq);
        }
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    RCLCPP_INFO(node->get_logger(), "Transform Finished");

    // Stay
    for (int i = 0; i < int(2 * mpc.freq); i++) {
        rclcpp::spin_some(node);
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }

    // ══════════════ Wait for trigger ══════════════
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger) {
            RCLCPP_INFO(node->get_logger(), "Wait For Odometry Node Initializing ...");

            if (!sim) {
                for (int i = 0; i < int(3 * mpc.freq); i++) {
                    rclcpp::spin_some(node);
                    imp_cmd.header.stamp = node->now();
                    imp_cmd_pub->publish(imp_cmd);
                    next_time += period;
                    node->get_clock()->sleep_until(next_time);
                }
            }

            for (auto& cmd : imp_cmd_modules) {
                cmd->bx = mpc.Bx_stance;
                cmd->by = mpc.By_stance;
                cmd->kx = mpc.Kx_stance;
                cmd->ky = mpc.Ky_stance;
            }

            // Sync target height from ground truth to avoid initial height jump
            rclcpp::spin_some(node);
            if (gt_z > 0.01) {
                mpc.target_pos_z = gt_z;
                RCLCPP_INFO(node->get_logger(), "Synced target_pos_z = %.4f from ground truth", gt_z);
            }

            // Initialize GT-based velocity estimator to avoid startup derivative spikes.
            gt_prev_x = gt_x;
            gt_prev_y = gt_y;
            gt_prev_z = gt_z;
            gt_vx_filt = 0.0;
            gt_vy_filt = 0.0;
            gt_vz_filt = 0.0;
            gt_prev_valid = true;

            RCLCPP_INFO(node->get_logger(), "MPC Controller Starts ...");

            // ══════════════════════════════════════════════
            //                 MAIN MPC LOOP
            // ══════════════════════════════════════════════
            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // ────────────────────────────────
                // Check if any leg is in PROBING or RECOVERY
                // ────────────────────────────────
                bool any_probing_or_recovery = false;
                for (int i = 0; i < 4; i++) {
                    if (leg_state[i] == LEG_PROBING || leg_state[i] == LEG_RECOVERY) {
                        any_probing_or_recovery = true;
                        break;
                    }
                }

                // ────────────────────────────────
                // Velocity ramp / gait freeze logic
                // ────────────────────────────────
                if (!any_probing_or_recovery && !gait_frozen) {
                    // Normal operation — velocity ramp at start/end
                    if (loop_count < int(1 * mpc.freq)) {
                        mpc.target_vel_x += velocity / (1 * mpc.freq);
                        walk_gait.set_velocity(mpc.target_vel_x);
                    } else if (loop_count > mpc.target_loop - int(1 * mpc.freq) && loop_count < mpc.target_loop) {
                        mpc.target_vel_x -= velocity / (1 * mpc.freq);
                        walk_gait.set_velocity(mpc.target_vel_x);
                    }
                    mpc.target_pos_x += mpc.target_vel_x * mpc.dt;
                } else if (!any_probing_or_recovery && gait_frozen) {
                    // All legs back to STANCE — ramp velocity back
                    resume_count++;
                    double ramp_vel = saved_velocity * std::min(1.0, static_cast<double>(resume_count) / gait_resume_ramp_cycles);
                    mpc.target_vel_x = ramp_vel;
                    walk_gait.set_velocity(mpc.target_vel_x);
                    mpc.target_pos_x += mpc.target_vel_x * mpc.dt;

                    if (resume_count >= gait_resume_ramp_cycles) {
                        gait_frozen = false;
                        RCLCPP_INFO(node->get_logger(), "Gait fully resumed");
                    }
                }
                // else: gait is frozen (velocity=0, target_pos_x frozen)

                // ────────────────────────────────
                // Get next eta from walk gait
                // ────────────────────────────────
                mpc.eta_list = walk_gait.step();

                // ────────────────────────────────
                // Per-leg state machine
                // ────────────────────────────────
                for (int i = 0; i < 4; i++) {
                    int gait_swing = walk_gait.get_swing_phase()[i];

                    switch (leg_state[i]) {

                    // ═══════════════════════════
                    // LEG_STANCE
                    // ═══════════════════════════
                    case LEG_STANCE:
                        // Apply normal gait eta
                        imp_cmd_modules[i]->theta = mpc.eta_list[0][i];
                        imp_cmd_modules[i]->beta  = (i == 1 || i == 2) ? mpc.eta_list[1][i] : -mpc.eta_list[1][i];

                        if (gait_swing == 1) {
                            // STANCE → SWING
                            leg_state[i] = LEG_SWING;
                            selection_matrix[i] = false;
                            pending_touchdown[i] = false;
                            imp_cmd_modules[i]->by = mpc.By_swing;
                            imp_cmd_modules[i]->ky = mpc.Ky_swing;
                            RCLCPP_INFO(node->get_logger(), "Leg %s: STANCE → SWING", leg_names[i]);
                        }
                        break;

                    // ═══════════════════════════
                    // LEG_SWING
                    // ═══════════════════════════
                    case LEG_SWING:
                        // Apply normal gait eta
                        imp_cmd_modules[i]->theta = mpc.eta_list[0][i];
                        imp_cmd_modules[i]->beta  = (i == 1 || i == 2) ? mpc.eta_list[1][i] : -mpc.eta_list[1][i];

                        if (gait_swing == 0) {
                            // Swing time ended — check contact with grace period
                            if (!pending_touchdown[i]) {
                                pending_touchdown[i] = true;
                                pending_countdown[i] = touchdown_grace_cycles;
                                // Tentatively set stance impedance
                                selection_matrix[i] = true;
                                imp_cmd_modules[i]->by = mpc.By_stance;
                                imp_cmd_modules[i]->ky = mpc.Ky_stance;
                            }

                            if (detect_contact(i, sim)) {
                                // SWING → STANCE (normal touchdown)
                                leg_state[i] = LEG_STANCE;
                                pending_touchdown[i] = false;
                                selection_matrix[i] = true;
                                imp_cmd_modules[i]->by = mpc.By_stance;
                                imp_cmd_modules[i]->ky = mpc.Ky_stance;
                                RCLCPP_INFO(node->get_logger(), "Leg %s: SWING → STANCE (contact OK)", leg_names[i]);
                            } else {
                                pending_countdown[i]--;
                                if (pending_countdown[i] <= 0) {
                                    // SWING → PROBING (step-miss!)
                                    pending_touchdown[i] = false;
                                    leg_state[i] = LEG_PROBING;
                                    probe_count[i] = 0;

                                    RCLCPP_WARN(node->get_logger(),
                                        "=== STEP-MISS on Leg %s → PROBING ===", leg_names[i]);

                                    // Record initial probe position from current gait output
                                    probe_theta[i] = imp_cmd_modules[i]->theta;
                                    probe_beta[i]  = imp_cmd_modules[i]->beta;

                                    // Set probing impedance: low stiffness, no MPC force
                                    selection_matrix[i] = false;
                                    imp_cmd_modules[i]->ky = Ky_probing;
                                    imp_cmd_modules[i]->by = By_probing;
                                    imp_cmd_modules[i]->kx = mpc.Kx_stance;
                                    imp_cmd_modules[i]->bx = mpc.Bx_stance;

                                    // Freeze gait if not already frozen
                                    if (!gait_frozen) {
                                        saved_velocity = mpc.target_vel_x;
                                        mpc.target_vel_x = 0.0;
                                        walk_gait.set_velocity(0.0);
                                        gait_frozen = true;
                                        resume_count = 0;
                                        RCLCPP_INFO(node->get_logger(), "Gait frozen (vel=0)");
                                    }

                                    // Ensure other legs in stance mode
                                    for (int j = 0; j < 4; j++) {
                                        if (j != i && leg_state[j] == LEG_STANCE) {
                                            selection_matrix[j] = true;
                                            imp_cmd_modules[j]->by = mpc.By_stance;
                                            imp_cmd_modules[j]->ky = mpc.Ky_stance;
                                        }
                                    }
                                }
                            }
                        }
                        break;

                    // ═══════════════════════════
                    // LEG_PROBING
                    // ═══════════════════════════
                    case LEG_PROBING: {
                        probe_count[i]++;

                        // Constant-velocity downward IK extension
                        // Get current contact point from probe position (use raw beta for IK)
                        double ik_beta = (i == 1 || i == 2) ? probe_beta[i] : -probe_beta[i];
                        legmodel.contact_map(probe_theta[i], ik_beta);
                        double target_px = legmodel.contact_p[0];
                        double target_py = legmodel.contact_p[1] - probe_dy;  // extend downward

                        auto new_eta = legmodel.inverse({target_px, target_py}, "G");
                        probe_theta[i] = new_eta[0];
                        // Store back in the sign convention used by impedance command
                        probe_beta[i] = (i == 1 || i == 2) ? new_eta[1] : -new_eta[1];

                        // Override impedance command with probe position
                        imp_cmd_modules[i]->theta = probe_theta[i];
                        imp_cmd_modules[i]->beta  = probe_beta[i];

                        // Keep out of MPC, probing impedance
                        selection_matrix[i] = false;
                        imp_cmd_modules[i]->ky = Ky_probing;
                        imp_cmd_modules[i]->by = By_probing;

                        if (detect_contact(i, sim)) {
                            // PROBING → RECOVERY (contact found!)
                            leg_state[i] = LEG_RECOVERY;
                            recovery_count_leg[i] = 0;

                            RCLCPP_INFO(node->get_logger(),
                                "Leg %s: PROBING → RECOVERY after %d cycles (%.2fs)",
                                leg_names[i], probe_count[i],
                                static_cast<double>(probe_count[i]) / mpc.freq);

                            // Add back to MPC with actual foot position; start ky ramp from 0
                            selection_matrix[i] = true;
                            imp_cmd_modules[i]->ky = 0.0;
                            imp_cmd_modules[i]->by = mpc.By_stance;
                        } else if (probe_count[i] >= probe_timeout) {
                            RCLCPP_ERROR(node->get_logger(),
                                "Probe timeout on Leg %s after %.1fs — halting!",
                                leg_names[i], 3.0);
                        }
                        break;
                    }

                    // ═══════════════════════════
                    // LEG_RECOVERY
                    // ═══════════════════════════
                    case LEG_RECOVERY: {
                        recovery_count_leg[i]++;

                        // Hold probe position (actual contact point) — don't use gait output
                        imp_cmd_modules[i]->theta = probe_theta[i];
                        imp_cmd_modules[i]->beta  = probe_beta[i];

                        // Ramp ky from 0 → Ky_stance
                        double ramp_frac = std::min(1.0, static_cast<double>(recovery_count_leg[i]) / recovery_ramp_cycles);
                        imp_cmd_modules[i]->ky = ramp_frac * mpc.Ky_stance;
                        selection_matrix[i] = true;  // MPC uses this leg

                        // Check if ramp + hold complete
                        int total_recovery = recovery_ramp_cycles + recovery_hold_cycles;
                        if (recovery_count_leg[i] >= total_recovery) {
                            // RECOVERY → STANCE
                            leg_state[i] = LEG_STANCE;
                            imp_cmd_modules[i]->ky = mpc.Ky_stance;
                            imp_cmd_modules[i]->by = mpc.By_stance;
                            selection_matrix[i] = true;

                            RCLCPP_INFO(node->get_logger(),
                                "Leg %s: RECOVERY → STANCE (warm-start complete, %.0fms)",
                                leg_names[i], static_cast<double>(recovery_count_leg[i]) / mpc.freq * 1000.0);

                            // Check if all legs back to STANCE — trigger gait resume
                            bool all_stance = true;
                            for (int j = 0; j < 4; j++) {
                                if (leg_state[j] != LEG_STANCE) { all_stance = false; break; }
                            }
                            if (all_stance && gait_frozen) {
                                resume_count = 0;
                                RCLCPP_INFO(node->get_logger(), "All legs STANCE — beginning gait resume ramp");
                            }
                        }
                        break;
                    }

                    } // end switch

                } // end for (per-leg)

                // ────────────────────────────────
                // Update odometry / IMU state
                // ────────────────────────────────
                if (sim) {
                    // Estimate velocity from GT position finite-difference + low-pass filter.
                    if (!gt_prev_valid) {
                        gt_prev_x = gt_x;
                        gt_prev_y = gt_y;
                        gt_prev_z = gt_z;
                        gt_prev_valid = true;
                    }
                    const double raw_vx = (gt_x - gt_prev_x) / mpc.dt;
                    const double raw_vy = (gt_y - gt_prev_y) / mpc.dt;
                    const double raw_vz = (gt_z - gt_prev_z) / mpc.dt;

                    const double vel_lpf_alpha = 0.2;  // 0~1, larger = faster but noisier
                    gt_vx_filt = (1.0 - vel_lpf_alpha) * gt_vx_filt + vel_lpf_alpha * raw_vx;
                    gt_vy_filt = (1.0 - vel_lpf_alpha) * gt_vy_filt + vel_lpf_alpha * raw_vy;
                    gt_vz_filt = (1.0 - vel_lpf_alpha) * gt_vz_filt + vel_lpf_alpha * raw_vz;

                    gt_prev_x = gt_x;
                    gt_prev_y = gt_y;
                    gt_prev_z = gt_z;

                    mpc.robot_vel[0] = gt_vx_filt;
                    mpc.robot_vel[1] = gt_vy_filt;
                    mpc.robot_vel[2] = gt_vz_filt;
                } else {
                    mpc.robot_vel[0] = odom_vel.x;
                    mpc.robot_vel[1] = odom_vel.y;
                    mpc.robot_vel[2] = odom_vel.z;
                }

                mpc.robot_pos[0] = gt_x;
                mpc.robot_pos[1] = gt_y;
                mpc.robot_pos[2] = gt_z;  // Use ground truth z for height tracking

                mpc.robot_ang.x() = gt_qx;
                mpc.robot_ang.y() = gt_qy;
                mpc.robot_ang.z() = gt_qz;
                mpc.robot_ang.w() = gt_qw;

                mpc.robot_ang_vel[0] = imu.angular_velocity.x;
                mpc.robot_ang_vel[1] = imu.angular_velocity.y;
                mpc.robot_ang_vel[2] = imu.angular_velocity.z;

                quaternion_to_euler(mpc.robot_ang, mpc.roll, mpc.pitch, mpc.yaw);

                // ────────────────────────────────
                // MPC solve
                // ────────────────────────────────
                Eigen::VectorXd x(mpc.n_x);
                x << mpc.roll,             mpc.pitch,            mpc.yaw,
                     mpc.robot_pos[0],     mpc.robot_pos[1],     mpc.robot_pos[2],
                     mpc.robot_ang_vel[0], mpc.robot_ang_vel[1], mpc.robot_ang_vel[2],
                     mpc.robot_vel[0],     mpc.robot_vel[1],     mpc.robot_vel[2],
                     -mpc.gravity;

                Eigen::VectorXd x_ref = Eigen::VectorXd::Zero((mpc.N - 1) * mpc.n_x);
                for (int i = 0; i < mpc.N - 1; ++i) {
                    x_ref.segment(i * mpc.n_x, mpc.n_x) << 0,                0,                0,
                                                            mpc.target_pos_x, 0, mpc.target_pos_z,
                                                            0,                0,                0,
                                                            mpc.target_vel_x, 0,                0,
                                                            -mpc.gravity;
                }

                // Leg contact positions in body frame
                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
                double ra[3] = {-legmodel.contact_p[0] + 0.222,  0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
                double rb[3] = { legmodel.contact_p[0] + 0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
                double rc[3] = { legmodel.contact_p[0] - 0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
                double rd[3] = {-legmodel.contact_p[0] - 0.222,  0.2, legmodel.contact_p[1]};

                mpc.init_matrices(ra, rb, rc, rd);

                Eigen::VectorXd force = mpc.step(x, x_ref, selection_matrix, force_state_modules);

                double force_A[3] = {force(0), force(1), force(2)};
                double force_B[3] = {force(3), force(4), force(5)};
                double force_C[3] = {force(6), force(7), force(8)};
                double force_D[3] = {force(9), force(10), force(11)};

                Eigen::Matrix3d R_T = mpc.robot_ang.toRotationMatrix().transpose();
                if (!sim) {
                    R_T(1, 2) *= -1;
                    R_T(2, 1) *= -1;
                }

                convert_force_to_local(force_A, R_T);
                convert_force_to_local(force_B, R_T);
                convert_force_to_local(force_C, R_T);
                convert_force_to_local(force_D, R_T);

                // ────────────────────────────────
                // Assign forces to impedance cmd
                // ────────────────────────────────
                double* force_legs[4] = {force_A, force_B, force_C, force_D};
                // Sign convention: A,D are mirrored in x; all fy = -fz (local z → local y downward)
                imp_cmd_modules[0]->fx = -force_A[0];
                imp_cmd_modules[0]->fy = -force_A[2];
                imp_cmd_modules[1]->fx =  force_B[0];
                imp_cmd_modules[1]->fy = -force_B[2];
                imp_cmd_modules[2]->fx =  force_C[0];
                imp_cmd_modules[2]->fy = -force_C[2];
                imp_cmd_modules[3]->fx = -force_D[0];
                imp_cmd_modules[3]->fy = -force_D[2];

                // During PROBING, zero out force; during RECOVERY, ramp force with ky
                for (int i = 0; i < 4; i++) {
                    if (leg_state[i] == LEG_PROBING) {
                        imp_cmd_modules[i]->fx = 0.0;
                        imp_cmd_modules[i]->fy = 0.0;
                    } else if (leg_state[i] == LEG_RECOVERY) {
                        double force_ramp = std::min(1.0,
                            static_cast<double>(recovery_count_leg[i]) / recovery_ramp_cycles);
                        imp_cmd_modules[i]->fx *= force_ramp;
                        imp_cmd_modules[i]->fy *= force_ramp;
                    }
                }

                // ────────────────────────────────
                // Publish
                // ────────────────────────────────
                imp_cmd.header.stamp = node->now();
                imp_cmd_pub->publish(imp_cmd);

                // ────────────────────────────────
                // Debug output
                // ────────────────────────────────
                std::cout << std::fixed << std::setprecision(3);

                // Per-leg state
                const char* leg_state_str[] = {"STANCE", "SWING", "PROBING", "RECOVERY"};
                std::cout << "LegState [A,B,C,D] = ["
                          << leg_state_str[leg_state[0]] << ", "
                          << leg_state_str[leg_state[1]] << ", "
                          << leg_state_str[leg_state[2]] << ", "
                          << leg_state_str[leg_state[3]] << "]";
                if (gait_frozen) std::cout << "  (gait frozen)";
                std::cout << std::endl;

                std::cout << "Ref Pos = [" << x_ref[3] << ", " << x_ref[4] << ", " << x_ref[5] << "]" << std::endl;
                std::cout << "Odom Pos = [" << mpc.robot_pos[0] << ", " << mpc.robot_pos[1] << ", " << mpc.robot_pos[2] << "]" << std::endl;
                std::cout << "GT   Pos = [" << gt_x << ", " << gt_y << ", " << gt_z << "]" << std::endl;
                std::cout << "GT Error = [" << (gt_x - mpc.robot_pos[0]) << ", "
                          << (gt_y - mpc.robot_pos[1]) << ", "
                          << (gt_z - mpc.robot_pos[2]) << "]" << std::endl;
                std::cout << "Odom Vel = [" << mpc.robot_vel[0] << ", " << mpc.robot_vel[1] << ", " << mpc.robot_vel[2] << "]" << std::endl;
                std::cout << "Odom Ang (deg) = [" << mpc.roll / M_PI * 180 << ", " << mpc.pitch / M_PI * 180 << ", " << mpc.yaw / M_PI * 180 << "]" << std::endl << std::endl;

                std::cout << "Contact [A,B,C,D] = ["
                          << sim_contact_modules[0]->contact << ", "
                          << sim_contact_modules[1]->contact << ", "
                          << sim_contact_modules[2]->contact << ", "
                          << sim_contact_modules[3]->contact << "]" << std::endl;
                std::cout << "Selection [A,B,C,D] = ["
                          << selection_matrix[0] << ", "
                          << selection_matrix[1] << ", "
                          << selection_matrix[2] << ", "
                          << selection_matrix[3] << "]" << std::endl;
                std::cout << "Swing [A,B,C,D] = ["
                          << walk_gait.get_swing_phase()[0] << ", "
                          << walk_gait.get_swing_phase()[1] << ", "
                          << walk_gait.get_swing_phase()[2] << ", "
                          << walk_gait.get_swing_phase()[3] << "]" << std::endl << std::endl;

                std::cout << "Force A: [" << force_A[0] << ", " << force_A[2] << "]  Cmd: [" << imp_cmd_modules[0]->fx << ", " << imp_cmd_modules[0]->fy << "]" << std::endl;
                std::cout << "Force B: [" << force_B[0] << ", " << force_B[2] << "]  Cmd: [" << imp_cmd_modules[1]->fx << ", " << imp_cmd_modules[1]->fy << "]" << std::endl;
                std::cout << "Force C: [" << force_C[0] << ", " << force_C[2] << "]  Cmd: [" << imp_cmd_modules[2]->fx << ", " << imp_cmd_modules[2]->fy << "]" << std::endl;
                std::cout << "Force D: [" << force_D[0] << ", " << force_D[2] << "]  Cmd: [" << imp_cmd_modules[3]->fx << ", " << imp_cmd_modules[3]->fy << "]" << std::endl << std::endl;

                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                if (loop_count >= mpc.target_loop && !gait_frozen)
                    std::cout << "Finished" << std::endl;

                next_time += period;
                if (!node->get_clock()->sleep_until(next_time)) {
                    RCLCPP_WARN(node->get_logger(), "Sleep until failed!");
                    break;
                }
            }
            break;
        }
        next_time += period;
        node->get_clock()->sleep_until(next_time);
    }
    return 0;
}
