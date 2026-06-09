#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_walk/walk_gait_with_contact.hpp"
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

WalkGaitWithContact::WalkGaitWithContact(bool sim, double CoM_bias, int rate, double BL, double BW, double BH) : /* Initializer List */
                                                                                           leg_model(sim),
                                                                                           BL(BL),
                                                                                           BW(BW),
                                                                                           BH(BH),
                                                                                           CoM_bias(CoM_bias),
                                                                                           rate(rate)
{
    // Initial dS & incre_duty
    dS = velocity / rate;
    incre_duty = dS / step_length;
} // end WalkGaitWithContact

void WalkGaitWithContact::initialize(double init_eta[8], double step_length_)
{
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4] = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    int current_rim = 0;
    swing_phase = {0, 0, 0, 0};
    step_count = {0, 0, 0, 0};
    step_length = step_length_;
    new_step_length = step_length;
    incre_duty = dS / step_length;
    for (int i = 0; i < 4; i++)
    {
        leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = leg_model.rim;
        leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1)
        {
            relative_foothold[i][0] = leg_model.U_l[0];
        }
        else if (current_rim == 2)
        {
            relative_foothold[i][0] = leg_model.L_l[0];
        }
        else if (current_rim == 3)
        {
            relative_foothold[i][0] = leg_model.G[0];
        }
        else if (current_rim == 4)
        {
            relative_foothold[i][0] = leg_model.L_r[0];
        }
        else if (current_rim == 5)
        {
            relative_foothold[i][0] = leg_model.U_r[0];
        }
        else
        {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        } // end if else
        double stand_height_eff = stand_height - ground_offset[i];
        relative_foothold[i][1] = -stand_height_eff;
    } // end for
    // Get initial leg duty
    int first_swing_leg = 0;
    for (int i = 1; i < 4; i++)
    {
        if (relative_foothold[i][0] < relative_foothold[first_swing_leg][0])
        {
            first_swing_leg = i;
        } // end if
    } // end for
    if (first_swing_leg == 0)
    {
        duty = {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
    }
    else if (first_swing_leg == 1)
    {
        duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
    }
    else if (first_swing_leg == 2)
    {
        duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
    }
    else if (first_swing_leg == 3)
    {
        duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
    } // end if else
    // Get foothold in world coordinate
    hip = {{{BL / 2, stand_height},
            {BL / 2, stand_height},
            {-BL / 2, stand_height},
            {-BL / 2, stand_height}}};
    next_hip = hip;
    // Initial leg configuration
    for (int i = 0; i < 4; i++)
    {
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0], next_hip[i][1] + relative_foothold[i][1]};
        touchdown_point[i] = foothold[i];
        current_step_length[i] = step_length;
        next_step_length[i] = step_length;
    } // end for
    // Initial theta/beta
    for (int i = 0; i < 4; i++)
    {
        theta[i] = init_theta[i];
        beta[i] = init_beta[i];
        leg_model.forward(theta[i], beta[i]);
        foot_point[i] = {hip[i][0] + leg_model.G[0], hip[i][1] + leg_model.G[1]};
        touchdown_point[i] = foot_point[i];
    } // end for
} // end initialize

std::array<std::array<double, 4>, 2> WalkGaitWithContact::step()
{
    // 檢查是否有任何腳正在探測，如果有，則凍結時間 (duty) 與水平位移
    bool any_probing = false;
    for (int i = 0; i < 4; i++) {
        if (late_probing[i]) {
            any_probing = true;
            break;
        }
    }

    touchdown = false;
    touchdown_leg = {false, false, false, false};
    for (int i = 0; i < 4; i++)
    {
        if (!any_probing) { // 只有在沒有腳處於探測狀態時，才推進時間與水平位置
            next_hip[i][0] += dS + sign_diff[i] * diff_dS;
            duty[i] += incre_duty;
        }
    } // end for
    for (int i = 0; i < 4; i++)
    {
        bool entered_stance = false;
        /* Keep duty in the range [0, 1] */
        if (duty[i] < 0)
        {
            duty[i] += 1.0;
        } // end if
        /* Calculate next foothold if entering swing phase */
        if ((duty[i] > (1 - swing_time)) && swing_phase[i] == 0)
        {
            swing_phase[i] = 1;
            double total_step_length; // step length considering differential
            double swing_hip_move_d;  // hip moving distance during swing phase
            // change to new step length when front leg start to swing
            if (((direction == 1) && (i == 0 || i == 1)) || ((direction == -1) && (i == 2 || i == 3)))
            { // front leg swing
                // apply new step length and differential
                next_step_length[i] = new_step_length;
                double rest_time = (1.0 - 4 * swing_time) / 2; // time during swing of front leg and next hind leg
                total_step_length = step_length + sign_diff[i] * diff_step_length;
                swing_hip_move_d = direction * swing_time * total_step_length;
                foothold[i] = {next_hip[i][0] + direction * ((1 - swing_time) / 2) * (new_step_length + sign_diff[i] * new_diff_step_length) + swing_hip_move_d + direction * (rest_time * (step_length - new_step_length)) + CoM_bias, 0.0}; 
                diff_step_length = new_diff_step_length;
            }
            else
            {                               // hind leg swing
                int last_leg = (i + 2) % 4; // Contralateral front leg
                step_length = current_step_length[last_leg];
                next_step_length[i] = step_length; // apply hind step length corresponding to the front leg's.
                total_step_length = step_length + sign_diff[i] * diff_step_length;
                swing_hip_move_d = direction * swing_time * total_step_length;
                foothold[i] = {next_hip[i][0] + direction * ((1 - swing_time) / 2) * total_step_length + swing_hip_move_d + CoM_bias, 0.0};
                incre_duty = dS / step_length; // change incre_duty corresponding to new step length when hind leg start to swing.
            } // end if else
            /* Bezier curve setup */
            leg_model.forward(theta[i], beta[i]);
            p_lo = {next_hip[i][0] + leg_model.G[0], next_hip[i][1] + leg_model.G[1]};
            
            // calculate contact rim when touch ground
            for (int j = 0; j < 5; j++)
            { // G, L_l, U_l
                double contact_height = j == 0 ? leg_model.r : leg_model.radius;
                // 恢復使用預設水平基準高度，確保 bezier 能正常規劃，並防止平地越走越低
                double stand_height_eff = stand_height - ground_offset[i];
                std::array<double, 2> contact_point = {foothold[i][0] - (next_hip[i][0] + swing_hip_move_d), -stand_height_eff + contact_height};
                result_eta = leg_model.inverse(contact_point, touch_rim_list[j]);
                leg_model.contact_map(result_eta[0], result_eta[1]);
                if (leg_model.rim == touch_rim_idx[j])
                {
                    current_rim = leg_model.rim;
                    break;
                } // end if
            } // end for
            // G position when touch ground
            leg_model.forward(result_eta[0], result_eta[1]);
            if (current_rim == 3)
            { // G
                p_td = {foothold[i][0], foothold[i][1] + leg_model.r};
            }
            else if (current_rim == 2)
            { // L_l
                p_td = {foothold[i][0] + leg_model.G[0] - leg_model.L_l[0], foothold[i][1] + leg_model.G[1] - leg_model.L_l[1] + leg_model.radius};
            }
            else if (current_rim == 4)
            { // L_r
                p_td = {foothold[i][0] + leg_model.G[0] - leg_model.L_r[0], foothold[i][1] + leg_model.G[1] - leg_model.L_r[1] + leg_model.radius};
            }
            else if (current_rim == 1)
            { // U_l
                p_td = {foothold[i][0] + leg_model.G[0] - leg_model.U_l[0], foothold[i][1] + leg_model.G[1] - leg_model.U_l[1] + leg_model.radius};
            }
            else if (current_rim == 5)
            { // U_r
                p_td = {foothold[i][0] + leg_model.G[0] - leg_model.U_r[0], foothold[i][1] + leg_model.G[1] - leg_model.U_r[1] + leg_model.radius};
            } // end if else
            touchdown_point[i] = p_td;
            sp[i] = SwingProfile(p_lo, p_td, step_height, direction);
        }
        else if ((direction == 1) && (duty[i] > 1.0))
        { // entering stance phase when velocirty > 0
            touchdown = true;
            touchdown_leg[i] = true;
            swing_phase[i] = 0;
            entered_stance = true;
            early_contact[i] = false;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
            if (sp[i].getDirection() == direction)
            { // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] += 1;
                current_step_length[i] = next_step_length[i];
            } // end if
        }
        else if ((direction == -1) && (duty[i] < (1.0 - swing_time)))
        { // entering stance phase when velocirty < 0
            touchdown = true;
            touchdown_leg[i] = true;
            swing_phase[i] = 0;
            entered_stance = true;
            early_contact[i] = false;
            if (sp[i].getDirection() == direction)
            { // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] -= 1;
                current_step_length[i] = next_step_length[i];
            } // end if
        } // end if else

        double current_swing_phase_ratio = 0.0;
        if (swing_phase[i] != 0)
        {
            if (sp[i].getDirection() == 1)
            {
                current_swing_phase_ratio = (duty[i] - (1 - swing_time)) / swing_time;
            }
            else
            {
                current_swing_phase_ratio = (1.0 - duty[i]) / swing_time;
            } // end if else
        }
        const bool leg_contact = update_contact_filter(i, current_swing_phase_ratio);
        if (entered_stance && !leg_contact)
        {
            late_probing[i] = true;
        }

        /* Calculate next theta, beta */
        if (swing_phase[i] == 0)
        { // Stance phase
            if (late_probing[i]) {
                if (leg_contact) {
                    late_probing[i] = false; // 觸地，結束下探
                    result_eta = leg_model.move(theta[i], beta[i], {next_hip[i][0] - hip[i][0], next_hip[i][1] - hip[i][1]});
                } else {
                    double probe_dz = probe_speed / rate; // 強迫足端繼續等速下降
                    result_eta = leg_model.move(theta[i], beta[i], {next_hip[i][0] - hip[i][0], (next_hip[i][1] - hip[i][1]) + probe_dz});
                }
            } else {
                result_eta = leg_model.move(theta[i], beta[i], {next_hip[i][0] - hip[i][0], next_hip[i][1] - hip[i][1]});
            }
        }
        else
        { // Swing phase
            swing_phase_ratio = current_swing_phase_ratio;
            // 如果在擺動下半段偵測到觸地，觸發提早觸地反射
            if (swing_phase_ratio > 0.5 && leg_contact) {
                early_contact[i] = true;
            }
            
            if (early_contact[i]) {
                // 放棄 Bezier 軌跡，瞬間鎖定當前高度（相當於直接執行 Stance 動作）
                result_eta = leg_model.move(theta[i], beta[i], {next_hip[i][0] - hip[i][0], next_hip[i][1] - hip[i][1]});
            } else {
                curve_point_temp = sp[i].getFootendPoint(swing_phase_ratio);
                std::array<double, 2> curve_point = {curve_point_temp[0] - next_hip[i][0], curve_point_temp[1] - next_hip[i][1]};
                result_eta = leg_model.inverse(curve_point, "G");
            }
        } // end if else
        theta[i] = result_eta[0];
        beta[i] = result_eta[1];
        leg_model.forward(theta[i], beta[i]);
        foot_point[i] = {next_hip[i][0] + leg_model.G[0], next_hip[i][1] + leg_model.G[1]};
        hip[i] = next_hip[i];
    } // end for
    return {theta, beta};
} // end step

void WalkGaitWithContact::set_velocity(double new_value)
{
    if (std::abs(new_value) > 0.5)
    {
        throw std::runtime_error("Velocity should not exceed 0.5 m/s.");
    } // end if
    velocity = new_value;
    dS = velocity / rate;
    incre_duty = dS / step_length;
    direction = velocity >= 0 ? 1 : -1;
    // change differential dS if the robot is turning
    if (curvature != 0.0)
    {
        diff_dS = dS * (outer_radius - inner_radius) / (outer_radius + inner_radius); // apply new value immediately
    } // end if
} // end set_velocity

void WalkGaitWithContact::set_stand_height(double new_value)
{
    if (new_value > 0.34)
    {
        throw std::runtime_error("Stand height should be no larger than 0.34.");
    } // end if
    for (int i = 0; i < 4; i++)
    {
        if (new_value - ground_offset[i] < 0.12 + step_height)
        {
            throw std::runtime_error("Effective stand height should be >= 0.12 + step_height for all legs.");
        }
    }
    stand_height = new_value;
    for (int i = 0; i < 4; i++)
    {
        next_hip[i][1] = stand_height;
    } // end for
} // end set_stand_height

void WalkGaitWithContact::set_ground_offset(const std::array<double, 4>& new_value)
{
    for (int i = 0; i < 4; i++)
    {
        if (stand_height - new_value[i] < 0.12 + step_height)
        {
            throw std::runtime_error("Effective stand height should be >= 0.12 + step_height for all legs.");
        }
    }
    ground_offset = new_value;
} // end set_ground_offset

void WalkGaitWithContact::set_step_length(double new_value)
{
    if (new_value <= 0.0)
    {
        throw std::runtime_error("Step length should be larger than zero.");
    } // end if
    new_step_length = new_value;
    // change differential step length if the robot is turning
    if (curvature != 0.0)
    {
        new_diff_step_length = new_step_length * (outer_radius - inner_radius) / (outer_radius + inner_radius); // apply new value when front leg swing
    } // end if
} // end set_step_length

void WalkGaitWithContact::set_step_height(double new_value)
{
    if (new_value <= 0.0)
    {
        throw std::runtime_error("Step height should be larger than zero.");
    } // end if
    for (int i = 0; i < 4; i++)
    {
        if (stand_height - ground_offset[i] < 0.12 + new_value)
        {
            throw std::runtime_error("Effective stand height should be >= 0.12 + step_height for all legs.");
        }
    }
    step_height = new_value;
} // end set_step_height

void WalkGaitWithContact::set_curvature(double new_value)
{
    curvature = new_value;
    if (curvature == 0.0)
    {
        new_diff_step_length = 0.0;
        diff_dS = 0.0;
    }
    else
    {
        double turn_radius = 1.0 / std::abs(curvature);
        outer_radius = turn_radius + BW / 2.0;
        inner_radius = turn_radius - BW / 2.0;
        /*
        step_length + d : step_length - d  = outer_radius : inner_radius
        (step_length - d) * outer_radius = (step_length + d) * inner_radius
        d * (outer_radius + inner_radius) = step_length * (outer_radius - inner_radius)
        */
        new_diff_step_length = new_step_length * (outer_radius - inner_radius) / (outer_radius + inner_radius); // apply new value when front leg swing
        diff_dS = dS * (outer_radius - inner_radius) / (outer_radius + inner_radius);                           // apply new value immediately
        // determine increase/decrease of differential according to sign of curvature and left/right leg
        if (curvature > 0.0)
        { // turn left
            sign_diff[0] = -1;
            sign_diff[1] = 1;
            sign_diff[2] = 1;
            sign_diff[3] = -1;
        }
        else
        { // turn right
            sign_diff[0] = 1;
            sign_diff[1] = -1;
            sign_diff[2] = -1;
            sign_diff[3] = 1;
        } // end if else
    } // end if else
} // end set_curvature

void WalkGaitWithContact::set_eta(std::array<std::array<double, 4>, 2> eta_)
{
    this->theta = eta_[0];
    this->beta = eta_[1];
} // end set_eta

void WalkGaitWithContact::set_duty(std::array<double, 4> duty_)
{
    for (int i = 0; i < 4; i++)
    {
        if (duty_[i] < 0 || duty_[i] > 1)
        {
            throw std::runtime_error("Duty should be in the range [0, 1].");
        } // end if
    } // end for
    this->duty = duty_;
} // end set_duty

void WalkGaitWithContact::set_probe_speed(double new_value)
{
    if (new_value < 0.0)
    {
        throw std::runtime_error("Probe speed should be a non-negative value.");
    } // end if
    this->probe_speed = new_value;
} // end set_probe_speed

void WalkGaitWithContact::set_contact_state(const std::array<bool, 4>& contact)
{
    this->raw_contact_state = contact;
    if (!contact_filter_enabled)
    {
        this->contact_state = contact;
    }
} // end set_contact_state

void WalkGaitWithContact::set_contact_filter(bool enabled, double swing_accept_ratio, int on_count, int off_count)
{
    if (swing_accept_ratio < 0.0 || swing_accept_ratio > 1.0)
    {
        throw std::runtime_error("Contact filter swing_accept_ratio should be in the range [0, 1].");
    }
    if (on_count <= 0 || off_count <= 0)
    {
        throw std::runtime_error("Contact filter debounce counts should be positive.");
    }
    contact_filter_enabled = enabled;
    contact_swing_accept_ratio = swing_accept_ratio;
    contact_on_count_threshold = on_count;
    contact_off_count_threshold = off_count;
    contact_on_count = {0, 0, 0, 0};
    contact_off_count = {0, 0, 0, 0};
    contact_state = raw_contact_state;
} // end set_contact_filter

bool WalkGaitWithContact::update_contact_filter(int leg_idx, double current_swing_phase_ratio)
{
    if (!contact_filter_enabled)
    {
        contact_state[leg_idx] = raw_contact_state[leg_idx];
        return contact_state[leg_idx];
    }

    if (swing_phase[leg_idx] != 0 && current_swing_phase_ratio < contact_swing_accept_ratio)
    {
        contact_state[leg_idx] = false;
        contact_on_count[leg_idx] = 0;
        contact_off_count[leg_idx] = 0;
        return false;
    }

    if (raw_contact_state[leg_idx])
    {
        contact_on_count[leg_idx]++;
        contact_off_count[leg_idx] = 0;
    }
    else
    {
        contact_on_count[leg_idx] = 0;
        contact_off_count[leg_idx]++;
    }

    if (contact_state[leg_idx])
    {
        if (contact_off_count[leg_idx] >= contact_off_count_threshold)
        {
            contact_state[leg_idx] = false;
        }
    }
    else if (contact_on_count[leg_idx] >= contact_on_count_threshold)
    {
        contact_state[leg_idx] = true;
    }

    return contact_state[leg_idx];
} // end update_contact_filter

std::array<int, 4> WalkGaitWithContact::get_step_count()
{
    return this->step_count;
} // end get_step_count

std::array<int, 4> WalkGaitWithContact::get_swing_phase()
{
    return this->swing_phase;
} // end get_swing_phase

std::array<double, 4> WalkGaitWithContact::get_duty()
{
    return this->duty;
} // end get_duty

bool WalkGaitWithContact::if_touchdown()
{
    return this->touchdown;
} // end if_touchdown

WalkGaitWithContact::DebugState WalkGaitWithContact::get_debug_state() const
{
    return DebugState{
        foothold,
        touchdown_point,
        foot_point,
        hip,
        next_hip,
        swing_phase,
        touchdown_leg,
        contact_state,
        early_contact,
        late_probing};
} // end get_debug_state
