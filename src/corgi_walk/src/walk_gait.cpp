#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_walk/walk_gait.hpp"
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

WalkGait::WalkGait(bool sim, double CoM_bias, int rate, double BL, double BW, double BH) : /* Initializer List */
                                                                                           leg_model(sim),
                                                                                           CoM_bias(CoM_bias),
                                                                                           rate(rate),
                                                                                           BL(BL),
                                                                                           BW(BW),
                                                                                           BH(BH)
{
    // Initial dS & incre_duty
    dS = velocity / rate;
    incre_duty = dS / step_length;
    // Farthest the foot arc centre can be from the hip (5 mm short of full extension).
    leg_model.forward(leg_model.max_theta, 0.0);
    max_reach = std::hypot(leg_model.G[0], leg_model.G[1]) - leg_model.R - 0.005;
} // end WalkGait

double WalkGait::beta_limit(double stand_height_)
{
    double H_O = stand_height_ - leg_model.foot_radius; // hip above the foot arc centre
    if (H_O <= 0.0)
    {
        return 0.0;
    } // end if
    return H_O >= max_reach ? 0.0 : std::min(max_beta, std::acos(H_O / max_reach));
} // end beta_limit

double WalkGait::max_step_length(double stand_height_)
{
    double H_O = stand_height_ - leg_model.foot_radius;
    double b = beta_limit(stand_height_);
    return (2.0 * H_O * std::tan(b) + 2.0 * leg_model.foot_radius * b) / (1.0 - swing_time);
} // end max_step_length

double WalkGait::stance_beta(double stance_travel)
{
    // While the hip travels stance_travel, the leg sweeps from -b to +b: the
    // contact point shifts by H_O*tan(b) on each side of the hip and the foot
    // arc rolls R_arc*b on each side (LegWheel TrajectoryPlanner3D).
    double H_O = stand_height - leg_model.foot_radius;
    double R_arc = leg_model.foot_radius;
    double b_max = beta_limit(stand_height);
    double b = std::min(b_max, stance_travel / (2.0 * (H_O + R_arc)));
    for (int iter = 0; iter < 20; iter++)
    {
        double f = 2.0 * H_O * std::tan(b) + 2.0 * R_arc * b - stance_travel;
        double df = 2.0 * H_O / (std::cos(b) * std::cos(b)) + 2.0 * R_arc;
        double step = f / df;
        b -= step;
        if (std::abs(step) < 1e-12)
        {
            break;
        } // end if
    } // end for
    return std::max(0.0, std::min(b, b_max));
} // end stance_beta

double WalkGait::touchdown_lead(double stance_travel)
{
    return (stand_height - leg_model.foot_radius) * std::tan(stance_beta(stance_travel));
} // end touchdown_lead

std::array<double, 8> WalkGait::get_init_eta(double step_length_)
{
    double lead = touchdown_lead((1 - swing_time) * step_length_);
    // Stance progress of each leg when module A is about to swing (see initialize).
    double init_duty[4] = {1 - swing_time, 0.5 - swing_time, 0.75 - swing_time, 0.25 - swing_time};
    std::array<double, 8> init_eta;
    for (int i = 0; i < 4; i++)
    {
        double foot_x = lead * (1.0 - 2.0 * init_duty[i] / (1 - swing_time));
        std::array<double, 2> eta = leg_model.inverse(std::array<double, 2>{foot_x, -(stand_height - leg_model.foot_radius)}, "O_r");
        init_eta[2 * i] = eta[0];
        init_eta[2 * i + 1] = (i == 1 || i == 2) ? eta[1] : -eta[1];
    } // end for
    return init_eta;
} // end get_init_eta

void WalkGait::initialize(double init_eta[8], double step_length_)
{
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4] = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    swing_phase = {0, 0, 0, 0};
    step_count = {0, 0, 0, 0};
    step_length = step_length_;
    new_step_length = step_length;
    incre_duty = dS / step_length;
    for (int i = 0; i < 4; i++)
    {
        // The foot arc (rim 1) is the only rim this gait plans contact on.
        leg_model.contact_map_3d(init_theta[i], init_beta[i], 0.0);
        if (leg_model.rim != 1)
        {
            std::cout << "Leg " << i << " does not stand on its foot arc with the given initial theta/beta." << std::endl;
        } // end if
        relative_foothold[i][0] = leg_model.contact_p[0];
        relative_foothold[i][1] = -stand_height;
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
    // Legs lift off a quarter cycle apart (LegWheel Walk phase offsets).
    if (first_swing_leg == 0)
    {
        duty = {1 - swing_time, 0.5 - swing_time, 0.75 - swing_time, 0.25 - swing_time};
    }
    else if (first_swing_leg == 1)
    {
        duty = {0.5 - swing_time, 1 - swing_time, 0.25 - swing_time, 0.75 - swing_time};
    }
    else if (first_swing_leg == 2)
    {
        duty = {0.25 - swing_time, 0.75 - swing_time, 1 - swing_time, 0.5 - swing_time};
    }
    else if (first_swing_leg == 3)
    {
        duty = {0.75 - swing_time, 0.25 - swing_time, 0.5 - swing_time, 1 - swing_time};
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
        current_step_length[i] = step_length;
        next_step_length[i] = step_length;
    } // end for
    // Initial theta/beta
    for (int i = 0; i < 4; i++)
    {
        theta[i] = init_theta[i];
        beta[i] = init_beta[i];
        prev_eta[i] = {init_theta[i], init_beta[i]};
    } // end for
} // end initialize

std::array<std::array<double, 4>, 2> WalkGait::step()
{
    touchdown = false;
    for (int i = 0; i < 4; i++)
    {
        next_hip[i][0] += dS + sign_diff[i] * diff_dS;
        duty[i] += incre_duty;
    } // end for
    for (int i = 0; i < 4; i++)
    {
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
                double rest_time = 0.25 - swing_time; // time between front leg touchdown and next hind leg lift-off
                total_step_length = step_length + sign_diff[i] * diff_step_length;
                swing_hip_move_d = direction * swing_time * total_step_length;
                foothold[i] = {next_hip[i][0] + direction * touchdown_lead((1 - swing_time) * (new_step_length + sign_diff[i] * new_diff_step_length)) + swing_hip_move_d + direction * (rest_time * (step_length - new_step_length)) + CoM_bias, 0}; // touch-down lead of the contact point (in hip coordinate) + distance hip traveled during swing phase + hip travel difference during rest time because different incre_duty caused by change of step length + CoM_bias.
                diff_step_length = new_diff_step_length;
            }
            else
            {                               // hind leg swing
                int last_leg = (i + 2) % 4; // Contralateral front leg
                step_length = current_step_length[last_leg];
                next_step_length[i] = step_length; // apply hind step length corresponding to the front leg's.
                total_step_length = step_length + sign_diff[i] * diff_step_length;
                swing_hip_move_d = direction * swing_time * total_step_length;
                foothold[i] = {next_hip[i][0] + direction * touchdown_lead((1 - swing_time) * total_step_length) + swing_hip_move_d + CoM_bias, 0};
                incre_duty = dS / step_length; // change incre_duty corresponding to new step length when hind leg start to swing.
            } // end if else
            /* Swing setup, as in LegWheel: the swing is planned relative to the hip
               and tracks the rim point that will touch the ground. */
            double hip_velocity = (dS + sign_diff[i] * diff_dS) * rate;
            double swing_duration = swing_time * step_length / std::max(std::abs(velocity), 1e-3);
            double stance_travel = (1 - swing_time) * (step_length + std::abs(diff_step_length));
            double touchdown_x = foothold[i][0] - (next_hip[i][0] + swing_hip_move_d); // contact point ahead of the hip at touchdown
            sp[i].plan(leg_model, {theta[i], beta[i]}, prev_eta[i], touchdown_x, stand_height, hip_velocity,
                       legwheel_swing::swing_velocity_scale(stance_beta(stance_travel)),
                       step_height, swing_duration, 1.0 / rate, direction);
        }
        else if ((direction == 1) && (duty[i] > 1.0))
        { // entering stance phase when velocirty > 0
            touchdown = true;
            swing_phase[i] = 0;
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
            swing_phase[i] = 0;
            if (sp[i].getDirection() == direction)
            { // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] -= 1;
                current_step_length[i] = next_step_length[i];
            } // end if
        } // end if else
        /* Calculate next theta, beta */
        if (swing_phase[i] == 0)
        { // Stance phase
            // Roll on the foot arc with the ABAD joint held at gamma = 0.
            std::array<double, 3> eta_3d = leg_model.move_3d(theta[i], beta[i], 0.0, {next_hip[i][0] - hip[i][0], 0.0, next_hip[i][1] - hip[i][1]});
            result_eta = {eta_3d[0], eta_3d[1]};
        }
        else
        { // Swing phase
            if (sp[i].getDirection() == 1)
            { // direction == 1
                swing_phase_ratio = (duty[i] - (1 - swing_time)) / swing_time;
            }
            else
            { // direction == -1
                swing_phase_ratio = (1.0 - duty[i]) / swing_time;
            } // end if else
            result_eta = sp[i].eta(leg_model, swing_phase_ratio, {theta[i], beta[i]});
        } // end if else
        prev_eta[i] = {theta[i], beta[i]};
        theta[i] = result_eta[0];
        beta[i] = result_eta[1];
        hip[i] = next_hip[i];
    } // end for
    return {theta, beta};
} // end step

void WalkGait::set_velocity(double new_value)
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

void WalkGait::set_stand_height(double new_value)
{
    // The foot arc centre must clear the hip during swing and stay within reach.
    if (new_value > leg_model.foot_radius + max_reach || new_value < leg_model.foot_radius + step_height)
    {
        throw std::runtime_error("Stand height should be between foot_radius+\"step_height\" and foot_radius+max_reach (0.145+step_height ~ 0.38).");
    } // end if
    stand_height = new_value;
    for (int i = 0; i < 4; i++)
    {
        next_hip[i][1] = stand_height;
    } // end for
} // end set_stand_height

void WalkGait::set_step_length(double new_value)
{
    if (new_value <= 0.0)
    {
        throw std::runtime_error("Step length should be larger than zero.");
    } // end if
    if (new_value > max_step_length(stand_height))
    {
        throw std::runtime_error("Step length exceeds what the foot arc can cover at this stand height (see max_step_length).");
    } // end if
    new_step_length = new_value;
    // change differential step length if the robot is turning
    if (curvature != 0.0)
    {
        new_diff_step_length = new_step_length * (outer_radius - inner_radius) / (outer_radius + inner_radius); // apply new value when front leg swing
    } // end if
} // end set_step_length

void WalkGait::set_step_height(double new_value)
{
    if (new_value <= 0.0)
    {
        throw std::runtime_error("Step height should be larger than zero.");
    } // end if
    step_height = new_value;
} // end set_step_height

void WalkGait::set_curvature(double new_value)
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

void WalkGait::set_eta(std::array<std::array<double, 4>, 2> eta_)
{
    this->theta = eta_[0];
    this->beta = eta_[1];
} // end set_eta

void WalkGait::set_duty(std::array<double, 4> duty_)
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

std::array<int, 4> WalkGait::get_step_count()
{
    return this->step_count;
} // end get_step_count

std::array<int, 4> WalkGait::get_swing_phase()
{
    return this->swing_phase;
} // end get_step_count

std::array<double, 4> WalkGait::get_duty()
{
    return this->duty;
} // end get_duty

bool WalkGait::if_touchdown()
{
    return this->touchdown;
} // end if_touchdown