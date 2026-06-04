#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include "corgi_msgs/msg/impedance_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"
#include "corgi_msgs/msg/gmo_contact_state_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "corgi_walk/walk_gait_with_contact.hpp"
#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/bezier.hpp"

#define INIT_THETA (M_PI * 17.0 / 180.0)
#define INIT_BETA (0.0)

corgi_msgs::msg::TriggerStamped trigger_msg;
// 預設為站立 (true)，確保 WAIT-to-WALK 轉換時，支撐腳不會誤觸發探測
std::array<bool, 4> leg_contacts = {true, true, true, true}; 

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)
{
    trigger_msg = *msg;
} // end trigger_cb

void contact_cb(const corgi_msgs::msg::GMOContactStateStamped::SharedPtr msg)
{
    leg_contacts[0] = msg->module_a.contact;
    leg_contacts[1] = msg->module_b.contact;
    leg_contacts[2] = msg->module_c.contact;
    leg_contacts[3] = msg->module_d.contact;
}

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

Marker make_marker(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std::string & ns,
    int id,
    int type)
{
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
}

std::vector<geometry_msgs::msg::Point> get_support_points(
    const WalkGaitWithContact::DebugState & debug,
    double body_width)
{
    const std::array<double, 4> leg_y = {body_width / 2.0, -body_width / 2.0, -body_width / 2.0, body_width / 2.0};
    std::vector<geometry_msgs::msg::Point> support_points;
    for (int i = 0; i < 4; i++)
    {
        const bool is_support = debug.swing_phase[i] == 0 && debug.contact_state[i] && !debug.late_probing[i];
        if (is_support)
        {
            support_points.push_back(make_point(debug.foot_point[i][0], leg_y[i], debug.foot_point[i][1] + 0.01));
        }
    }

    if (support_points.size() < 3)
    {
        return support_points;
    }

    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto & point : support_points)
    {
        center_x += point.x;
        center_y += point.y;
    }
    center_x /= static_cast<double>(support_points.size());
    center_y /= static_cast<double>(support_points.size());

    std::sort(support_points.begin(), support_points.end(), [center_x, center_y](const auto & a, const auto & b) {
        return std::atan2(a.y - center_y, a.x - center_x) < std::atan2(b.y - center_y, b.x - center_x);
    });
    return support_points;
}

void append_touchdown_history(
    const WalkGaitWithContact::DebugState & debug,
    std::array<std::vector<geometry_msgs::msg::Point>, 4> & touchdown_history,
    double body_width,
    std::size_t max_points)
{
    const std::array<double, 4> leg_y = {body_width / 2.0, -body_width / 2.0, -body_width / 2.0, body_width / 2.0};
    for (int i = 0; i < 4; i++)
    {
        if (!debug.touchdown_leg[i])
        {
            continue;
        }
        touchdown_history[i].push_back(make_point(debug.touchdown_point[i][0], leg_y[i], debug.touchdown_point[i][1]));
        if (touchdown_history[i].size() > max_points)
        {
            touchdown_history[i].erase(touchdown_history[i].begin());
        }
    }
}

void publish_walk_debug_markers(
    const rclcpp::Publisher<MarkerArray>::SharedPtr & marker_pub,
    const WalkGaitWithContact::DebugState & debug,
    const std::array<std::vector<geometry_msgs::msg::Point>, 4> & touchdown_history,
    const std::array<double, 2> & com_bias,
    double body_width,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
{
    const std::array<double, 4> leg_y = {body_width / 2.0, -body_width / 2.0, -body_width / 2.0, body_width / 2.0};
    const std::array<std::string, 4> leg_names = {"LF", "RF", "RH", "LH"};
    const std::array<std_msgs::msg::ColorRGBA, 4> leg_colors = {
        make_color(0.95f, 0.22f, 0.20f, 1.0f),
        make_color(0.20f, 0.72f, 0.32f, 1.0f),
        make_color(0.22f, 0.42f, 0.95f, 1.0f),
        make_color(0.95f, 0.78f, 0.20f, 1.0f)};

    MarkerArray markers;
    const auto support_points = get_support_points(debug, body_width);

    double body_center_x = 0.0;
    double body_center_z = 0.0;
    for (int i = 0; i < 4; i++)
    {
        body_center_x += debug.hip[i][0];
        body_center_z += debug.hip[i][1];
    }
    body_center_x /= 4.0;
    body_center_z /= 4.0;

    Marker body = make_marker(frame_id, stamp, "body_outline", 0, Marker::LINE_STRIP);
    body.scale.x = 0.01;
    body.color = make_color(0.85f, 0.85f, 0.85f, 0.9f);
    body.points = {
        make_point(debug.hip[0][0], leg_y[0], debug.hip[0][1]),
        make_point(debug.hip[1][0], leg_y[1], debug.hip[1][1]),
        make_point(debug.hip[2][0], leg_y[2], debug.hip[2][1]),
        make_point(debug.hip[3][0], leg_y[3], debug.hip[3][1]),
        make_point(debug.hip[0][0], leg_y[0], debug.hip[0][1])};
    markers.markers.push_back(body);

    Marker hips = make_marker(frame_id, stamp, "hips", 1, Marker::CUBE_LIST);
    hips.scale.x = 0.035;
    hips.scale.y = 0.035;
    hips.scale.z = 0.035;
    hips.color = make_color(0.85f, 0.85f, 0.85f, 1.0f);
    for (int i = 0; i < 4; i++)
    {
        hips.points.push_back(make_point(debug.hip[i][0], leg_y[i], debug.hip[i][1]));
    }
    markers.markers.push_back(hips);

    Marker com = make_marker(frame_id, stamp, "com", 2, Marker::SPHERE);
    com.pose.position = make_point(body_center_x + com_bias[0], com_bias[1], body_center_z);
    com.scale.x = 0.06;
    com.scale.y = 0.06;
    com.scale.z = 0.06;
    com.color = make_color(1.0f, 1.0f, 1.0f, 1.0f);
    markers.markers.push_back(com);

    Marker com_label = make_marker(frame_id, stamp, "labels", 2, Marker::TEXT_VIEW_FACING);
    com_label.pose.position = make_point(body_center_x + com_bias[0], com_bias[1] + 0.055, body_center_z + 0.06);
    com_label.scale.z = 0.05;
    com_label.color = make_color(1.0f, 1.0f, 1.0f, 1.0f);
    com_label.text = "CoM";
    markers.markers.push_back(com_label);

    Marker support_vertices = make_marker(frame_id, stamp, "support_polygon_vertices", 3, Marker::SPHERE_LIST);
    support_vertices.scale.x = 0.032;
    support_vertices.scale.y = 0.032;
    support_vertices.scale.z = 0.032;
    support_vertices.color = make_color(0.0f, 0.95f, 0.95f, 1.0f);
    support_vertices.points = support_points;
    markers.markers.push_back(support_vertices);

    if (support_points.size() >= 2)
    {
        Marker support_outline = make_marker(frame_id, stamp, "support_polygon_outline", 4, Marker::LINE_STRIP);
        support_outline.scale.x = 0.018;
        support_outline.color = make_color(0.0f, 0.95f, 0.95f, 1.0f);
        support_outline.points = support_points;
        if (support_points.size() >= 3)
        {
            support_outline.points.push_back(support_points.front());
        }
        markers.markers.push_back(support_outline);
    }

    if (support_points.size() >= 3)
    {
        Marker support_fill = make_marker(frame_id, stamp, "support_polygon_fill", 5, Marker::TRIANGLE_LIST);
        support_fill.color = make_color(0.0f, 0.75f, 0.9f, 0.22f);
        for (std::size_t i = 1; i + 1 < support_points.size(); i++)
        {
            support_fill.points.push_back(support_points[0]);
            support_fill.points.push_back(support_points[i]);
            support_fill.points.push_back(support_points[i + 1]);
        }
        markers.markers.push_back(support_fill);
    }

    for (int i = 0; i < 4; i++)
    {
        Marker history = make_marker(frame_id, stamp, "touchdown_history", 10 + i, Marker::SPHERE_LIST);
        history.scale.x = 0.018;
        history.scale.y = 0.018;
        history.scale.z = 0.018;
        history.color = leg_colors[i];
        history.color.a = 0.55f;
        history.points = touchdown_history[i];
        markers.markers.push_back(history);

        Marker touchdown = make_marker(frame_id, stamp, "touchdown_target", 20 + i, Marker::SPHERE);
        touchdown.pose.position = make_point(debug.touchdown_point[i][0], leg_y[i], debug.touchdown_point[i][1]);
        const double scale = debug.touchdown_leg[i] ? 0.07 : 0.04;
        touchdown.scale.x = scale;
        touchdown.scale.y = scale;
        touchdown.scale.z = scale;
        touchdown.color = leg_colors[i];
        touchdown.color.a = debug.swing_phase[i] ? 0.95f : 0.55f;
        markers.markers.push_back(touchdown);

        Marker foot = make_marker(frame_id, stamp, "foot_current", 40 + i, Marker::CUBE);
        foot.pose.position = make_point(debug.foot_point[i][0], leg_y[i], debug.foot_point[i][1]);
        foot.scale.x = 0.028;
        foot.scale.y = 0.028;
        foot.scale.z = 0.028;
        foot.color = leg_colors[i];
        foot.color.a = 1.0f;
        markers.markers.push_back(foot);

        Marker label = make_marker(frame_id, stamp, "labels", 30 + i, Marker::TEXT_VIEW_FACING);
        label.pose.position = make_point(debug.touchdown_point[i][0], leg_y[i] + 0.055, debug.touchdown_point[i][1] + 0.055);
        label.scale.z = 0.045;
        label.color = leg_colors[i];
        label.text = leg_names[i];
        markers.markers.push_back(label);
    }

    marker_pub->publish(markers);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("walk_impedance_test");
    auto imp_cmd_pub = node->create_publisher<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 10);
    auto phase_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("walk/swing_phase", 10);
    auto debug_marker_pub = node->create_publisher<MarkerArray>("walk/debug_markers", 10);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10, trigger_cb);
        
    auto contact_sub = node->create_subscription<corgi_msgs::msg::GMOContactStateStamped>(
        "/gmo/contact_state", 10, contact_cb);

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
    std_msgs::msg::Int32MultiArray phase_msg;
    phase_msg.data.resize(4, 0);
    std::array<corgi_msgs::msg::ImpedanceCmd *, 4> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d};
    for (int i = 0; i < 4; i++)
    {
        imp_cmd_modules[i]->theta = INIT_THETA;
        imp_cmd_modules[i]->beta = INIT_BETA;
        imp_cmd_modules[i]->fx = 0.0;
        imp_cmd_modules[i]->fy = -52.0;
        imp_cmd_modules[i]->mx = 0.0;
        imp_cmd_modules[i]->my = 0.0;
        imp_cmd_modules[i]->bx = 200.0;
        imp_cmd_modules[i]->by = 200.0;
        imp_cmd_modules[i]->kx = 2000.0;
        imp_cmd_modules[i]->ky = 2000.0;
    } // end for

    enum STATES
    {
        INIT,
        TRANSFORM,
        WAIT,
        WALK,
        END
    };
    const std::array<double, 2> CoM_bias = {-0.02, 0.0};
    const double body_width = 0.4;
    const int sampling_rate = 1000;
    const int transform_count = 5 * sampling_rate; // 5s
    // const double init_eta[8] = {1.857467698281913, 0.4791102940603915, 1.6046663223045279, 0.12914729012802004, 1.6046663223045279, -0.12914729012802004, 1.857467698281913, -0.4791102940603915}; // stand height 0.25
    // const double init_eta[8] = {1.8571554834938668,0.4790144528333341,2.0636290799909855,0.10633741753260191,2.0636290799909855,-0.10633741753260191,1.8571554834938668,-0.4790144528333341}; // left stand height 0.25, right stand 0.3
    const double init_eta[8] = {1.2744470401482761, 0.4161719979302237, 1.1222141023936798, 0.11005079310996896, 1.1222141023936798, -0.11005079310996896, 1.2744470401482761, -0.4161719979302237};  // stand height 0.2
    double velocity = 0.1;
    double stand_height = 0.2;
    double step_length = 0.2;
    double step_height = 0.06;
    std::array<double, 4> ground_offset = {0.0, 0.0, 0.0, 0.0}; // LF, RF, RH, LH
    int count = 0;
    std::array<int, 4> step_count;
    double max_cal_time = 0.0;

    /* Initial variable */
    WalkGaitWithContact walk_gait(false, CoM_bias[0], sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA, INIT_BETA, INIT_BETA, INIT_BETA}}}; // init eta (wheel mode)

    /* Other variable */
    STATES state = INIT, last_state = INIT;
    double transform_ratio;
    int command_count;
    bool gait_initialized = false;
    int visualization_count = 0;
    const std::size_t touchdown_history_limit = 200;
    std::array<std::vector<geometry_msgs::msg::Point>, 4> touchdown_history;

    // Runtime-tunable leg ground offsets (LF, RF, RH, LH), useful for different leg-length tests.
    node->declare_parameter<std::vector<double>>("ground_offset", {ground_offset[0], ground_offset[1], ground_offset[2], ground_offset[3]});
    node->declare_parameter<bool>("enable_visualization", true);
    node->declare_parameter<int>("visualization_decimation", 20);
    node->declare_parameter<std::string>("visualization_frame", "walk_debug");
    std::array<double, 4> applied_ground_offset = ground_offset;
    auto to_array4 = [](const std::vector<double> & values) {
        std::array<double, 4> out = {0.0, 0.0, 0.0, 0.0};
        std::copy_n(values.begin(), 4, out.begin());
        return out;
    };
    auto offsets_changed = [](const std::array<double, 4> & a, const std::array<double, 4> & b) {
        const double eps = 1e-9;
        for (int i = 0; i < 4; i++)
        {
            if (std::abs(a[i] - b[i]) > eps)
            {
                return true;
            }
        }
        return false;
    };

    /* Behavior loop */
    // --- Synchronization Setup ---
    // Define control period based on sampling_rate (seconds = 1 / sampling_rate)
    rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / static_cast<double>(sampling_rate));

    // Wait for the ROS 2 clock to start (important if simulation is paused)
    RCLCPP_INFO(node->get_logger(), "Waiting for Webots clock...");
    while (rclcpp::ok())
    {
        // 1. Process callbacks to try to receive /clock messages
        rclcpp::spin_some(node);

        // 2. Check if the current time is greater than 0 (indicates that the clock has been received)
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break; // Successfully synchronized, exit waiting loop
        }

        // 3. Sleep briefly to avoid 100% CPU usage (Wall time is fine while waiting for the connection)
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto start_time = node->now();
    rclcpp::Time next_time = start_time;
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    walk_gait.set_ground_offset(ground_offset);
    while (rclcpp::ok())
    {
        auto one_loop_start = std::chrono::high_resolution_clock::now();
        rclcpp::spin_some(node);

        std::vector<double> ground_offset_param;
        if (node->get_parameter("ground_offset", ground_offset_param))
        {
            if (ground_offset_param.size() == 4)
            {
                std::array<double, 4> requested_ground_offset = to_array4(ground_offset_param);
                if (offsets_changed(requested_ground_offset, applied_ground_offset))
                {
                    try
                    {
                        walk_gait.set_ground_offset(requested_ground_offset);
                        applied_ground_offset = requested_ground_offset;
                        RCLCPP_INFO(node->get_logger(),
                                    "Updated ground_offset [LF, RF, RH, LH] = [%.4f, %.4f, %.4f, %.4f]",
                                    applied_ground_offset[0],
                                    applied_ground_offset[1],
                                    applied_ground_offset[2],
                                    applied_ground_offset[3]);
                    }
                    catch (const std::exception & e)
                    {
                        RCLCPP_WARN(node->get_logger(),
                                    "Reject ground_offset update: %s",
                                    e.what());
                        node->set_parameter(rclcpp::Parameter(
                            "ground_offset",
                            std::vector<double>{applied_ground_offset[0],
                                                applied_ground_offset[1],
                                                applied_ground_offset[2],
                                                applied_ground_offset[3]}));
                    }
                }
            }
            else
            {
                RCLCPP_WARN_THROTTLE(node->get_logger(),
                                     *node->get_clock(),
                                     2000,
                                     "Parameter ground_offset must have exactly 4 values: [LF, RF, RH, LH]");
            }
        }

        if (state == END)
        {
            break;
        } // end if
        // state machine
        switch (state)
        {
        case INIT:
            transform_ratio = 0.0;
            command_count = 0;
            break;
        case TRANSFORM:
            transform_ratio += 1.0 / transform_count;
            for (int i = 0; i < 4; i++)
            {
                eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i * 2] - INIT_THETA);
                eta_list[1][i] = INIT_BETA + transform_ratio * (init_eta[i * 2 + 1] - INIT_BETA);
                eta_list[1][i] = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
            } // end for
            break;
        case WAIT:
            if (last_state != state)
            {
                double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                walk_gait.initialize(current_eta);
                gait_initialized = true;
            } // end if
            break;
        case WALK:
            // 每週期的計算前，將最新的觸地狀態寫入步態生成器
            walk_gait.set_contact_state(leg_contacts);
            eta_list = walk_gait.step();
            command_count++;
            break;
        default:
            break;
        } // end switch
        last_state = state;

        // next state
        switch (state)
        {
        case INIT:
            state = TRANSFORM;
            break;
        case TRANSFORM:
            if (transform_ratio > 1.0)
            {
                state = WAIT;
            } // end if
            break;
        case WAIT:
            if (trigger_msg.enable)
            {
                state = WALK;
            } // end if
            break;
        case WALK:
            step_count = walk_gait.get_step_count();
            if (step_count[0] >= 10 && step_count[1] >= 10 && step_count[2] >= 10 && step_count[3] >= 10)
            { // all legs have stepped at least twice
                state = END;
            } // end if
            break;
        default:
            break;
        } // end switch

        /* Publish impedance commands */
        for (int i = 0; i < 4; i++)
        {
            if (eta_list[0][i] > M_PI * 160.0 / 180.0)
            {
                std::cout << "Leg " << i << " exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI * 160.0 / 180.0;
            } // end if
            if (eta_list[0][i] < M_PI * 16.9 / 180.0)
            {
                std::cout << "Leg " << i << " exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI * 16.9 / 180.0;
            } // end if
            imp_cmd_modules[i]->theta = eta_list[0][i];
            imp_cmd_modules[i]->beta = (i == 1 || i == 2) ? (eta_list[1][i]) : -(eta_list[1][i]);
            imp_cmd_modules[i]->fx = 0.0;
            imp_cmd_modules[i]->fy = -52.0;
            imp_cmd_modules[i]->mx = 0.0;
            imp_cmd_modules[i]->my = 0.0;
        } // end for
        imp_cmd.header.seq = count++;
        imp_cmd.header.stamp = node->now();
        imp_cmd_pub->publish(imp_cmd);
        auto swing_phase = walk_gait.get_swing_phase();
        for (int i = 0; i < 4; i++) { phase_msg.data[i] = swing_phase[i]; }
        phase_pub->publish(phase_msg);

        if (gait_initialized)
        {
            const auto debug_state = walk_gait.get_debug_state();
            append_touchdown_history(debug_state, touchdown_history, body_width, touchdown_history_limit);

            bool enable_visualization = true;
            int visualization_decimation = 20;
            std::string visualization_frame = "walk_debug";
            node->get_parameter("enable_visualization", enable_visualization);
            node->get_parameter("visualization_decimation", visualization_decimation);
            node->get_parameter("visualization_frame", visualization_frame);
            visualization_decimation = std::max(1, visualization_decimation);
            if (enable_visualization && visualization_count++ % visualization_decimation == 0)
            {
                publish_walk_debug_markers(
                    debug_marker_pub,
                    debug_state,
                    touchdown_history,
                    CoM_bias,
                    body_width,
                    visualization_frame,
                    node->now());
            }
        }

        auto one_loop_end = std::chrono::high_resolution_clock::now();
        auto one_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(one_loop_end - one_loop_start);
        if (one_loop_duration.count() > max_cal_time)
        {
            max_cal_time = one_loop_duration.count();
            std::cout << "max time: " << max_cal_time << " us" << std::endl;
        } // end if

        // --- Synchronized Sleep ---
        next_time += period;
        if (!node->get_clock()->sleep_until(next_time))
        {
            // If the clock jumps or we miss a cycle, warn but continue
            RCLCPP_WARN(node->get_logger(), "Missed control cycle or clock jump");
            next_time = node->now(); // Reset baseline
        }
    } // end while

    auto end_time = node->now();
    auto duration = end_time - start_time;

    std::cout << "max time: " << max_cal_time << " us" << std::endl;
    std::cout << "time: " << duration.seconds() << " seconds" << std::endl;
    std::cout << "total count: " << command_count << std::endl;

    rclcpp::shutdown();
    return 0;
} // end main
