#include "wlw.hpp"

double deg2rad(double degrees) {
    return degrees * PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / PI;
}

// Generate linspace for terrain position (distance and height) with 1 mm resolution
void genTerrainPosition(double walking_distance, double tilt_angle_rad, vector<pair<double, double>> &terrain_positions) {
    for (int i = 0; i <= walking_distance; ++i) {
        double height = i * tan(tilt_angle_rad);
        terrain_positions.emplace_back(i, height); // (distance, height)
    }
}

void setting(double &walking_distance, double &step_length, double &velocity, double &tilt_angle_deg, double &tilt_angle_rad) {
    walking_distance = 0.4;    // m
    step_length = 300.0;       // mm
    velocity = 0.2;            // m/s
    tilt_angle_deg = 20.0;     // degrees
    tilt_angle_rad = deg2rad(tilt_angle_deg); // Convert to radians
}

void setting_calculation(double walking_distance, double step_length, double tilt_angle_rad, const vector<pair<double, double>> &terrain_positions, bool output_details) {
    // Step calculation considering the swing phase
    double total_step_length = (4.0 / 3.0) * step_length;
    int total_steps = static_cast<int>(walking_distance * 1000 / total_step_length);

    // Output simulation details
    cout << "==================================="<< endl;
    cout << "Robot Setting" << endl;
    cout << "Walking Distance: " << walking_distance << " m" << endl;
    cout << "Terrain Tilt Angle: " << rad2deg(tilt_angle_rad) << " degrees" << endl;
    cout << "Step Length: " << step_length << " mm" << endl;
    cout << "Body Velocity: " << velocity << " m/s" << endl;
    cout << "Total Steps: " << total_steps << " steps" << endl;
    

    if (output_details) {
        cout << "===================================" << endl;
        cout << "\nTerrain Position (1 mm resolution):" << endl;
        for (const auto &position : terrain_positions) {
            cout << "Distance: " << position.first << " mm, Height: " << position.second << " mm" << endl;
        }
    }
}

void findMinMaxHeight(const vector<pair<double, double>> &terrain_positions) {
    // Find the maximum height
    auto max_height_it = std::max_element(
        terrain_positions.begin(), 
        terrain_positions.end(), 
        [](const pair<double, double> &a, const pair<double, double> &b) {
            return a.second < b.second; 
        }
    );

    // Find the minimum height
    auto min_height_it = std::min_element(
        terrain_positions.begin(), 
        terrain_positions.end(), 
        [](const pair<double, double> &a, const pair<double, double> &b) {
            return a.second < b.second; 
        }
    );

    // Print results
    if (max_height_it != terrain_positions.end()) {
        cout << "===================================" << endl;
        cout << "Maximum Height: " << max_height_it->second << " mm at Distance: " 
             << max_height_it->first << " mm" << endl;
    }

    if (min_height_it != terrain_positions.end()) {
        cout << "Minimum Height: " << min_height_it->second << " mm at Distance: " 
             << min_height_it->first << " mm" << endl;
    }

    // Check height range
    if (max_height_it != terrain_positions.end() && min_height_it != terrain_positions.end()) {
        double height_range = max_height_it->second - min_height_it->second +119;
        if (height_range < 340) {
            cout << "Leg height Range from " << height_range << " mm to 340 mm" << endl;
        }
    }
}
// 250104 assume calculate single leg and for only one cycle


int main(int argc, char **argv) { 
    // // Initial settings and check
    // setting(walking_distance, step_length, velocity, tilt_angle_deg, tilt_angle_rad);
    // genTerrainPosition(walking_distance * 1000, tilt_angle_rad, terrain_positions);
    // setting_calculation(walking_distance, step_length, tilt_angle_rad, terrain_positions, false);
    // findMinMaxHeight(terrain_positions);
    // cout << "===================================" << endl;
    // // Calculations

    ros::init(argc, argv, "wlw_pub");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    ros::Rate rate(1000);

    // bool sim = true;
    // LegModel leg_model(sim);
    // LegModel leg_model2(sim);

    // InputData Input;
    // Input.MaxTDlength = 0.1;
    // //sqrt(pow(0.340, 2) - pow(Input.Height, 2)); // m 
    // Input.total_time = Input.Distance / Input.Velocity;

    // // Define O_list
    // vector<Vector2d> O_list;
    // for (double x = 0; x <= Input.MaxTDlength; x += 0.0001) {
    //     O_list.push_back(Vector2d(x, 0)); // Add points along the x-axis
    // }

    // double new_theta_beta[2] = {deg2rad(50),deg2rad(45)};
    // leg_model.forward(new_theta_beta[0], new_theta_beta[1],true);
    // leg_model.contact_map(new_theta_beta[0], new_theta_beta[1],deg2rad(20));
    
    // std::array<double, 2> new_theta_beta2;
    // double pos[2] = {leg_model.contact_p[0] - 0.1125*sin(deg2rad(20)),leg_model.contact_p[1]+0.1125*cos(deg2rad(20))};
    // new_theta_beta2 = leg_model2.inverse(pos, "L_l");
    // // std::cout << "Inverse Output eta(degree): " << new_theta_beta2[0]*180.0/M_PI << ", "<< new_theta_beta2[1]*180.0/M_PI << std::endl;
    // leg_model2.contact_map(new_theta_beta2[0], new_theta_beta2[1], deg2rad(20));
    // leg_model2.forward(new_theta_beta2[0], new_theta_beta2[1],true);
    // // std::cout << "Inverse contact: " << leg_model2.contact_p[0] << ", " << leg_model2.contact_p[1] << std::endl;
    // std::cout << "Height: " << -leg_model2.contact_p[1]  << std::endl;
    // // Check and update theta, beta
    // for (int i=0; i<4; i++) {
    //     motor_cmd_modules[i]->theta = new_theta_beta2[0];
    //     if (i==1 || i==2) {
    //         motor_cmd_modules[i]->beta  = new_theta_beta2[1];
    //     } else {
    //         motor_cmd_modules[i]->beta  = -new_theta_beta2[1];
    //     }
    //     motor_cmd_modules[i]->kp = 90;
    //     motor_cmd_modules[i]->ki = 0;
    //     motor_cmd_modules[i]->kd = 1.75;
    //     motor_cmd_modules[i]->torque_r = 0;
    //     motor_cmd_modules[i]->torque_l = 0;
    // }//end for
    // for (int i=0; i<1000; i++) {
    //     motor_pub.publish(motor_cmd);
    //     rate.sleep();  
    // }

    // // // Open CSV file for writing
    // // std::ofstream csv_file("robot_data.csv");
    // // if (!csv_file.is_open()) {
    // //     std::cerr << "Error: Could not open file for writing.\n";
    // //     return -1;
    // // }
    // //  // Write the CSV header
    // // csv_file << "i,theta (deg),beta (deg),height (mm)\n";

    // std::array<double, 2> result_eta;
    // for (size_t i = 0; i < O_list.size(); ++i) {
    //     // cout << "O[" << i << "] = (" << O_list[i][0] << ", " << O_list[i][1] << ")" << endl;
    //     result_eta = leg_model2.move(new_theta_beta2[0], new_theta_beta2[1], {0.0001, 0}, deg2rad(20));
    //     new_theta_beta2[0] = result_eta[0];
    //     new_theta_beta2[1] = result_eta[1];
    //     leg_model2.contact_map(new_theta_beta2[0], new_theta_beta2[1], deg2rad(20));
    //     // std::cout << "Height: " << -leg_model2.contact_p[1] + i*0.0001*sin(deg2rad(20)) << std::endl;


    //     // // Write data to CSV file
    //     // csv_file << i << ","
    //     //          << new_theta_beta2[0] * 180.0 / M_PI << ","
    //     //          << new_theta_beta2[1] * 180.0 / M_PI << ","
    //     //          << -leg_model.contact_p[1]  << "\n";
        
    //     // Check and update theta, beta
    //     for (int i=0; i<4; i++) {
    //         motor_cmd_modules[i]->theta = new_theta_beta2[0];
    //         if (i==1 || i==2) {
    //             motor_cmd_modules[i]->beta  = new_theta_beta2[1];
    //         } else {
    //             motor_cmd_modules[i]->beta  = -new_theta_beta2[1];
    //         }
    //         motor_cmd_modules[i]->kp = 90;
    //         motor_cmd_modules[i]->ki = 0;
    //         motor_cmd_modules[i]->kd = 1.75;
    //         motor_cmd_modules[i]->torque_r = 0;
    //         motor_cmd_modules[i]->torque_l = 0;
    //         std::cout << "current_eta " << i << ": "<< new_theta_beta2[0]*180.0/M_PI << ","<<new_theta_beta2[1]*180.0/M_PI<< std::endl;
    //     }
    //     motor_pub.publish(motor_cmd);
    //     rate.sleep();
    // }


    // // // Close the CSV file
    // // csv_file.close();
    // // std::cout << "Data saved to robot_data.csv\n";
    

    bool sim = true;
    LegModel leg_model(sim);
    // try wlw pose function
    double Height = 0.120;
    double slope_deg = 20;
    // 腳要多前面觸地
    double shift_x = -0.1;
    std::array<double, 2> new_theta_beta2;
    double pos[2] = {shift_x - leg_model.radius*sin(deg2rad(slope_deg)),-Height+leg_model.radius*cos(deg2rad(slope_deg))};
    new_theta_beta2 = leg_model.inverse(pos, "U_l");
    std::cout << "Inverse Output eta(degree): " << new_theta_beta2[0]*180.0/M_PI << ", "<< new_theta_beta2[1]*180.0/M_PI << std::endl;
    leg_model.contact_map(new_theta_beta2[0], new_theta_beta2[1], deg2rad(slope_deg));
    leg_model.forward(new_theta_beta2[0], new_theta_beta2[1],true);
    std::cout << "Inverse contact: " << leg_model.contact_p[0] << ", " << leg_model.contact_p[1] << std::endl;
    std::cout << "Height: " << -leg_model.contact_p[1]  << std::endl;
    new_theta_beta2 = {deg2rad(76.81),deg2rad(-39.56)};    // Check and update theta, beta
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->theta = new_theta_beta2[0];
        if (i==1 || i==2) {
            motor_cmd_modules[i]->beta  = new_theta_beta2[1];
        } else {
            motor_cmd_modules[i]->beta  = -new_theta_beta2[1];
        }
        motor_cmd_modules[i]->kp = 90;
        motor_cmd_modules[i]->ki = 0;
        motor_cmd_modules[i]->kd = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for

    motor_cmd_modules[0]->theta =deg2rad(76.81);
    motor_cmd_modules[0]->beta = deg2rad(-39.56);
    motor_cmd_modules[1]->theta =deg2rad(70.15);
    motor_cmd_modules[1]->beta = deg2rad(34.64);
    motor_cmd_modules[2]->theta =deg2rad(70.15);
    motor_cmd_modules[2]->beta = deg2rad(-34.64);    
    motor_cmd_modules[3]->theta =deg2rad(76.81);
    motor_cmd_modules[3]->beta = deg2rad(39.56);

    for (int i=0; i<1000; i++) {
        motor_pub.publish(motor_cmd);
        rate.sleep();  
    }

    std::cout << "end" << -leg_model.contact_p[1]  << std::endl;
    return 0;
}