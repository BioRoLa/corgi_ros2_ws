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
    cout << "Robot Walking Setting" << endl;
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
    // Initial settings and check
    setting(walking_distance, step_length, velocity, tilt_angle_deg, tilt_angle_rad);
    genTerrainPosition(walking_distance * 1000, tilt_angle_rad, terrain_positions);
    setting_calculation(walking_distance, step_length, tilt_angle_rad, terrain_positions, false);
    findMinMaxHeight(terrain_positions);
    cout << "===================================" << endl;
    // Calculations

    return 0;
}