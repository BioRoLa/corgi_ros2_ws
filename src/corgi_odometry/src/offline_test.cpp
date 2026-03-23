#include "OfflineTestNode.hpp"
#include "Params.hpp"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    try {
        // CLI argument parsing (backward-compatible)
        // Usage: offline_test [sigma_a_x] [sigma_leg_x] [sigma_leg_z] [threshold] [imu_noise] [quiet] [use_esekf]
        corgi::Params params;
        if (argc >= 2) params.sigma_a.x()       = std::stof(argv[1]);
        if (argc >= 3) params.sigma_leg_vec.x()  = std::stof(argv[2]);
        if (argc >= 4) params.sigma_leg_vec.z()  = std::stof(argv[3]);
        if (argc >= 5) params.mahalanobis_threshold = std::stof(argv[4]);
        if (argc >= 6) params.simulate_imu_noise = (std::string(argv[5]) == "1");
        if (argc >= 7) params.quiet              = (std::string(argv[6]) == "1");
        if (argc >= 8) params.use_esekf_state    = (std::string(argv[7]) == "1");

        corgi::OfflineTestNode node(params);
        return node.run();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
