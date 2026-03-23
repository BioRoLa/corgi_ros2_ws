#include "offline/OfflineTestNode.hpp"
#include "common/Params.hpp"
#include "common/ParamsIO.hpp"
#include <iostream>
#include <string>
#include <filesystem>

int main(int argc, char** argv) {
    try {
        corgi::Params params;

        // Default YAML path: <package>/config/config_test.yaml
        std::string yaml_path =
            (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()
             / "config" / "config_tuned_v1.yaml").string();

        // Check for --config <path> argument
        for (int i = 1; i < argc - 1; ++i) {
            if (std::string(argv[i]) == "--config") {
                yaml_path = argv[i + 1];
                break;
            }
        }

        // Load YAML if the file exists
        if (std::filesystem::exists(yaml_path)) {
            params = corgi::load_params(yaml_path);
            std::cout << "Loaded config: " << yaml_path << "\n";
        }

        // CLI overrides (backward-compatible positional args)
        // Usage: offline_test [sigma_a_x] [sigma_leg_x] [sigma_leg_z] [threshold] [imu_noise] [quiet] [use_esekf]
        // Skip positional parsing when --config is used
        bool has_config_flag = false;
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--config") { has_config_flag = true; break; }
        }
        if (!has_config_flag) {
            if (argc >= 2) params.sigma_a.x()           = std::stof(argv[1]);
            if (argc >= 3) params.sigma_leg_vec.x()     = std::stof(argv[2]);
            if (argc >= 4) params.sigma_leg_vec.z()     = std::stof(argv[3]);
            if (argc >= 5) params.mahalanobis_threshold = std::stof(argv[4]);
            if (argc >= 6) params.simulate_imu_noise    = (std::string(argv[5]) == "1");
            if (argc >= 7) params.quiet                 = (std::string(argv[6]) == "1");
            if (argc >= 8) params.use_esekf_state       = (std::string(argv[7]) == "1");
        }

        corgi::OfflineTestNode node(params);
        return node.run();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
