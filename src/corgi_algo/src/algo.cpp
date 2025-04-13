#include "Simple_fsm.hpp"
#include "hybrid_gen.hpp"
#include "wheeled_gen.hpp"
#include "legged_gen.hpp"
#include "transform_gen.hpp"

class KeybordControl : public Wheeled, public Hybrid, public Legged, public Transform
{
    public: 
        KeybordControl(ros::NodeHandle& nh, GaitSelector& gs): Wheeled(nh), Hybrid(nh), Legged(nh), Transform(), gaitSelector(gs)
        {
            // Initialize the keyboard input thread
            input_thread = std::thread(&KeybordControl::keyboardInputThread, this);
        }
        ~KeybordControl() {
            if (input_thread.joinable()) {
                input_thread.join();
            }
        }
        void keyboardInputThread() {
            while (ros::ok()) {
                // Read an entire line from the console.
                std::string input_line;
                std::cout << "Enter command: ";
                std::getline(std::cin, input_line);
                
                // Use a lock if needed during the processing.
                {
                    std::lock_guard<std::mutex> lock(input_mutex);
                    
                    // Here, dispatch based on the first character:
                    if (input_line.empty())
                        continue;
                    
                    char identifier = input_line[0];
                    // For gait commands, use specific single letters
                    if (identifier == 'l' && input_line == "l") { // for Legged gait
                        std::cout << "Legged" << std::endl;
                        changeGait("2");
                    } else if (identifier == 'w' && input_line == "w") { // for Wheeled gait
                        std::cout << "Wheeled" << std::endl;
                        changeGait("1");
                    } else if (identifier == 'h' && input_line == "h") { // for Hybrid gait
                        std::cout << "Hybrid" << std::endl;
                        changeGait("3");
                    } else if (identifier == 'p') {
                        std::cout << "Paused" << std::endl;
                        // Optionally implement pause functionality.
                    } else if (identifier == 'r') {
                        std::cout << "Resume" << std::endl;
                        // Optionally implement resume.
                    } else if (identifier == 'q') {
                        std::cout << "Quitting, closing everything (including joystick)..." << std::endl;
                        
                        // Signal the joystick thread (or any extra threads) to stop
                        joyShutdown = true;
                        if(joyThread.joinable()) {
                            joyThread.join();
                        }
                        ros::shutdown();
                    } else {
                        // For parameter change commands:
                        changeGait(input_line); // Pass the full string command
                    }
                }
            }
        }
        
    private:
        GaitSelector& gaitSelector;   // hold a reference
        std::mutex input_mutex;
        std::thread input_thread;
        void changeGait(const std::string& command) {
            // Set the newGait to the current value by default.
            gaitSelector.newGait = gaitSelector.currentGait;
        
            // Gait-change commands: you use exact string matches "1", "2", "3".
            if (command == "1") {
                gaitSelector.newGait = Gait::WHEELED;
            } else if (command == "2") {
                gaitSelector.newGait = Gait::LEGGED;
            } else if (command == "3") {
                gaitSelector.newGait = Gait::HYBRID;
            }
            // Parameter change commands:
            else if (command[0] == 'v') {
                // Change velocity command: e.g., "v1.5"
                // Extract the substring after the command letter.
                std::string velocity_str = command.substr(1);
                try {
                    // see the gait and use the function od the wheel mode just set
                    double newVelocity = std::stod(velocity_str);
                    std::cout << "Origin velocity : " << gaitSelector.velocity << std::endl;
                    gaitSelector.velocity = newVelocity;
                    std::cout << "Setting velocity : " << gaitSelector.velocity << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Invalid velocity parameter: " << velocity_str << std::endl;
                }
                return;
            } else if (command[0] == 's') {
                std::string stepLength_str = command.substr(1);
                try {
                    // see the gait and use the function od the wheel mode just set
                    std::cout << "Origin step length : " << gaitSelector.step_length << std::endl;
                    double newStepLength = std::stod(stepLength_str);
                    gaitSelector.step_length = newStepLength;   
                    std::cout << "Setting step length : " << gaitSelector.step_length << std::endl;
                    // Update your system’s step length accordingly.
                } catch (const std::exception& e) {
                    std::cerr << "Invalid step length parameter: " << stepLength_str << std::endl;
                }
                return;
            } else if (command[0] == 'h' && command.size() > 1) {
                std::string standHeight_str = command.substr(1);
                try {
                    // see the gait and use the function od the wheel mode just set
                    std::cout << "Origin stand height : " << gaitSelector.stand_height << std::endl;
                    double newStandHeight = std::stod(standHeight_str);
                    gaitSelector.stand_height = newStandHeight;
                    std::cout << "Setting stand height : " << gaitSelector.stand_height << std::endl;
                    // Update system standing height here.
                } catch (const std::exception& e) {
                    std::cerr << "Invalid stand height parameter: " << standHeight_str << std::endl;
                }
                return;
            } else if (command[0] == 'j') {
                // Change lift height command: e.g., "j0.2"
                std::string stepHeight_str = command.substr(1);
                try {
                    // see the gait and use the function od the wheel mode just set
                    std::cout << "Origin lift height : " << gaitSelector.step_height << std::endl;
                    double newStepHeight = std::stod(stepHeight_str);
                    gaitSelector.step_height = newStepHeight;
                    std::cout << "Setting lift height : " << gaitSelector.step_height << std::endl;
                    // Update system lift height here.
                } catch (const std::exception& e) {
                    std::cerr << "Invalid lift height parameter: " << stepHeight_str << std::endl;
                }
                return;
            } else {
                std::cerr << "Unknown command: " << command << std::endl;
                return;
            }
            
            // If the command resulted in a gait change, do the gait transformation.
            if (gaitSelector.newGait != gaitSelector.currentGait) {
                printCurrentGait();
                GaitTransform();
                gaitSelector.currentGait = gaitSelector.newGait;
                printCurrentGait();
            }
        }
        
        
        void printCurrentGait() const {
            switch (gaitSelector.currentGait) {
            case Gait::WHEELED:
                std::cout << "Current gait: WHEELED" << std::endl;
                break;
            case Gait::LEGGED:
                std::cout << "Current gait: LEGGED" << std::endl;
                break;
            case Gait::HYBRID:
                std::cout << "Current gait: HYBRID" << std::endl;
                break;
            case Gait::TRANSFORM:
                std::cout << "Current gait: TRANSFORM" << std::endl;
                break;
            default:
                std::cerr << "Unknown gait" << std::endl;
                break;
            }
        }

        void GaitTransform() {
            switch (gaitSelector.currentGait)
            {
            case Gait::WHEELED:
                gaitSelector.currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(gaitSelector.newGait == Gait::LEGGED) {
                    std::cout << "Transforming from WHEELED to LEGGED" << std::endl;
                } 
                else if(gaitSelector.newGait == Gait::HYBRID) {
                    std::cout << "Transforming from WHEELED to HYBRID" << std::endl;
                }
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
            break;
            case Gait::LEGGED:
                gaitSelector.currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(gaitSelector.newGait == Gait::WHEELED) {
                    std::cout << "Transforming from LEGGED to WHEELED" << std::endl;
                } 
                else if(gaitSelector.newGait == Gait::HYBRID) {
                    std::cout << "Transforming from LEGGED to HYBRID" << std::endl;
                } 
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
                break;
            case Gait::HYBRID:
                gaitSelector.currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(gaitSelector.newGait == Gait::WHEELED) {
                    std::cout << "Transforming from HYBRID to WHEELED" << std::endl;
                } 
                else if(gaitSelector.newGait == Gait::LEGGED) {
                    std::cout << "Transforming from HYBRID to LEGGED" << std::endl;
                } 
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
                break;
            
            default:
                break;
            }
            
            std::cout << "Transforming..." << std::endl;
        }
        
        bool joyShutdown = false; // Global flag to signal the joystick thread to exit.
        std::thread joyThread;

        // This is a sample joystick processing loop.
        void joystickProcessing() {
            while (!joyShutdown && ros::ok()) {
                // Process joystick input here.
                // ...
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            std::cout << "Joystick thread shutting down." << std::endl;
        }
};

int main(int argc, char **argv){
    ROS_INFO("Simple FSM Control\n");
    ros::init(argc, argv, "corgi_gait_selector");
    ros::NodeHandle nh;

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();    

    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);
    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);

    
    // Start the keyboard input thread
    KeybordControl keybordControl(nh, gaitSelector);
    // Create a thread for keyboard input
    std::thread input_thread(&KeybordControl::keyboardInputThread, &keybordControl);    

    /*    main loop   */ 
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}





