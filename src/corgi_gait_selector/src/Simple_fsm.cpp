#include "Simple_fsm.hpp"

class GaitSelector {
    public:
        GaitSelector() : currentGait(Gait::WHEEL) {}
        void changeGait(const std::string& command) {
            newGait = currentGait;

            if (command == "1") {
                newGait = Gait::WHEEL;
            } else if (command == "2") {
                newGait = Gait::LEG;
            } else if (command == "3") {
                newGait = Gait::WLW;
            } else if (command[0] == 'v') {
                std::string velocity = command.substr(1);
                std::cout << "Setting velocity to " << velocity << std::endl;
                return;
            } else if (command[0] == 'l') {
                std::string stepLength = command.substr(1);
                std::cout << "Setting step length to " << stepLength << std::endl;
                return;
            } else if (command[0] == 'h') {
                std::string standHeight = command.substr(1);
                std::cout << "Setting stand height to " << standHeight << std::endl;
                return;
            } else if (command[0] == 'j') {
                std::string liftHeight = command.substr(1);
                std::cout << "Setting lift height to " << liftHeight << std::endl;
                return;
            } else {
                std::cerr << "Unknown command: " << command << std::endl;
                return;
            }

            if (newGait != currentGait) {
                printCurrentGait();
                Transform();  
                currentGait = newGait; 
          }
        }
    private:
        Gait currentGait;
        Gait newGait;
        void printCurrentGait() const {
            switch (currentGait) {
            case Gait::WHEEL:
                std::cout << "Current gait: WHEEL" << std::endl;
                break;
            case Gait::LEG:
                std::cout << "Current gait: LEG" << std::endl;
                break;
            case Gait::WLW:
                std::cout << "Current gait: WLW" << std::endl;
                break;
            case Gait::TRANSFORM:
                std::cout << "Current gait: TRANSFORM" << std::endl;
                break;
            default:
                std::cerr << "Unknown gait" << std::endl;
                break;
            }
        }
        void Transform() {
            switch (currentGait)
            {
            case Gait::WHEEL:
                currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(newGait == Gait::LEG) {
                    std::cout << "Transforming from WHEEL to LEG" << std::endl;
                } 
                else if(newGait == Gait::WLW) {
                    std::cout << "Transforming from WHEEL to WLW" << std::endl;
                }
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
            break;
            case Gait::LEG:
                currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(newGait == Gait::WHEEL) {
                    std::cout << "Transforming from LEG to WHEEL" << std::endl;
                } 
                else if(newGait == Gait::WLW) {
                    std::cout << "Transforming from LEG to WLW" << std::endl;
                } 
                else{
                    std::cerr << "Unknown transform gait" << std::endl;
                }
                break;
            case Gait::WLW:
                currentGait = Gait::TRANSFORM;
                printCurrentGait();
                if(newGait == Gait::WHEEL) {
                    std::cout << "Transforming from WLW to WHEEL" << std::endl;
                } 
                else if(newGait == Gait::LEG) {
                    std::cout << "Transforming from WLW to LEG" << std::endl;
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

        
    };


int main(int argc, char **argv){
    GaitSelector gaitSelector;
    std::cout << "Starting gait selector" << std::endl;
    std::cout << "changeGait(2)" << std::endl;
    gaitSelector.changeGait("2");
    std::cout << "changeGait(3)" << std::endl;
    gaitSelector.changeGait("3");
    std::cout << "changeGait(1)" << std::endl;
    gaitSelector.changeGait("1");
    std::cout << "changeGait(v0.5)" << std::endl;
    gaitSelector.changeGait("v0.5");
    std::cout << "changeGait(l0.3)" << std::endl;
    gaitSelector.changeGait("l0.3");
    std::cout << "changeGait(h0.2)" << std::endl;
    gaitSelector.changeGait("h0.2");
    std::cout << "changeGait(j0.1)" << std::endl;
    gaitSelector.changeGait("j0.1");
    std::cout << "changeGait(4)" << std::endl;
    gaitSelector.changeGait("4");
    return 0;
}
