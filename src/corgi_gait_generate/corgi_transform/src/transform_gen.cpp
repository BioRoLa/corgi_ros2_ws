#include <iostream>
#include <memory>
#include "transform_gen.hpp"

// Declare factory functions with C linkage.
extern "C" IGaitTransform* createHybridToWheeled();
extern "C" IGaitTransform* createHybridToLegged();
extern "C" IGaitTransform* createWheeledToHybrid();
extern "C" IGaitTransform* createWheeledToLegged();
extern "C" IGaitTransform* createLeggedToWheeled();
extern "C" IGaitTransform* createLeggedToHybrid();

void Transform::GaitTransform(Gait current, Gait next) {
    std::unique_ptr<IGaitTransform> strategy;
    if (current == Gait::HYBRID && next == Gait::WHEELED) {
        strategy.reset(createHybridToWheeled());
    } else if (current == Gait::HYBRID && next == Gait::LEGGED) {
        strategy.reset(createHybridToLegged());
    } else if (current == Gait::WHEELED && next == Gait::HYBRID) {
        strategy.reset(createWheeledToHybrid());
    } else if (current == Gait::WHEELED && next == Gait::LEGGED) {
        strategy.reset(createWheeledToLegged());
    } else if (current == Gait::LEGGED && next == Gait::WHEELED) {
        strategy.reset(createLeggedToWheeled());
    } else if (current == Gait::LEGGED && next == Gait::HYBRID) {
        strategy.reset(createLeggedToHybrid());
    }

    if (strategy){
        strategy->transform(-0.03,0,0,0.14);
        std::cout << "Transformation finished." << std::endl;
    }else
        std::cout << "Invalid Transform Selection" << std::endl;
}
