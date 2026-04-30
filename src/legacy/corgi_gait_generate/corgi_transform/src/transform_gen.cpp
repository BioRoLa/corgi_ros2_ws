#include <iostream>
#include <memory>
#include "corgi_transform/transform_gen.hpp"

IGaitTransform* createWheeledToHybrid(std::shared_ptr<Hybrid> hybrid_ptr,std::shared_ptr<Legged> legged_ptr);
IGaitTransform* createHybridToLegged(std::shared_ptr<Hybrid> hybrid_ptr,std::shared_ptr<Legged> legged_ptr);
IGaitTransform* createWheeledToLegged(std::shared_ptr<Hybrid> hybrid_ptr,std::shared_ptr<Legged> legged_ptr);

void Transform::GaitTransform(Gait current, Gait next) {
    std::unique_ptr<IGaitTransform> strategy;
    if (current == Gait::WHEELED && next == Gait::HYBRID) {
        strategy.reset(createWheeledToHybrid(hybrid_ptr_,legged_ptr_));
    } 
    else if (current == Gait::HYBRID && next == Gait::LEGGED) {
        strategy.reset(createHybridToLegged(hybrid_ptr_,legged_ptr_));
    }
    else if (current == Gait::WHEELED && next == Gait::LEGGED) {
        strategy.reset(createWheeledToLegged(hybrid_ptr_,legged_ptr_));
    }
    if (strategy){
        strategy->transform();
        std::cout << "Transformation finished." << std::endl;
    } else {
        std::cout << "Invalid Transform Selection" << std::endl;
    }
}