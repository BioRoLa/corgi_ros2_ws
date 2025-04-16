#ifndef TRANSFORM_GEN_HPP
#define TRANSFORM_GEN_HPP

#include "Simple_fsm.hpp"  
#include "hybrid_gen.hpp"
#include "wheeled_gen.hpp"
#include "legged_gen.hpp"
#include <iostream>

const double PI = M_PI;

// Interface for gait transformation strategies
class IGaitTransform {
public:
    virtual void transform(double shift, int wait_step, bool transfer_state, double expect_height) = 0;
    virtual ~IGaitTransform() {}
};

#ifdef __cplusplus
extern "C" {
#endif

// Factory function declarations with C linkage
IGaitTransform* createHybridToWheeled();
IGaitTransform* createHybridToLegged();
IGaitTransform* createWheeledToHybrid();
IGaitTransform* createWheeledToLegged();
IGaitTransform* createLeggedToWheeled();
IGaitTransform* createLeggedToHybrid();

#ifdef __cplusplus
}
#endif

// Declaration of the Transform class
class Transform {
public:
    // Selects and executes the appropriate transformation based on current and next gait.
    void GaitTransform(Gait current, Gait next);
};

#endif 
