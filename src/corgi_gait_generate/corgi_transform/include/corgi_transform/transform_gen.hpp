#ifndef TRANSFORM_GEN_HPP
#define TRANSFORM_GEN_HPP

#include "corgi_gait_selector/Simple_fsm.hpp"
#include "corgi_hybrid/hybrid_gen.hpp"
#include "corgi_wheeled/wheeled_gen.hpp"
#include "corgi_legged/legged_gen.hpp"
#include <cmath>
#include <iostream>

const double PI = M_PI;

// Interface for gait transformation strategies
class IGaitTransform
{
public:
    virtual void transform() = 0;
    virtual ~IGaitTransform() {}
};

#ifdef __cplusplus
extern "C"
{
#endif

    // Factory function declarations with C linkage
    IGaitTransform *createWheeledToHybrid(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr);
    IGaitTransform *createHybridToLegged(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr);
    IGaitTransform *createWheeledToLegged(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr);

#ifdef __cplusplus
}
#endif

// Declaration of the Transform class
class Transform
{
public:
    Transform(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr)
        : hybrid_ptr_(hybrid_ptr), legged_ptr_(legged_ptr) {}

    void GaitTransform(Gait current, Gait next);

private:
    std::shared_ptr<Hybrid> hybrid_ptr_;
    std::shared_ptr<Legged> legged_ptr_;
};
#endif
