// LeggedToHybrid.cpp
#include <iostream>
#include "transform_gen.hpp"

class LeggedToHybrid : public IGaitTransform,   public Hybrid 
{
    public:
        LeggedToHybrid(ros::NodeHandle& nh) : Hybrid(nh) {}
        void transform(double shift, int wait_step, bool transfer_state, double expect_height) override {
            for (int step = 0;step<5000;step++) {
                change_Height(expect_height);
                motor_cmd.header.seq = step;
                motor_cmd.header.stamp = ros::Time::now();
                Step(1, 1, shift);
            } 
        }
};

extern "C" IGaitTransform* createLeggedToHybrid() {
    ros::NodeHandle nh;
    return new LeggedToHybrid(nh);
}


