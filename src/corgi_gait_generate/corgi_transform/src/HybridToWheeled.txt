// HybridToWheeled.cpp
#include <iostream>
#include "transform_gen.hpp"

class HybridToWheeled : public IGaitTransform, public Hybrid
{
    public:
        HybridToWheeled(ros::NodeHandle& nh) : Hybrid(nh),check_theta(0), check_phase(0), wheel_delta_beta(0.0) {}
        void transform(double shift, int wait_step, bool transfer_state, double expect_height) override {       
            while(check_theta<4){
                check_theta=0;
                check_phase=0;
                for(int i = 0; i < 4; i++){
                    if (swing_phase[i]==0){
                        check_phase++;
                    }
                }
                if(check_phase==4){
                    if(stand_height>=leg_model.radius+0.0001){
                        change_Height(stand_height- 0.0001);
                    }   
                    Step(1, 1, shift);
                    for(int i = 0; i < 4; i++){
                        if(eta[i][0]<=18*PI/180){
                            check_theta++;
                        }
                    }
                }
                else{
                    Step(1, 1, shift);
                }
            }
            for(int i=0;i<(wait_step*pub_rate);i++){
                wheel_delta_beta = velocity/(leg_model.radius * pub_rate);
                for(int i = 0; i < 4; i++){
                    eta[i][0] = 17.0 * PI / 180.0;
                    eta[i][1] = eta[i][1]+wheel_delta_beta;
                }
                Send(1);
            }
        }
    private:
        int check_theta;
        int check_phase;
        double wheel_delta_beta;
};

extern "C" IGaitTransform* createHybridToWheeled() {
    ros::NodeHandle nh;
    return new HybridToWheeled(nh);
}


// IGaitTransform* transformer = createHybridToLegged();
// transformer->transform(-0.03, 5); // pass appropriate shift and wait_step values
// delete transformer;