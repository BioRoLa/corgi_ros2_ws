// HybridToLegged.cpp
#include <iostream>
#include "transform_gen.hpp"
class HybridToLegged : public IGaitTransform 
{
    public:
        HybridToLegged(std::shared_ptr<Hybrid> transform_hybrid_ptr,std::shared_ptr<Legged> transform_leg_ptr)
            : transform_hybrid(transform_hybrid_ptr),inherit(transform_hybrid_ptr->gaitSelector) {}
        ~HybridToLegged() override = default;
        // Inherit the gaitSelector from Hybrid
        std::shared_ptr<GaitSelector> inherit;
        void transform() override { 
            inherit-> step_length = 0.5;
            inherit-> duty = {1 - inherit->swing_time, 0.5 - inherit->swing_time, 0.5, 0.0};
            inherit->current_shift = {0.0, 0.0, 0.0, 0.0}; // No shift in legged mode
            for(int i =0; i<4;i++){
                inherit->current_stand_height[i] = 0.159;
                auto tmp0 = transform_hybrid->find_pose(inherit->current_stand_height[i], inherit->current_shift[i], (inherit->step_length/2) ,inherit-> duty[i], 0);
                inherit->next_eta[i][0] = tmp0[0];
                inherit->next_eta[i][1] = tmp0[1];
            }
            for (int i =0;i<10;i++){
                inherit->Send();
            }
            // double height = ;
            while(1) {
                for(int i=0;i<4;i++){
                   if(inherit->next_hip[i][0]>0.45 && inherit->next_hip[i][0]<0.95){
                    inherit->current_stand_height[i] = 0.129;
                   }
                   if(inherit->next_hip[i][0]>0.95){
                    inherit->current_stand_height[i] = 0.159;
                   }
                }
                
                transform_hybrid->Step();
                inherit->Send();
            }
        }
    private:
        std::shared_ptr<Hybrid> transform_hybrid;
        double expect_height = 0.1425;
};     
extern "C" IGaitTransform* createHybridToLegged(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr) {
    return new HybridToLegged(hybrid_ptr, legged_ptr);
}