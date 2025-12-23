// HybridToLegged.cpp
#include <iostream>
#include "corgi_transform/transform_gen.hpp"
class HybridToLegged : public IGaitTransform 
{
    public:
        HybridToLegged(std::shared_ptr<Hybrid> transform_hybrid_ptr,std::shared_ptr<Legged> transform_leg_ptr)
            : transform_hybrid(transform_hybrid_ptr),inherit(transform_hybrid_ptr->gaitSelector),transform_leg(transform_leg_ptr) {}
        ~HybridToLegged() override = default;
        // Inherit the gaitSelector from Hybrid
        std::shared_ptr<GaitSelector> inherit;
        void transform() override { 
            int count = 0;
            // inherit->Receive();
            inherit->step_length = 0.15;//
            transform_hybrid->update_nextFrame();
            inherit->body = inherit->next_body;
            for (int i=0; i<4; i++) {
                inherit->hip[i] = inherit->next_hip[i];
            }
            inherit->foothold = inherit->next_foothold;
            inherit->dS = inherit->velocity / inherit->pub_rate;
            inherit->incre_duty = inherit->dS / inherit->step_length;
            inherit->stand_height = inherit->current_stand_height[0]-0.002; // use the first leg's height as the stand height
            for (int i=0; i<4; i++) {
                inherit->current_stand_height[i] = inherit->stand_height;
            }
            std::cout << "initial: "<<inherit->stand_height << std::endl;
            // adjust height
            count = 0;
            while(inherit->stand_height < 0.18){
                count = 0;
                for (int i=0; i<4; i++) {
                    if(inherit->swing_phase[i]==0){
                        count++;
                    }
                }
                if(count==4){
                    // std::cout << gaitSelector->stand_height << std::endl;
                    transform_hybrid->change_Height_all(inherit->stand_height+0.0001);
                }
                transform_hybrid->Step();
                inherit->Send();
            
            }
            // 直接接走看看應該可以
            std::cout << "height ready" << std::endl;
            transform_leg->Initial();
            double moving_distance = inherit->step_length;
            double step_count = moving_distance/ inherit->dS;
            std::cout << "moving_distance" << moving_distance <<std::endl;
            std::cout << "step_count" << step_count <<std::endl;
            int leg_count =0;
            for (leg_count;leg_count<step_count;leg_count++) {
                std::cout << leg_count <<" / " << step_count <<std::endl;
                transform_leg->set_step_length(0.2);
                transform_leg->set_stand_height(0.18);
                transform_leg->next_Step();
                inherit->Send();
            }
    }
    private:
        std::shared_ptr<Hybrid> transform_hybrid;
        std::shared_ptr<Legged> transform_leg;
        int check_point=0;
        std::array<int, 4> walk_transform;
        double temp_step_length;
        std::array<double, 2> swing_pose_temp;
        std::array<double, 2> swing_variation_temp;
};     
extern "C" IGaitTransform* createHybridToLegged(std::shared_ptr<Hybrid> hybrid_ptr, std::shared_ptr<Legged> legged_ptr) {
    return new HybridToLegged(hybrid_ptr, legged_ptr);
}