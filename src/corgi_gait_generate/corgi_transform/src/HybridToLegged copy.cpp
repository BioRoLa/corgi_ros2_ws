// HybridToLegged.cpp
#include <iostream>
#include "transform_gen.hpp"
class HybridToLegged : public IGaitTransform 
{
    public:
        HybridToLegged(std::shared_ptr<Hybrid> transform_hybrid_ptr)
            : transform_hybrid(transform_hybrid_ptr),inherit(transform_hybrid_ptr->gaitSelector) {}
        ~HybridToLegged() override = default;
        // Inherit the gaitSelector from Hybrid
        std::shared_ptr<GaitSelector> inherit;
        void transform() override { 
            inherit-> duty = {1 - inherit->swing_time, 0.5 - inherit->swing_time, 0.5, 0.0};
            for(int i =0; i<4;i++){
                inherit->current_stand_height[i] = 0.159;
                inherit->current_shift[i] = 0.0;
                auto tmp0 = transform_hybrid->find_pose(inherit->current_stand_height[i], inherit->current_shift[i], (inherit->step_length/2) ,inherit-> duty[i], 0);
                inherit->next_eta[i][0] = tmp0[0];
                inherit->next_eta[i][1] = tmp0[1];
            }
            // std::cout << "Climbing the first slope..." << std::endl;
            // // set initial pose (terrain start x = 0-0.13/2 = -0.065)
            // auto pose = transform_hybrid->find_pose(expect_height, 0, 0.5, 0.0 , 0);
            // inherit->leg_model.contact_map(pose[0], pose[1], false);
            // // front leg initial contact = 0.0437007 ()
            // std::cout << -0.065 - (inherit->leg_model.contact_p[0]+ inherit->BL/2)<<" , " <<inherit->leg_model.contact_p[1] << std::endl;
            // inherit->next_eta[1][0] = pose[0];
            // inherit->next_eta[1][1] = pose[1];
            
            
            // for (int i =0;i<10;i++){
            //     for (int j=0;j<4;j++){
            //         if(j!=1){
            //             if (j==0){
            //                 pose = transform_hybrid->find_pose(expect_height, 0, 0.5, 0.5 , 0);
            //                 inherit->next_eta[j][0] = pose[0];
            //                 inherit->next_eta[j][1] = pose[1];
            //             }
            //             else{
            //                 if(j==2){
            //                     pose = transform_hybrid->find_pose(expect_height, 0, 0.5, 0.2 , 0);
            //                     inherit->next_eta[j][0] = pose[0];
            //                     inherit->next_eta[j][1] = pose[1];
            //                 }
            //                 else{
            //                     pose = transform_hybrid->find_pose(expect_height, 0, 0.5, 0.7 , 0);
            //                     inherit->next_eta[j][0] = pose[0];
            //                     inherit->next_eta[j][1] = pose[1];
            //                 }
                            
            //             }
            //         }
            //     }
            //     inherit->Send();
            // }
            // double td_dis = 0.13;
            // double dis = 0.5*inherit->swing_time/(1-inherit->swing_time);
            // double delta_time_step = dis/(inherit->velocity/inherit->pub_rate);
            // int count =0;
            // for (count;count<delta_time_step;count++){
            //     for (int i=0; i<4; i++){
            //         if(i==0 || i==1){
            //             pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {dis/delta_time_step, 0},0.2268);
            //             inherit->next_eta[i][0] = pose[0];
            //             inherit->next_eta[i][1] = pose[1];
            //         }
            //         else{
            //             pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {dis/delta_time_step, 0},0);
            //             inherit->next_eta[i][0] = pose[0];
            //             inherit->next_eta[i][1] = pose[1];
            //         }
                    
            //     }
            //     inherit->Send();
            // }
            // auto swing_end = transform_hybrid->find_pose(expect_height, 0, 0.5, 0.7 , 0);
            // std::cout << "swing_end = " << swing_end[0] << ", " << swing_end[1] << std::endl;
            // std::cout << "inherit->next_eta[2][1] = " << inherit->next_eta[2][0] << ", " << inherit->next_eta[2][1] << std::endl;
            // while(swing_end[1]>inherit->next_eta[3][1]){
            //     swing_end[1]-=2*PI;
            //     std::cout << "swing_end = " << swing_end[0] << ", " << swing_end[1] << std::endl;
            // }
            // // duty = 0.6, 0.1, 0.3, 0.8
            // // swing 2 the first time
            // std::vector<SwingPoint> traj_tr;
            // for (int i = 0; i <= 100; ++i) {
            //     double t = static_cast<double>(i) / 100;
            //     double theta = 0.0, beta = 0.0;

            //     double mid_theta = 17.0 * M_PI / 180.0;
            //     if (t < 0.3) {
            //         theta = inherit->next_eta[3][0] + (mid_theta - inherit->next_eta[3][0]) * (2 * t);
            //     } else if(t>0.6) {
            //         theta = mid_theta + (swing_end[0] - mid_theta) * (2 * (t - 0.6));
            //     }
            //     if (t > 0.3) {
            //         beta  = inherit->next_eta[3][1]  + (swing_end[1] - inherit->next_eta[3][1] ) * (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t);
            //     }
            //     else{
            //         beta  = inherit->next_eta[3][1];
            //     }
                
            //     traj_tr.push_back({t, theta, beta});
            // }
            // std::cout << "traj_tr"<< traj_tr.size() << std::endl;
            // double wheel_delta_beta = inherit->velocity/(inherit->leg_model.radius * inherit->pub_rate);
            // delta_time_step = 0.2/(inherit->velocity/inherit->pub_rate);
            // double bomb = 0.065/((inherit->velocity/inherit->pub_rate));
            // count =0;
            // for (count;count<delta_time_step;count++){
            //     // std::cout << "count = " << count << std::endl;
            //     double phase_ratio = static_cast<double>(count) / delta_time_step;
            //     int idx = static_cast<int>(phase_ratio * 100);
            //     idx = std::min(idx, 99);
            //     for (int i=0; i<4; i++){
                    
            //         if(i==0){
            //             if(delta_time_step >=bomb){
            //                 pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {0.2/delta_time_step, 0},0);
            //                 inherit->next_eta[i][0] = pose[0];
            //                 inherit->next_eta[i][1] = pose[1];
            //             }
            //             else{
            //                 pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {0.2/delta_time_step, 0},0.2268);
            //                 inherit->next_eta[i][0] = pose[0];
            //                 inherit->next_eta[i][1] = pose[1];
            //             }
                        
            //         }
            //         else if(i==1 ){
            //             if(inherit->next_eta[i][0] <=18*PI/180){
            //                 inherit->next_eta[i][0] = 17*PI/180;
            //                 inherit->next_eta[i][1] -= wheel_delta_beta;
            //             }
            //             else{
            //                 pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {0.2/delta_time_step, 0},0);
            //                 inherit->next_eta[i][0] = pose[0];
            //                 inherit->next_eta[i][1] = pose[1];
            //             }
                        
            //         }
            //         else if(i==3){
            //             inherit->next_eta[i][0] = traj_tr[idx].theta;
            //             inherit->next_eta[i][1] = traj_tr[idx].beta;
            //         }
            //         else{
            //             pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {0.2/delta_time_step, 0},0);
            //             inherit->next_eta[i][0] = pose[0];
            //             inherit->next_eta[i][1] = pose[1];
            //         }
                    
            //     }
            //     inherit->Send();
            // }
            // // duty = 0.8, 0.3, 0.5, 0.0
            // // 前輪輪模式, 後輪walk
            // double LF_deltatheta = inherit->next_eta[0][0]-17*PI/180;
            // double RF_deltatheta = inherit->next_eta[1][0]-17*PI/180;
            // delta_time_step = 0.05/(inherit->velocity/inherit->pub_rate);
            // LF_deltatheta = LF_deltatheta/delta_time_step;
            // RF_deltatheta = RF_deltatheta/delta_time_step;
            // count =0;
            // for (count;count<delta_time_step;count++){
            //     for (int i=0; i<4; i++){
            //         if(i==0|| i==1){
            //             if(inherit->next_eta[i][0]<=18*PI/180){
            //                 inherit->next_eta[i][0] = 17*PI/180;
            //                 inherit->next_eta[i][1] -= wheel_delta_beta;
            //             }
            //             else{
            //                 pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {0.05/delta_time_step, -0.001},0);
            //                 inherit->next_eta[i][0] = pose[0];
            //                 inherit->next_eta[i][1] = pose[1];
            //             }
                        
            //         }
            //         else{
            //             pose = inherit->leg_model.move(inherit->next_eta[i][0], inherit->next_eta[i][1], {0.05/delta_time_step, 0},0);
            //             inherit->next_eta[i][0] = pose[0];
            //             inherit->next_eta[i][1] = pose[1];
            //         }
                    
            //     }
            //     inherit->Send();
            //     if(inherit->next_eta[0][0] == 17*PI/180 && inherit->next_eta[1][0] == 17*PI/180){
            //         count = delta_time_step;
            //     }
            // }
            
        }
    private:
        std::shared_ptr<Hybrid> transform_hybrid;
        double expect_height = 0.1425;
};     
extern "C" IGaitTransform* createHybridToLegged(std::shared_ptr<Hybrid> hybrid_ptr) {
    return new HybridToLegged(hybrid_ptr);
}

