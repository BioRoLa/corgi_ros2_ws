#include "transform_gen.hpp"
#include "hybrid_gen.hpp"
#include "wheeled_gen.hpp"
#include "legged_gen.hpp"

int main(int argc, char** argv) {
    ROS_INFO("Transform mode test\n");
    ros::init(argc, argv, "corgi_hybrid_test");
    ros::NodeHandle nh;
    
    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /*     Gait Selector Setting    */ 
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    
    /*    Initialize of each mode   */ 
    // wheeled mode
    WheeledCmd WheeledCmd("pure");
    Wheeled wheeled(nh);
    // legged mode
    Legged legged(nh);
    // hybrid mode 
    Hybrid hybrid(nh);  
    // transform mode
    Transform transformer;
    
    /*   Hybrid   */
    std::cout << "hybrid" << std::endl;
    hybrid.Initialize(2, 10, 1, 0, 5, 2, -0.03);
    for (int step = 0;step<2000;step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        hybrid.Step(1, 1, -0.03);
    } 

    /*  Transform    */
    std::cout << "hybrid to wheeled" << std::endl;
    transformer.GaitTransform(Gait::HYBRID, Gait::WHEELED);

    /*   Wheeled   */
    std::cout << "wheeled" << std::endl;
    for (int step = 0; step < 1000; step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        wheeled.Roll(1, 1, 1, 0, 4095, 0);
    }    
    /*  Transform    */
    std::cout << "wheeled to hybrid" << std::endl;
    transformer.GaitTransform(Gait::WHEELED, Gait::HYBRID);

    /*   Hybrid   */
    std::cout << "hybrid" << std::endl;
    for (int step = 0;step<2000;step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        hybrid.Step(1, 1, -0.03);
    } 
    for (int step = 0;step<10000;step++) {
        // Test change_Height
        if(step <= 8000){
            hybrid.change_Height(0.14 + 0.025*(step)/8000);
        }
        // Test change_Step_length
        if (step >= 5000){
            hybrid.change_Step_length(0.3);
            hybrid.change_Velocity(0.05);
            gaitSelector.motor_cmd.header.seq = step;
            gaitSelector.motor_cmd.header.stamp = ros::Time::now();
            hybrid.Step(1, 1, -0.03);
        }
        else{
            gaitSelector.motor_cmd.header.seq = step;
            gaitSelector.motor_cmd.header.stamp = ros::Time::now();
            hybrid.Step(1, 1, -0.03);
        }
    }
    int check_leg = 0;
    for (int i=0; i<4; i++){
        if (gaitSelector.swing_phase[i]==0){
            check_leg++;
        }
    }
    while (check_leg!=4){
        hybrid.Step(1, 1, -0.03);
        check_leg = 0;
        for (int i=0; i<4; i++){
            if (gaitSelector.swing_phase[i]==0){
                check_leg++;
            }
        }
    }

    /*  Transform    */
    std::cout << "hybrid to legged" << std::endl;
    transformer.GaitTransform(Gait::HYBRID, Gait::LEGGED);
    
    /*   Legged   */
    std::cout << "legged" << std::endl;
    legged.Initial();
    for (int step = 0;step<5000;step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        legged.Step();
    }

    check_leg = 0;
    for (int i=0; i<4; i++){
        if (gaitSelector.swing_phase[i]==0){
            check_leg++;
        }
    }
    while (check_leg!=4){
        legged.Step();
        check_leg = 0;
        for (int i=0; i<4; i++){
            if (gaitSelector.swing_phase[i]==0){
                check_leg++;
            }
        }
    }

    /*  Transform    */
    std::cout << "legged to wheeled" << std::endl;
    transformer.GaitTransform(Gait::LEGGED, Gait::WHEELED);

    /*   Wheeled   */
    std::cout << "wheeled" << std::endl;
    for (int step = 0; step < 1000; step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        wheeled.Roll(1, 1, 1, 0, 4095, 0);
    }   
   
    /*  Transform    */
    std::cout << "wheeled to legged" << std::endl;
    transformer.GaitTransform(Gait::WHEELED, Gait::LEGGED);

    /*   Legged   */
    std::cout << "legged" << std::endl;
    legged.Initial();
    for (int step = 0;step<5000;step++) {
        legged.set_step_length(0.2);
        legged.set_stand_height(0.165);
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        legged.Step();
        }

    check_leg = 0;
    for (int i=0; i<4; i++){
        if (gaitSelector.swing_phase[i]==0){
            check_leg++;
        }
    }
    // 2130
    while (check_leg!=4 || gaitSelector.duty[0]>= 0.8){
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        legged.Step();
        check_leg = 0;
        for (int i=0; i<4; i++){
            if (gaitSelector.swing_phase[i]==0){
                check_leg++;
            }
        }
    }
    /*  Transform    */
    std::cout << "legged to hybrid" << std::endl;
    transformer.GaitTransform(Gait::LEGGED, Gait::HYBRID);

    /*   Hybrid   */
    std::cout << "hybrid" << std::endl;
    for (int step = 0;step<5000;step++) {
        gaitSelector.motor_cmd.header.seq = step;
        gaitSelector.motor_cmd.header.stamp = ros::Time::now();
        hybrid.Step(1, 1, -0.03);
    } 
    
    check_leg = 0;
    for (int i=0; i<4; i++){
        if (gaitSelector.swing_phase[i]==0){
            check_leg++;
        }
    }
    while (check_leg!=4){
        hybrid.Step(1, 1, -0.03);
        check_leg = 0;
        for (int i=0; i<4; i++){
            if (gaitSelector.swing_phase[i]==0){
                check_leg++;
            }
        }
    }

    std::cout << "End of testing!" << std::endl;
    return 0;
}
