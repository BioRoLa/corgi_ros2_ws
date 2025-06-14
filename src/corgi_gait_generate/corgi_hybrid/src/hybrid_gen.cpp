#include "hybrid_gen.hpp"

const double PI = M_PI;

TerrainInfo::TerrainInfo(TerrainType type)
: type_(type)
, startX_(0.0), startY_(0.0)
, segmentLength_(1.0), slopeAngleRad_(0.0)
, numSegments_(0)
, plainHeight_(0.0){
    if (type_ == TerrainType::Zigzag) {
        generateZigzagPoints();
    }
}

void TerrainInfo::setTerrainType(TerrainType type) {
    type_ = type;
    if (type_ == TerrainType::Zigzag) {
        generateZigzagPoints();
    }
}

void TerrainInfo::setZigzagParameters(double startX, double startY,
                                      double segmentLength,
                                      double slopeDeg,
                                      int numSegments)
{
    startX_        = startX;
    startY_        = startY;
    segmentLength_ = segmentLength;
    slopeAngleRad_ = slopeDeg * M_PI / 180.0;
    numSegments_   = numSegments;
    generateZigzagPoints();
}

void TerrainInfo::setPlainHeight(double height) {
    plainHeight_ = height;
}

void TerrainInfo::setSlopeAngle(double slopeDeg) {
    // global linear slope: y = startY_ + tan(slopeAngle)*(x - startX_)
    slopeAngleRad_ = slopeDeg * M_PI / 180.0;
}

void TerrainInfo::generateZigzagPoints() {
    zigzagPoints_.clear();
    zigzagPoints_.reserve(numSegments_ + 1);
    zigzagPoints_.emplace_back(startX_, startY_);

    for (int i = 0; i < numSegments_; ++i) {
        double angle = (i % 2 == 0) ? slopeAngleRad_ : -slopeAngleRad_;
        double dx    = segmentLength_;
        double dy    = std::tan(angle) * segmentLength_;
        auto [x0,y0] = zigzagPoints_.back();
        zigzagPoints_.emplace_back(x0 + dx, y0 + dy);
    }
}

double TerrainInfo::getTerrainHeight(double x) const {
    switch (type_) {
        case TerrainType::Zigzag: {
            if (zigzagPoints_.empty()) return 0.0;
            // 找到 x 落在哪一段
            auto it = std::upper_bound(
                zigzagPoints_.begin(), zigzagPoints_.end(), x,
                [](double value, const std::pair<double,double>& pt){
                    return value < pt.first;
                });
            if (it == zigzagPoints_.begin()) {
                return zigzagPoints_.front().second;
            }
            if (it == zigzagPoints_.end()) {
                return zigzagPoints_.back().second;
            }
            // 線性插值
            auto [x1,y1] = *(it-1);
            auto [x2,y2] = *it;
            double t = (x - x1) / (x2 - x1);
            return y1 + t * (y2 - y1);
        }
        case TerrainType::Plain:
            return plainHeight_;
        case TerrainType::Slope:
            // 以 startX_, startY_ 作為基準
            return startY_ + std::tan(slopeAngleRad_) * (x - startX_);
        case TerrainType::Obstacle:
            // TODO: 根據需求自行實作障礙
            return plainHeight_;
        default:
            return 0.0;
    }
}

double TerrainInfo::getTargetHeight(double x, double standHeight) const {
    double terrainY = getTerrainHeight(x);
    return standHeight - terrainY;
}


Hybrid::Hybrid(std::shared_ptr<GaitSelector> gait_selector_ptr)
    : gaitSelector(gait_selector_ptr) {
}

std::array<double, 2> Hybrid::find_pose(double height, float shift, float steplength, double slope){
    std::array<double, 2> pose;
    double pos[2] = {0, -height + gaitSelector->leg_model.r};
    pose = gaitSelector->leg_model.inverse(pos, "G");
    if (shift + steplength >= 0) {
        for (double i = 0; i < shift + steplength; i += 0.001) {
            pose = gaitSelector->leg_model.move(pose[0], pose[1], {0.001, 0},0); //, slope
        }
    } else {
        for (double i = 0; i > shift + steplength; i -= 0.001) {
            pose = gaitSelector->leg_model.move(pose[0], pose[1], {-0.001, 0},0);
        }
    }
    return pose;
}
 
void Hybrid::Initialize(int swing_index, int set_type) {
    // set_type = set original pose or not
    if(set_type){
        // 1>3>0>2, swing_index = who swings first
        switch (swing_index) {
            case 0:
            {
                gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
                break;
            }
            case 1:
            {
                gaitSelector->duty = {0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.0, 0.5};
                break;
            }
            case 2:
            {
                gaitSelector->duty = {0.5 - 2 * gaitSelector->swing_time, 1 - 2 * gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time};
                break;
            }
            case 3:
            {
                gaitSelector->duty = {1 - 2 * gaitSelector->swing_time, 0.5 - 2 * gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time};
                break;
            }
            default:
                for (int k=0 ; k<4 ; k++){
                    gaitSelector->eta[k][0] = 0.0; 
                    gaitSelector->eta[k][1] = 0.0;
                }
                break;
        }
                
        for(int i =0; i<4;i++){
            // std::cout << "gaitSelector->duty[" << i << "] = " << gaitSelector->duty[i] << std::endl;
            auto tmp0 = find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], (gaitSelector->step_length/2) - (gaitSelector->duty[i]/(1-gaitSelector->swing_time)) * gaitSelector->step_length, terrain_slope);
            gaitSelector->next_eta[i][0] = tmp0[0];
            gaitSelector->next_eta[i][1] = tmp0[1];
        }

    }
    else{
        std::cout << "Read current pose and try set a gaitSelector->duty from guess the leg" << std::endl;
        // read current pose
        sleep(1);
        gaitSelector->Receive();
        // tune all the gaitSelector->eta[i][1] between -pi ~ PI
        for (int i=0; i<4; i++){
            if (gaitSelector->eta[i][1] > PI){
                gaitSelector->eta[i][1] -= 2*PI;
            }
            else if (gaitSelector->eta[i][1] < -PI){
                gaitSelector->eta[i][1] += 2*PI;
            }
        }
        // set gaitSelector->duty by checking beta -> choose the biggest and set swing_index to it
        int swing_index = 0;
        double max_beta = gaitSelector->eta[0][1];
        for (int i=1; i<4; i++){
            if (gaitSelector->eta[i][1] > max_beta){
                max_beta = gaitSelector->eta[i][1];
                swing_index = i;
            }
        }   
        // std::cout << "--------------------------" << std::endl;
        std::cout << "Swing leg index selected: " << swing_index << std::endl;
        gaitSelector->Receive();
        // 1>3>0>2, swing_index = who swings first
        switch (swing_index) {
            case 0:
            {
                gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
                break;
            }
            case 1:
            {
                gaitSelector->duty = {0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.0, 0.5};
                break;
            }
            case 2:
            {
                gaitSelector->duty = {0.5 - 2 * gaitSelector->swing_time, 1 - 2 * gaitSelector->swing_time, 1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time};
                break;
            }
            case 3:
            {
                gaitSelector->duty = {1 - 2 * gaitSelector->swing_time, 0.5 - 2 * gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 1 - gaitSelector->swing_time};
                break;
            }
            default:
                for (int k=0 ; k<4 ; k++){
                    gaitSelector->eta[k][0] = 0.0; 
                    gaitSelector->eta[k][1] = 0.0;
                }
                break;
        }
        
        gaitSelector->swing_phase = {0};  
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                gaitSelector->next_eta[i][j] = gaitSelector->eta[i][j];
            }
        }   
    }    

    
    if (set_type){
        if (gaitSelector->transfer_state){
            gaitSelector->Transfer(gaitSelector->do_pub, gaitSelector->transfer_sec, gaitSelector->wait_sec);
        }
        else{
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 2; j++) {
                    gaitSelector->eta[i][j] = gaitSelector->next_eta[i][j];
                }
            }
            if(gaitSelector->do_pub){
                gaitSelector->pub_time = 10;
                gaitSelector->Send(gaitSelector->pub_time);
                gaitSelector->pub_time = 1;
            }
        }
    }   
    // next_body, next_hip, next_foothold, relative_foothold
    update_nextFrame();
    gaitSelector->body = gaitSelector->next_body;
    gaitSelector->hip = gaitSelector->next_hip;
    gaitSelector->foothold = gaitSelector->next_foothold;
}



void Hybrid::Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
    double startX = relative[swing_leg][0];
    double startY = relative[swing_leg][1];
    double endX   = target[0];
    double endY   = target[1];

    // 不一定都需要
    while(endY < startY) {
        endY += 2.0 * PI;
    } 

    gaitSelector->leg_model.forward(endX, endY, false);
    gaitSelector->leg_model.contact_map(endX, endY);
    int rim_id = gaitSelector->leg_model.rim;
    double alpha0 = gaitSelector->leg_model.alpha;
    Eigen::Vector2d body_velocity = {gaitSelector->velocity,0};
    Eigen::Vector2d terrain = {cos(terrain_slope), sin(terrain_slope)};
    // Eigen::Vector2d terrain = {1,0};
    // 儲存一條完整 swing 軌跡
    swing_traj[swing_leg] = HybridSwing::generate(gaitSelector->leg_model, swing_type, startX, endX, startY, endY, rim_id, alpha0, body_velocity, terrain, 100);

    variation[0] = endX - startX;
    variation[1] = endY - startY;
    target[0] = startX;
    target[1] = startY;
}

void Hybrid::Swing_step(std::array<double, 2> target, std::array<double, 2> variation, int swing_leg, double duty_ratio){
    double ratio = (duty_ratio - (1-gaitSelector->swing_time)) / gaitSelector->swing_time;
    ratio = clamp(ratio, 0.0, 1.0); 

    int idx = static_cast<int>(ratio * 100);
    if (idx >= 100) idx = 100;

    gaitSelector->eta[swing_leg][0] = swing_traj[swing_leg][idx].theta;
    gaitSelector->eta[swing_leg][1] = swing_traj[swing_leg][idx].beta;
}

void Hybrid::Step(){
    for (int i=0; i<4; i++) {
        gaitSelector->next_hip[i][0] += gaitSelector->dS + gaitSelector->sign_diff[i]*gaitSelector->diff_dS;
        gaitSelector->duty[i] += gaitSelector->incre_duty;    
    }

    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (gaitSelector->duty[i] < 0){ 
            gaitSelector->duty[i] += 1.0; 
        }

        /* Calculate next foothold if entering swing phase*/
        // Enter SW (calculate swing phase traj)
        if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
            gaitSelector->swing_phase[i] = 1;
            double total_step_length; // step length considering differential
            double swing_hip_move_d; // hip moving distance during swing phase
            // change to new step length when front leg start to swing
            // front leg swing
            if ( ((gaitSelector->direction == 1) && (i==0 || i==1)) || ((gaitSelector->direction == -1) && (i==2 || i==3)) )   
            {  
                // apply new step length and differential
                gaitSelector->next_step_length[i] = gaitSelector->new_step_length;   
                double rest_time = (1.0 - 4*gaitSelector->swing_time) / 2;
                total_step_length = gaitSelector->step_length + gaitSelector->sign_diff[i]*gaitSelector->diff_step_length;
                swing_hip_move_d = gaitSelector->direction * gaitSelector->swing_time * total_step_length;
                // TDL 
                // foothold[i] = {next_hip[i][0] + direction*((1-swing_time)/2)*(new_step_length + sign_diff[i]*new_diff_step_length) + swing_hip_move_d + (rest_time*(step_length - new_step_length)) + CoM_bias, 0};    // half distance between leave and touch-down position (in hip coordinate) + distance hip traveled during swing phase + hip travel difference during rest time because different incre_duty caused by change of step length + CoM_bias.
                // gaitSelector->foothold[i] = {gaitSelector->next_hip[i][0] + ((1-gaitSelector->swing_time)/2)*(gaitSelector->new_step_length ) + (gaitSelector->swing_time)*(gaitSelector->step_length ) + (rest_time*(gaitSelector->step_length - gaitSelector->new_step_length)), 0};   
                gaitSelector->diff_step_length = gaitSelector->new_diff_step_length;
            }
            // hind leg swing
            else {    
                int last_leg = (i+2) % 4;   // Contralateral front leg 對側
                // apply hind step length corresponding to the front leg's.
                gaitSelector->step_length = gaitSelector->current_step_length[last_leg];
                gaitSelector->next_step_length[i] = gaitSelector->step_length;   
                total_step_length = gaitSelector->step_length + gaitSelector->sign_diff[i]*gaitSelector->diff_step_length;
                swing_hip_move_d = gaitSelector->direction * gaitSelector->swing_time * total_step_length; 
                // TDL 
                // gaitSelector->foothold[i] = {gaitSelector->next_hip[i][0] + ((1-gaitSelector->swing_time)/2+gaitSelector->swing_time)*(gaitSelector->step_length), 0};
                gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            }


            if (terrain_type == TerrainType::Slope){
                // gaitSelector->duty[i] + step * gaitSelector->incre_duty < 1.0 calculate the step 
                // step = (1.0 - gaitSelector->duty[i]) / gaitSelector->incre_duty; (how to take the integer part)
                std::cout << "swing leg: " << i << std::endl;
                swing_desired_height = gaitSelector->current_stand_height[i] + ((int)((1.0 - gaitSelector->duty[i]) / gaitSelector->incre_duty)) * gaitSelector->dS * tan(terrain_slope); 
                std::cout << "current height: " << gaitSelector->current_stand_height[i] << std::endl;
                std::cout << "desired height: " << swing_desired_height << std::endl;
                swing_pose = find_pose(swing_desired_height, gaitSelector->current_shift[i], (gaitSelector->step_length*3/6), terrain_slope);  
                Swing(gaitSelector->eta, swing_pose, swing_variation, i);
            }
            else{
                swing_pose = find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], (gaitSelector->step_length*3/6), terrain_slope);  
                Swing(gaitSelector->eta, swing_pose, swing_variation, i);
            }
            
            
        } 
        // Enter TD
        else if ((gaitSelector->direction == 1) && (gaitSelector->duty[i] > 1.0)) {                  
            gaitSelector->swing_phase[i] = 0;
            gaitSelector->duty[i] -= 1.0; // Keep gaitSelector->duty in the range [0, 1]
            // if (sp[i].getDirection() == direction){ // if the leg swing a whole swing phase, instead of swing back.
                // step_count[i] += 1;
                gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
            // }//end if
        }else if ((gaitSelector->direction == -1) && (gaitSelector->duty[i] < (1.0-gaitSelector->swing_time))) {    // entering stance phase when velocirty < 0
            gaitSelector->swing_phase[i] = 0;
            // if (sp[i].getDirection() == direction){ // if the leg swing a whole swing phase, instead of swing back.
                // step_count[i] -= 1;
                gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];   
            // }//end if
        }

        /* Calculate next gaitSelector->eta */
        // calculate the nest Stance phase traj
        if (gaitSelector->swing_phase[i] == 0) { 
            gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
            std::array<double, 2> result_eta;
            // result_eta = gaitSelector->leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, 0); 
            result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {-(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, terrain_slope);
            gaitSelector->eta[i][0] = result_eta[0];
            gaitSelector->eta[i][1] = result_eta[1];
        } 
        // read the next Swing phase traj
        else { 
            Swing_step(swing_pose, swing_variation, i, gaitSelector->duty[i]);
        }
        // update the gaitSelector->hip position
        gaitSelector->hip[i] = gaitSelector->next_hip[i];
    }
    
    // gaitSelector->Send gaitSelector->eta 
    if(gaitSelector->do_pub){
        gaitSelector->Send(gaitSelector->pub_time);
    }

}

void Hybrid::change_Height(double new_value, int leg_index){
    // 0-3 specific leg, 4 all legs
    // real-change when in the stance phase
    // gaitSelector->new_step_length = new_value;
    if (leg_index == 4){
        for (int i=0; i<4; i++) {
            // gaitSelector->new_stand_height[leg_index] = new_value;
            gaitSelector->current_stand_height[leg_index] = new_value;
        }
    }
    else{
        // gaitSelector->new_stand_height[leg_index] = new_value;
        gaitSelector->current_stand_height[leg_index] = new_value;
    }
    // add limitation
}

void Hybrid::change_Step_length(double new_value){
    gaitSelector->new_step_length = new_value;
    // add limitation
}

void Hybrid::change_Velocity(double new_value){
    gaitSelector->velocity = new_value;
    gaitSelector->dS = gaitSelector->velocity / gaitSelector->pub_rate;
    gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;  
}

void Hybrid::update_nextFrame(){
    // use next eta 
    for (int i=0; i<4; i++) {
        if(i==1 || i==2) {
            gaitSelector->leg_model.contact_map(gaitSelector->next_eta[i][0], -gaitSelector->next_eta[i][1],0);
            gaitSelector->leg_model.forward(gaitSelector->next_eta[i][0], -gaitSelector->next_eta[i][1],true);
        }
        else{
            gaitSelector->leg_model.contact_map(gaitSelector->next_eta[i][0], gaitSelector->next_eta[i][1],0);
            gaitSelector->leg_model.forward(gaitSelector->next_eta[i][0], gaitSelector->next_eta[i][1],true);
        }
        
        // calculate gaitSelector->relative_foothold
        // in leg frame
        gaitSelector->relative_foothold[i][0] = gaitSelector->leg_model.contact_p[0];
        gaitSelector->relative_foothold[i][1] = gaitSelector->leg_model.contact_p[1];
        // std::cout << "relative_foothold: " << gaitSelector->relative_foothold[i][0] << ", " << gaitSelector->relative_foothold[i][1] << std::endl;
    }
    computeNextBody();

    // in robot frame
    gaitSelector->next_hip = {{{gaitSelector->next_body[0]+gaitSelector->BL/2, gaitSelector->next_body[1]+gaitSelector->BW/2, gaitSelector->next_body[2]},
                               {gaitSelector->next_body[0]+gaitSelector->BL/2, gaitSelector->next_body[1]-gaitSelector->BW/2, gaitSelector->next_body[2]},
                               {gaitSelector->next_body[0]-gaitSelector->BL/2, gaitSelector->next_body[1]-gaitSelector->BW/2, gaitSelector->next_body[2]},
                               {gaitSelector->next_body[0]-gaitSelector->BL/2, gaitSelector->next_body[1]+gaitSelector->BW/2, gaitSelector->next_body[2]}}};

    gaitSelector->next_foothold = {{{gaitSelector->next_hip[0][0] - gaitSelector->relative_foothold[0][1], gaitSelector->next_hip[0][1], gaitSelector->next_hip[0][2] + gaitSelector->relative_foothold[0][0]},
                                    {gaitSelector->next_hip[1][0] - gaitSelector->relative_foothold[1][1], gaitSelector->next_hip[1][1], gaitSelector->next_hip[1][2] + gaitSelector->relative_foothold[1][0]},
                                    {gaitSelector->next_hip[2][0] - gaitSelector->relative_foothold[2][1], gaitSelector->next_hip[2][1], gaitSelector->next_hip[2][2] + gaitSelector->relative_foothold[2][0]},
                                    {gaitSelector->next_hip[3][0] - gaitSelector->relative_foothold[3][1], gaitSelector->next_hip[3][1], gaitSelector->next_hip[3][2] + gaitSelector->relative_foothold[3][0]}}};
}

void Hybrid::computeNextBody() {
    gaitSelector->next_body[0] = gaitSelector->body[0] + gaitSelector->dS;  // move forward
    gaitSelector->next_body[1] = gaitSelector->body[1];       // lateral stays
    double sum_z = 0.0;

    for (int i = 0; i < 4; ++i) {
        double z_i = gaitSelector->relative_foothold[i][1];
        sum_z += -z_i;  // because z_i = hip_z - foot_z → foot_z = hip_z + z_i → hip_z = foot_z - z_i → body_z = foot_z - z_i
    }

    gaitSelector->next_body[2] = sum_z / 4.0;
    // std::cout << "height: " << gaitSelector->next_body[2] << std::endl;
}

double Hybrid::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}
void Hybrid::csv_title(std::ofstream &file) {
    // open the file and keep saving / in no file then open the new one to save
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }
    // write the header
    file << "step, "
         << "eta[0][0], eta[0][1], "
         << "eta[1][0], eta[1][1], "
         << "eta[2][0], eta[2][1], "
         << "eta[3][0], eta[3][1], "
         << "next_eta[0][0], next_eta[0][1], "
         << "next_eta[1][0], next_eta[1][1], "
         << "next_eta[2][0], next_eta[2][1], "
         << "next_eta[3][0], next_eta[3][1], "
         << "relative_foothold.x, relative_foothold.y, "
         << "body.x, body.y, body.z, "
         << "next_body.x, next_body.y, next_body.z, "
         << "hip.x, hip.y, hip.z, "
         << "next_hip.x, next_hip.y, next_hip.z, "
         << "swing_phase[0], swing_phase[1], swing_phase[2], swing_phase[3], "
         << "duty[0], duty[1], duty[2], duty[3]" << std::endl;

}


// save the step, eta, next_eta, relative_foothold, body, next_body, hip, next_hip, swing_phase, duty of each leg to csv "totall_steps.csv"
void Hybrid::save_to_csv(std::ofstream &file, int step) {
    // open the file and keep saving / in no file then open the new one to save
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }
    // // write the header
    // file << "step, "
    //      << "eta[0][0], eta[0][1], "
    //      << "eta[1][0], eta[1][1], "
    //      << "eta[2][0], eta[2][1], "
    //      << "eta[3][0], eta[3][1], "
    //      << "next_eta[0][0], next_eta[0][1], "
    //      << "next_eta[1][0], next_eta[1][1], "
    //      << "next_eta[2][0], next_eta[2][1], "
    //      << "next_eta[3][0], next_eta[3][1], "
    //      << "relative_foothold.x, relative_foothold.y, "
    //      << "body.x, body.y, body.z, "
    //      << "next_body.x, next_body.y, next_body.z, "
    //      << "hip.x, hip.y, hip.z, "
    //      << "next_hip.x, next_hip.y, next_hip.z, "
    //      << "swing_phase[0], swing_phase[1], swing_phase[2], swing_phase[3], "
    //      << "duty[0], duty[1], duty[2], duty[3]" << std::endl;
    // write the data
    file << step << ", ";
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->eta[i][0] << ", " << gaitSelector->eta[i][1] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->next_eta[i][0] << ", " << gaitSelector->next_eta[i][1] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->relative_foothold[i][0] << ", " << gaitSelector->relative_foothold[i][1] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->body[i] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->next_body[i] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->hip[i][0] << ", " << gaitSelector->hip[i][1] << ", " << gaitSelector->hip[i][2] << ", ";
    }
    for (int i = 0; i < 3; i++) {
        file << gaitSelector->next_hip[i][0] << ", " << gaitSelector->next_hip[i][1] << ", " << gaitSelector->next_hip[i][2] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->swing_phase[i] << ", ";
    }
    for (int i = 0; i < 4; i++) {
        file << gaitSelector->duty[i] << ", ";
    }   
    file << std::endl;
    // close the file
    
    // std::cout << "Data saved to file" << std::endl;
    std::cout << "step: " << step << std::endl;
}