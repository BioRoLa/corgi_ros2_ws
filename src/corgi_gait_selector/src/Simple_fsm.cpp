#include "Simple_fsm.hpp"

GaitSelector::GaitSelector( ros::NodeHandle& nh, 
                            bool sim, 
                            double CoM_bias, 
                            int pub_rate, 
                            double BL, 
                            double BW, 
                            double BH): 
    leg_model(sim), 
    CoM_bias(CoM_bias), 
    BL(BL), 
    BW(BW), 
    BH(BH), 
    pub_rate(pub_rate),
    rng(rd()), 
    dist(0, 359),
    currentGait(Gait::WHEELED)
{
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &GaitSelector::motor_state_cb, this);
    motor_cmd_pub_ = nh.advertise<corgi_msgs::MotorCmdStamped>("/motor/command", pub_rate);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("stable_triangle", pub_rate);
    rate_ptr = new ros::Rate(pub_rate);

    // Initialize dS & incre_duty
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;  

    // in leg frame
    for (int i = 0; i < 4; ++i) {
        relative_foothold[i][0] = -stand_height;
        relative_foothold[i][1] = 0;
    }

    // in robot frame
    body = {0, 0, stand_height};
    hip = {{{body[0]+BL/2, body[1]+BW/2, body[2]},
            {body[0]+BL/2, body[1]-BW/2, body[2]},
            {body[0]-BL/2, body[1]-BW/2, body[2]},
            {body[0]-BL/2, body[1]+BW/2, body[2]}}};

    foothold = {{{hip[0][0] - relative_foothold[0][1], hip[0][1], hip[0][2] + relative_foothold[0][0]},
                 {hip[1][0] - relative_foothold[1][1], hip[1][1], hip[1][2] + relative_foothold[1][0]},
                 {hip[2][0] - relative_foothold[2][1], hip[2][1], hip[2][2] + relative_foothold[2][0]},
                 {hip[3][0] - relative_foothold[3][1], hip[3][1], hip[3][2] + relative_foothold[3][0]}}};

    

    next_body = body;
    next_hip = hip;
    next_foothold = foothold;

    // cout the above
    // std::cout << "body: " << body[0] << ", " << body[1] << ", " << body[2] << std::endl;
    // for (int i = 0; i < 4; ++i) {
    //     std::cout << "---------------------------------------------------------------------------------" << std::endl;
    //     std::cout << "leg: "<< i << std::endl;
    //     std::cout << "relative_foothold: " << relative_foothold[i][0] << ", " << relative_foothold[i][1] << std::endl;
    //     std::cout << "hip: " << hip[i][0] << ", " << hip[i][1] << ", " << hip[i][2] << std::endl;
    //     std::cout << "foothold: " << foothold[i][0] << ", " << foothold[i][1] << ", " << foothold[i][2] << std::endl;
    // }
    // std::cout << "---------------------------------------------------------------------------------" << std::endl;

}

GaitSelector::~GaitSelector() {
    delete rate_ptr;
    rate_ptr = nullptr;
}



void GaitSelector::setCmd(std::array<double, 2> send, int index, bool dir) {
    if (dir==true){
        motor_cmd_modules[index]->beta  = -send[1];
    }
    else{
        motor_cmd_modules[index]->beta  = send[1];
    }
    motor_cmd_modules[index]->theta = send[0];
    motor_cmd_modules[index]->kp_r = 150;
    motor_cmd_modules[index]->ki_r = 0;
    motor_cmd_modules[index]->kd_r = 1.75;
    motor_cmd_modules[index]->kp_l = 150;
    motor_cmd_modules[index]->ki_l = 0;
    motor_cmd_modules[index]->kd_l = 1.75;
}

void GaitSelector::publish(int freq) {
    for (int i = 0; i < freq; i++) {
        motor_cmd_pub_.publish(motor_cmd);
        rate_ptr->sleep();
    }
}

void GaitSelector::Send(int freq){
    for(int i =0; i<4; i++){
        // std::cout << i << ": " <<current_eta[i][0]*180.0/M_PI << ", "<< current_eta[i][1]*180.0/M_PI << std::endl;
        // std::cout <<"Send"<<std::endl;
        std::array<double, 2> tmp = { eta[i][0], eta[i][1] };
        if (i==1 || i==2) {
            setCmd(tmp, i, true);
        } else {
            setCmd(tmp, i, false);
        }     
    }
    publish(freq);
}

void GaitSelector::motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
    // std::cout << "motor_state: " << std::endl;
}  






void GaitSelector::Transfer(int pub, int transfer_sec, int wait_sec){
    // next_eta = target and grep current_motor_pose -> devided to step until current_eta
    // transfer
    std::vector<std::vector<double>> theta_steps(4), beta_steps(4);
    for (int i=0; i<4; i++){
        theta_steps[i] = linspace(motor_state_modules[i]->theta, next_eta[i][0], transfer_sec*pub_rate);
        beta_steps[i]  = linspace(motor_state_modules[i]->beta,  next_eta[i][1], transfer_sec*pub_rate);
    }
    for (int step_i = 0; step_i < transfer_sec*pub_rate; step_i++) {
        for  (int i=0; i<4; i++){
            eta[i][0] = theta_steps[i][step_i];
            eta[i][1] = beta_steps[i][step_i];
        }       
        if (pub){
            Send(1);
        }
    }

    // wait
    for (int step_i = 0; step_i < wait_sec*pub_rate; step_i++) {
        if (pub){
            Send(1);
        }
    }

}

void GaitSelector::Receive(){
    // eta store the current motor pose
    for (int i=0; i<4; i++){
        eta[i][0] = motor_state_modules[i]->theta;
        eta[i][1]  = motor_state_modules[i]->beta;
        if (i==1 || i==2) {
            eta[i][1] = -eta[i][1];
        }
    }
    // for (int i = 0; i < 4; i++){
    //     std::cout << i << ": " 
    //               << eta[i][0] * 180 / M_PI << ", " 
    //               << eta[i][1] * 180 / M_PI << std::endl;
    // }
}

std::vector<double> GaitSelector::linspace(double start, double end, int num_steps) {
    std::vector<double> result;
    if (num_steps < 1) return result; 
    
    result.resize(num_steps);
    if (num_steps == 1) {
        // Only one step -> just start
        result[0] = start;
        return result;
    }
    
    double step = (end - start) / (num_steps - 1);
    for (int i = 0; i < num_steps; ++i) {
        result[i] = start + step * i;
    }
    return result;
}

// Initialize static variables
std::array<double, 4> GaitSelector::duty = {0.0};
std::array<int, 4> GaitSelector::swing_phase = {0};

double GaitSelector::swing_time = 0.2;   
double GaitSelector::velocity = 0.05;  
double GaitSelector::stand_height = 0.15;
double GaitSelector::step_length = 0.4; 
double GaitSelector::step_height = 0.03; 

double GaitSelector::curvature = 0.0; // +: turn left, -:turn right, 0: straight

std::array<double, 4> GaitSelector::current_step_length = {step_length, step_length, step_length, step_length};
std::array<double, 4> GaitSelector::next_step_length    = {step_length, step_length, step_length, step_length};
double GaitSelector::new_step_length = step_length;
std::array<double, 4> GaitSelector::current_shift = {-0.03, -0.03, -0.03, -0.03};
double GaitSelector::relative_foothold[4][2] = {0.0};
double GaitSelector::eta[4][2] = {0.0};
double GaitSelector::next_eta[4][2]= {0.0};
std::array<std::array<double, 3>, 4> GaitSelector::foothold= {0.0};
std::array<std::array<double, 3>, 4> GaitSelector::next_foothold=foothold;
std::array<double, 3>GaitSelector::body= {0.0};
std::array<double, 3>GaitSelector::next_body = body;
std::array<std::array<double, 3>, 4> GaitSelector::hip = {0.0};
std::array<std::array<double, 3>, 4> GaitSelector::next_hip = hip;

// For turning 
double GaitSelector::outer_radius = 0.0;
double GaitSelector::inner_radius = 0.0;
double GaitSelector::diff_step_length = 0.0;  // Differential step length 
double GaitSelector::new_diff_step_length = 0.0;  // New differential step length
double GaitSelector::diff_dS = 0.0;   // Differential dS
int GaitSelector::sign_diff[4] = {0.0};   // Differential sign

corgi_msgs::MotorStateStamped GaitSelector::motor_state = corgi_msgs::MotorStateStamped();

std::vector<corgi_msgs::MotorState*> GaitSelector::motor_state_modules = {
    &GaitSelector::motor_state.module_a,
    &GaitSelector::motor_state.module_b,
    &GaitSelector::motor_state.module_c,
    &GaitSelector::motor_state.module_d
};

int GaitSelector::pub_time = 1;
int GaitSelector::do_pub = 1;
int GaitSelector::transfer_state = 1;
int GaitSelector::transfer_sec = 5;
int GaitSelector::wait_sec = 2;

int GaitSelector::direction = 1;

    

// add statecallback
//  bool sim=true,
// double CoM_bias=0.0,
// int pub_rate=1000,
// double BL=0.444,
// double BW=0.4,
// double BH=0.2


// wheel add pub rate
// add Transfer


// void GaitSelector::process_and_visualize() {
//     std::vector<Eigen::Vector3d> contact_points;
//     std::vector<int> support_indices;

//     for (int i = 0; i < 4; ++i) {
//         if (swing_phase[i] == 1) continue;  // In swing phase, skip

//         double theta = motor_state_modules[i]->theta;
//         double beta  = motor_state_modules[i]->beta;
//         std::cout << i << ": " 
//                   << theta * 180 / M_PI << ", " 
//                   << beta * 180 / M_PI << std::endl;

//         leg_model.contact_map(theta, beta, 0);
//         leg_model.forward(theta, beta, true);

//         double x_hip = leg_model.contact_p[0];
//         double y_hip = leg_model.contact_p[1];

//         std::cout << "x_hip: " << x_hip << ", y_hip: " << y_hip << std::endl;

//         Eigen::Vector3d base_pos;
//         switch (i) {
//             case 0: base_pos = Eigen::Vector3d( BL/2,  BW/2, 0); break;
//             case 1: base_pos = Eigen::Vector3d( BL/2, -BW/2, 0); break;
//             case 2: base_pos = Eigen::Vector3d(-BL/2,  BW/2, 0); break;
//             case 3: base_pos = Eigen::Vector3d(-BL/2, -BW/2, 0); break;
//         }

//         Eigen::Vector3d contact_robot;
//         contact_robot[0] = base_pos[0] + x_hip;
//         contact_robot[1] = base_pos[1];
//         contact_robot[2] = base_pos[2] + y_hip;

//         contact_points.push_back(contact_robot);
//         support_indices.push_back(contact_points.size() - 1);  // Track which index
//     }

//     publish_support_polygon(contact_points, support_indices);
//     publish_com_marker();
// }

// void GaitSelector::publish_support_polygon(
//     const std::vector<Eigen::Vector3d>& pts,
//     const std::vector<int>& indices
// ) {
//     // ---- LINE LIST (edges) ----
//     visualization_msgs::Marker line_marker;
//     line_marker.header.frame_id = "map";
//     line_marker.header.stamp = ros::Time::now();
//     line_marker.ns = "support_polygon";
//     line_marker.id = 0;
//     line_marker.type = visualization_msgs::Marker::LINE_LIST;
//     line_marker.action = visualization_msgs::Marker::ADD;
//     line_marker.scale.x = 0.01;
//     line_marker.color.r = 0.0;
//     line_marker.color.g = 1.0;
//     line_marker.color.b = 1.0;
//     line_marker.color.a = 1.0;

//     for (size_t i = 0; i < indices.size(); ++i) {
//         geometry_msgs::Point p1, p2;
//         const auto& pt1 = pts[indices[i]];
//         const auto& pt2 = pts[indices[(i + 1) % indices.size()]];
//         p1.x = pt1[0]; p1.y = pt1[1]; p1.z = pt1[2];
//         p2.x = pt2[0]; p2.y = pt2[1]; p2.z = pt2[2];
//         line_marker.points.push_back(p1);
//         line_marker.points.push_back(p2);
//     }

//     marker_pub_.publish(line_marker);

//     // ---- TRIANGLE LIST (filled region) ----
//     visualization_msgs::Marker fill_marker;
//     fill_marker.header.frame_id = "map";
//     fill_marker.header.stamp = ros::Time::now();
//     fill_marker.ns = "support_polygon";
//     fill_marker.id = 1;
//     fill_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
//     fill_marker.action = visualization_msgs::Marker::ADD;
//     fill_marker.scale.x = 1.0;  // not used for triangles
//     // fill_marker.color.r = 0.0;
//     // fill_marker.color.g = 1.0;
//     // fill_marker.color.b = 1.0;
//     // fill_marker.color.a = 0.3;  // semi-transparent

//     if (indices.size() == 3) {
//         // One triangle
//         for (int i = 0; i < 3; ++i) {
//             geometry_msgs::Point pt;
//             const auto& p = pts[indices[i]];
//             pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
//             fill_marker.points.push_back(pt);
//         }
//     } else if (indices.size() == 4) {
//         // Two triangles from quad: (0,1,2) and (0,2,3)
//         geometry_msgs::Point quad[4];
//         for (int i = 0; i < 4; ++i) {
//             const auto& p = pts[indices[i]];
//             quad[i].x = p[0]; quad[i].y = p[1]; quad[i].z = p[2];
//         }
//         fill_marker.points.push_back(quad[0]);
//         fill_marker.points.push_back(quad[1]);
//         fill_marker.points.push_back(quad[2]);

//         fill_marker.points.push_back(quad[0]);
//         fill_marker.points.push_back(quad[2]);
//         fill_marker.points.push_back(quad[3]);
//     }
//     // Assume COM at center of body projected on XY
//     Eigen::Vector2d com_xy(0.0, 0.0);
//     bool com_inside = is_point_inside_polygon(pts, indices, com_xy);
//     fill_marker.color.r = com_inside ? 0.0 : 1.0;
//     fill_marker.color.g = com_inside ? 1.0 : 0.0;
//     fill_marker.color.b = com_inside ? 1.0 : 0.0;
//     fill_marker.color.a = 0.3;  // semi-transparent

//     marker_pub_.publish(fill_marker);
// }
// void GaitSelector::publish_com_marker() {
//     visualization_msgs::Marker com;
//     com.header.frame_id = "map";
//     com.header.stamp = ros::Time::now();
//     com.ns = "com_marker";
//     com.id = 1;
//     com.type = visualization_msgs::Marker::SPHERE;
//     com.action = visualization_msgs::Marker::ADD;
//     com.scale.x = 0.03;
//     com.scale.y = 0.03;
//     com.scale.z = 0.03;
//     com.color.r = 1.0;
//     com.color.g = 0.0;
//     com.color.b = 0.0;
//     com.color.a = 1.0;

//     // COM assumed to be center of base
//     com.pose.position.x = 0.0;
//     com.pose.position.y = 0.0;
//     com.pose.position.z = - stand_height;
//     com.pose.orientation.w = 1.0;

//     marker_pub_.publish(com);
// }


// bool GaitSelector::is_point_inside_polygon(
//     const std::vector<Eigen::Vector3d>& points,
//     const std::vector<int>& indices,
//     const Eigen::Vector2d& point_xy
// ) {
//     int crossings = 0;
//     size_t n = indices.size();

//     for (size_t i = 0; i < n; ++i) {
//         const auto& a = points[indices[i]];
//         const auto& b = points[indices[(i + 1) % n]];

//         Eigen::Vector2d pa(a[0], a[1]);
//         Eigen::Vector2d pb(b[0], b[1]);

//         if (((pa[1] > point_xy[1]) != (pb[1] > point_xy[1])) &&
//             (point_xy[0] < (pb[0] - pa[0]) * (point_xy[1] - pa[1]) / (pb[1] - pa[1] + 1e-6) + pa[0])) {
//             crossings++;
//         }
//     }

//     return (crossings % 2 == 1);
// }

