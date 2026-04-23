#include "corgi_odometry.hpp"
#include <geometry_msgs/msg/vector3.hpp>

using namespace estimation_model;

DataProcessor::CsvLogger logger;
DataProcessor::CsvLogger input_logger;

std::vector<std::string> odo_headers = {
        "v_.x", "v_.y", "v_.z", 
        "p.x", "p.y", "p.z", 
        "zLF.x", "zLF.y", "zLF.z", 
        "zRF.x", "zRF.y", "zRF.z", 
        "zRH.x", "zRH.y", "zRH.z", 
        "zLH.x", "zLH.y", "zLH.z", 
        "ba.x", "ba.y", "ba.z", 
        "lf.contact","rf.contact","rh.contact","lh.contact",
        "lf.cscore","rf.cscore","rh.cscore","lh.cscore",
        "threshold",
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz"
};

// Input CSV: records estimator inputs + outputs with data_recorder-compatible headers
// Layout: 1 trigger + 8 motor theta/beta + 8 encoder derivatives + 10 imu + 39 estimation
constexpr int INPUT_STATE_SIZE = 27 + ODOM_DATA_SIZE;
std::vector<std::string> input_headers = {
        "trigger",
        "state_theta_a", "state_beta_a",
        "state_theta_b", "state_beta_b",
        "state_theta_c", "state_beta_c",
        "state_theta_d", "state_beta_d",
        "enc_beta_d_a", "enc_theta_d_a",
        "enc_beta_d_b", "enc_theta_d_b",
        "enc_beta_d_c", "enc_theta_d_c",
        "enc_beta_d_d", "enc_theta_d_d",
        "imu_orien_x", "imu_orien_y", "imu_orien_z", "imu_orien_w",
        "imu_ang_vel_x", "imu_ang_vel_y", "imu_ang_vel_z",
        "imu_lin_acc_x", "imu_lin_acc_y", "imu_lin_acc_z",
        "v_.x", "v_.y", "v_.z",
        "p.x", "p.y", "p.z",
        "zLF.x", "zLF.y", "zLF.z",
        "zRF.x", "zRF.y", "zRF.z",
        "zRH.x", "zRH.y", "zRH.z",
        "zLH.x", "zLH.y", "zLH.z",
        "ba.x", "ba.y", "ba.z",
        "lf.contact","rf.contact","rh.contact","lh.contact",
        "lf.cscore","rf.cscore","rh.cscore","lh.cscore",
        "threshold",
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz"
};

std::string output_file_path;
std::string input_file_path;
std::string output_file_name = "";
Eigen::VectorXf estimate_state = Eigen::VectorXf::Zero(ODOM_DATA_SIZE);
Eigen::VectorXf input_state   = Eigen::VectorXf::Zero(INPUT_STATE_SIZE);

// Variables
int J = ODOM_ESTIMATION_TIME_RANGE;
bool trigger = false;
float dt;
bool initialized = false;
Eigen::VectorXf x = Eigen::VectorXf::Zero(6 * J);
Eigen::Vector3f p;
Eigen::Quaternionf q_init;
Eigen::Matrix3f R_init;
Eigen::Matrix3f R;
Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
Eigen::Vector3f v_init;
Eigen::Vector3f a;
Eigen::Vector3f w;
Eigen::Quaternionf q;
Eigen::Quaternionf q_prev;

geometry_msgs::msg::Vector3 prev_v;

bool exclude[4];

int counter = 0;
//calculate the angle by lidar, need to implement lidar sensor to get the value
//set to -100 to disable lidar
float alpha_lf = -100;
float alpha_rf = -100;
float alpha_rh = -100;
float alpha_lh = -100;

Eigen::Matrix3f P_cov;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::ImuStamped imu;

rclcpp::Logger node_logger = rclcpp::get_logger("corgi_odometry_legacy");

// Callbacks
void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;

    if (RECORD_DATA){output_file_name = msg->output_filename;}

    std::string raw_filename = msg->output_filename;

    if (trigger && output_file_name != "") {
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros2_ws/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path + "_odom.csv";
        while (logger.file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + "_odom.csv";
            index++;
        }

        if (index != 1) {
            output_file_name += "_" + std::to_string(index-1) + "_odom.csv";
            output_file_path += "_" + std::to_string(index-1) + "_odom.csv";
        }
        else {
            output_file_name += "_odom.csv";
            output_file_path += "_odom.csv";
        }

        if (!logger.init) {

            // Initialize the CSV file.
            logger.initCSV(output_file_path, odo_headers);

            RCLCPP_INFO(node_logger, "Saving data to %s", output_file_name.c_str());
        }
    }
    else {
        if(logger.init){
            logger.finalizeCSV();
            RCLCPP_INFO(node_logger, "Saved data to %s", output_file_name.c_str());
        }
    }

    // Input logger: always active independent of RECORD_DATA
    if (trigger && raw_filename != "") {
        if (!input_logger.init) {
            std::string base = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros2_ws/output_data/" + raw_filename;
            input_file_path = base + "_input_odom.csv";
            int idx = 1;
            while (input_logger.file_exists(input_file_path)) {
                input_file_path = base + "_" + std::to_string(idx++) + "_input_odom.csv";
            }
            input_logger.initCSV(input_file_path, input_headers);
            RCLCPP_INFO(node_logger, "Saving input data to %s", input_file_path.c_str());
        }
    }
    else {
        if (input_logger.init) {
            input_logger.finalizeCSV();
            RCLCPP_INFO(node_logger, "Saved input data to %s", input_file_path.c_str());
        }
    }
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
    motor_state = *msg;
}

void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg){
    imu = *msg;
}

void contact_cb(const corgi_msgs::msg::ContactStateStamped::SharedPtr msg){
    exclude[0] = !msg->module_a.contact;
    exclude[1] = !msg->module_b.contact;
    exclude[2] = !msg->module_c.contact;
    exclude[3] = !msg->module_d.contact;
}

geometry_msgs::msg::Vector3 low_pass_filter(const geometry_msgs::msg::Vector3 &input, const geometry_msgs::msg::Vector3 &prev_input, float cutoff_freq, float sample_rate) {
    // Calculate the alpha value for the low-pass filter
    double alpha = 1.0 / (1.0 + (cutoff_freq / sample_rate));
    geometry_msgs::msg::Vector3 output;
    output.x = alpha * input.x + (1 - alpha) * prev_input.x;
    output.y = alpha * input.y + (1 - alpha) * prev_input.y;
    output.z = alpha * input.z + (1 - alpha) * prev_input.z;
    return output;
}

void Encoder::UpdateState(float dt){
    theta_prev = theta;
    beta_prev  = beta;
    theta = module->theta;
    if(opposite){
        beta  = -module->beta;
    }
    else{
        beta  = module->beta;
    }
    beta_d = (beta - beta_prev) / dt;
    theta_d = (theta - theta_prev) / dt;
    //pitch angle is opposite to y axis
    w_y = (imu->angular_velocity.y);
    state << theta, beta, beta_d, w_y, theta_d;
}

void Encoder::init(float dt){
    theta = module->theta;
    if(opposite){
        beta  = -module->beta;
    }
    else{
        beta  = module->beta;
    }
    UpdateState(dt);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corgi_odometry_legacy");
    
    // Wait for clock synchronization
    RCLCPP_INFO(node->get_logger(), "Waiting for clock synchronization...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0) {
            RCLCPP_INFO(node->get_logger(), "Clock synced! Sim Time: %.2f", node->now().seconds());
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // ROS Publishers
    auto velocity_pub = node->create_publisher<geometry_msgs::msg::Vector3>("odometry/legacy/velocity", 10);
    auto position_pub = node->create_publisher<geometry_msgs::msg::Vector3>("odometry/legacy/position", 10);
    auto contact_pub = node->create_publisher<corgi_msgs::msg::ContactStateStamped>("odometry/legacy/contact", 10);

    // ROS Subscribers
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>(
        "trigger", 10, trigger_cb);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>(
        "motor/state", 10, motor_state_cb);
    auto imu_sub = node->create_subscription<corgi_msgs::msg::ImuStamped>(
        "imu", 10, imu_cb);
    auto contact_sub = node->create_subscription<corgi_msgs::msg::ContactStateStamped>(
        "odometry/legacy/contact", 10, contact_cb);

    node_logger = node->get_logger();
    Eigen::initParallel();
    
    rclcpp::Duration period(0, 1000000000.0 / ODOM_ESTIMATOR_RATE); // Convert Hz to nanoseconds
    rclcpp::Time next_time = node->now();

    /* Estimate model initialization */

    //initial time
    dt = 1.0 / float(ODOM_ESTIMATOR_RATE);

    //initial velocity for low pass filter
    prev_v.x = 0.0;
    prev_v.y = 0.0;
    prev_v.z = 0.0;

    //IMU data (Observation)
    U u(J, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), dt);

    //Legs model
    Leg lf_leg(Eigen::Vector3f( MOTOR_OFFSET_X,  MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg rf_leg(Eigen::Vector3f( MOTOR_OFFSET_X, -MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg rh_leg(Eigen::Vector3f(-MOTOR_OFFSET_X, -MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
    Leg lh_leg(Eigen::Vector3f(-MOTOR_OFFSET_X,  MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
    
    //Legs encoder
    // motor a,d for left side, which rotation is opposite to right side
    Encoder encoder_lf(&motor_state.module_a, &imu, false);
    Encoder encoder_rf(&motor_state.module_b, &imu, true);
    Encoder encoder_rh(&motor_state.module_c, &imu, true);
    Encoder encoder_lh(&motor_state.module_d, &imu, false);

    //Dynamic predictor
    DP lf(J + 1, lf_leg, &u);
    DP rf(J + 1, rf_leg, &u);
    DP rh(J + 1, rh_leg, &u);
    DP lh(J + 1, lh_leg, &u);

    rot = Eigen::Matrix3f::Identity();
    v_init << 0, 0, 0;

    //initial state
    for (int i = 0; i < J; i++) {
        x(i * 3) = v_init(0);
        x(i * 3 + 1) = v_init(1);
        x(i * 3 + 2) = v_init(2);
        x((i + J) * 3) = 0;
        x((i + J) * 3 + 1) = 0;
        x((i + J) * 3 + 2) = 0;
    }

    //PKLD
    PKLD lf_pkld(dt, &lf);
    PKLD rf_pkld(dt, &rf);
    PKLD rh_pkld(dt, &rh);
    PKLD lh_pkld(dt, &lh);

    //GKLD
    GKLD filter(J, dt, &u);
    filter.threshold = THRESHOLD;
    filter.init(x);

    while (rclcpp::ok()){
        rclcpp::spin_some(node);

        if(trigger){
            if (!initialized) {

                /*Initialization : input first data*/
                //Encoder states
                encoder_lf.init(dt);
                encoder_rf.init(dt);
                encoder_rh.init(dt);
                encoder_lh.init(dt);
                //Dynamic predictor
                lf.init(encoder_lf.GetState(), 0);
                rf.init(encoder_rf.GetState(), 0);
                rh.init(encoder_rh.GetState(), 0);
                lh.init(encoder_lh.GetState(), 0);
                //PKLD
                filter.push_pkld(&lf_pkld);
                filter.push_pkld(&rf_pkld);
                filter.push_pkld(&rh_pkld);
                filter.push_pkld(&lh_pkld);

                //
                p << 0, 0, 0;
                //quaternion
                q_init = Eigen::Quaternionf(imu.orientation.w,imu.orientation.x, imu.orientation.y, imu.orientation.z);
                R_init = q_init.toRotationMatrix();
                R_init = rot * R_init;
                q_prev = q_init;

                initialized = true;
                // Skip filter update on the initialization tick; wait for next timer cycle
                // so the first UpdateState computes a real derivative over a full dt interval.
                next_time += period;
                node->get_clock()->sleep_until(next_time);
                continue;
            }
            //Update encoder states
            a = Eigen::Vector3f(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
            w = Eigen::Vector3f(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
            q = Eigen::Quaternionf(imu.orientation.w,imu.orientation.x, imu.orientation.y, imu.orientation.z);

            encoder_lf.UpdateState(dt);
            encoder_rf.UpdateState(dt);
            encoder_rh.UpdateState(dt);
            encoder_lh.UpdateState(dt);

            R = ESTIMATE_POSITION_FRAME ? q_prev.toRotationMatrix() : Eigen::Matrix3f::Identity();
            u.push_data(a, w, dt);
            lf.push_data(encoder_lf.GetState(), w, dt, alpha_lf);
            rf.push_data(encoder_rf.GetState(), w, dt, alpha_rf);
            rh.push_data(encoder_rh.GetState(), w, dt, alpha_rh);
            lh.push_data(encoder_lh.GetState(), w, dt, alpha_lh);

            filter.predict();

            if (KLD){
                filter.valid();
            }
            else{
                filter.valid(exclude);
            }
            
            x = filter.state();
            
            P_cov = filter.Y_inv.block<3, 3>(3*J-3, 3*J-3);
            p += rot * R * R_init.transpose() * x.segment(3 * J - 3, 3) * dt;
            
            // Print the counter number using RCLCPP_INFO
            RCLCPP_INFO(node_logger, "Counter: %d", counter);
            RCLCPP_INFO(node_logger, "Estimated Position: %f, %f, %f", p(0), p(1), p(2));
            RCLCPP_INFO(node_logger, "Estimated Velocity: %f, %f, %f", x(3 * J - 3), x(3 * J - 2), x(3 * J - 1));

            // Publish the estimated velocity and position
            geometry_msgs::msg::Vector3 velocity_msg;
            velocity_msg.x = x(3 * J - 3);
            velocity_msg.y = x(3 * J - 2);
            velocity_msg.z = x(3 * J - 1);
            velocity_pub->publish(velocity_msg);

            geometry_msgs::msg::Vector3 position_msg;
            position_msg.x = p(0);
            position_msg.y = p(1);
            position_msg.z = p(2);
            position_pub->publish(position_msg);

            if (PUB_CONTACT){
                // Publish contact state (1 for contact, 0 for no contact, higher score for non-contact)
                corgi_msgs::msg::ContactStateStamped contact_msg;
                contact_msg.module_a.contact = !filter.exclude[0];
                contact_msg.module_b.contact = !filter.exclude[1];
                contact_msg.module_c.contact = !filter.exclude[2];
                contact_msg.module_d.contact = !filter.exclude[3];
                contact_msg.module_a.score = filter.scores[0];
                contact_msg.module_b.score = filter.scores[1];
                contact_msg.module_c.score = filter.scores[2];
                contact_msg.module_d.score = filter.scores[3];
                contact_pub->publish(contact_msg);
            }

            // Always populate estimate_state (used by both odo logger and input logger)
            estimate_state.segment(0, 3) = x.segment(3 * J - 3, 3);                                 //velocity
            estimate_state.segment(3, 3) = p;                                                       //position
            estimate_state.segment(6, 3) = 1. / dt / (float) J * lf.z(dt);                          //lf leg velocity
            estimate_state.segment(9, 3) = 1. / dt / (float) J * rf.z(dt);                          //rf leg velocity
            estimate_state.segment(12, 3) = 1. / dt / (float) J * rh.z(dt);                         //rh leg velocity
            estimate_state.segment(15, 3) = 1. / dt / (float) J * lh.z(dt);                         //lh leg velocity
            estimate_state.segment(18, 3) = x.segment(6 * J - 3, 3);                                //bias
            estimate_state.segment(21, 4) = Eigen::Vector4f(!filter.exclude[0], !filter.exclude[1], !filter.exclude[2], !filter.exclude[3]);  //contact
            estimate_state.segment(25, 4) = Eigen::Vector<float, 4>(filter.scores[0], filter.scores[1], filter.scores[2], filter.scores[3]); //contact score
            estimate_state(29) = filter.threshold;                                                    //threshold
            estimate_state.segment(30, 9) = Eigen::Map<const Eigen::VectorXf>(P_cov.data(), P_cov.size());

            if (RECORD_DATA){
                logger.logState(estimate_state);
            }

            // Record inputs + outputs to input_logger (always, when initialized)
            if (input_logger.init) {
                input_state(0)  = 1.0f; // trigger
                input_state(1)  = static_cast<float>(motor_state.module_a.theta);
                input_state(2)  = static_cast<float>(motor_state.module_a.beta);
                input_state(3)  = static_cast<float>(motor_state.module_b.theta);
                input_state(4)  = static_cast<float>(motor_state.module_b.beta);
                input_state(5)  = static_cast<float>(motor_state.module_c.theta);
                input_state(6)  = static_cast<float>(motor_state.module_c.beta);
                input_state(7)  = static_cast<float>(motor_state.module_d.theta);
                input_state(8)  = static_cast<float>(motor_state.module_d.beta);
                // encoder derivatives as computed by Encoder::UpdateState
                input_state(9)  = encoder_lf.GetState()(2); // enc_beta_d_a
                input_state(10) = encoder_lf.GetState()(4); // enc_theta_d_a
                input_state(11) = encoder_rf.GetState()(2); // enc_beta_d_b
                input_state(12) = encoder_rf.GetState()(4); // enc_theta_d_b
                input_state(13) = encoder_rh.GetState()(2); // enc_beta_d_c
                input_state(14) = encoder_rh.GetState()(4); // enc_theta_d_c
                input_state(15) = encoder_lh.GetState()(2); // enc_beta_d_d
                input_state(16) = encoder_lh.GetState()(4); // enc_theta_d_d
                input_state(17) = static_cast<float>(imu.orientation.x);
                input_state(18) = static_cast<float>(imu.orientation.y);
                input_state(19) = static_cast<float>(imu.orientation.z);
                input_state(20) = static_cast<float>(imu.orientation.w);
                input_state(21) = static_cast<float>(imu.angular_velocity.x);
                input_state(22) = static_cast<float>(imu.angular_velocity.y);
                input_state(23) = static_cast<float>(imu.angular_velocity.z);
                input_state(24) = static_cast<float>(imu.linear_acceleration.x);
                input_state(25) = static_cast<float>(imu.linear_acceleration.y);
                input_state(26) = static_cast<float>(imu.linear_acceleration.z);
                input_state.segment(27, ODOM_DATA_SIZE) = estimate_state;
                input_logger.logState(input_state);
            }
            q_prev = q;
            counter ++;
        }
        
        if(counter > 0 && !trigger){
            break;
        }
        
        next_time += period;
        if(!node->get_clock()->sleep_until(next_time)){
            RCLCPP_WARN(node_logger, "Sleep until failed!");
            break;
        }
    }

    rclcpp::shutdown();
    
    return 0;
}
