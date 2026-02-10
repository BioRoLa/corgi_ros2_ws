#include "corgi_odometry.hpp"

using namespace estimation_model;

DataProcessor::CsvLogger logger;

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
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz",
        "filtered_v_.x", "filtered_v_.y", "filtered_v_.z",
        "filtered_p.x", "filtered_p.y", "filtered_p.z"
};

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

class CorgiOdometryNode : public rclcpp::Node
{
public:
    CorgiOdometryNode() : Node("corgi_odometry")
    {
        // Publishers
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("odometry/velocity", 10);
        position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("odometry/position", 10);
        filtered_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("odometry/filtered_velocity", 10);
        filtered_position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("odometry/filtered_position", 10);
        contact_pub_ = this->create_publisher<corgi_msgs::msg::ContactStateStamped>("odometry/contact", 10);

        // Subscribers
        trigger_sub_ = this->create_subscription<corgi_msgs::msg::TriggerStamped>(
            "trigger", 10, std::bind(&CorgiOdometryNode::trigger_cb, this, std::placeholders::_1));
        motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            "motor/state", 10, std::bind(&CorgiOdometryNode::motor_state_cb, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<corgi_msgs::msg::ImuStamped>(
            "imu", 10, std::bind(&CorgiOdometryNode::imu_cb, this, std::placeholders::_1));
        contact_sub_ = this->create_subscription<corgi_msgs::msg::ContactStateStamped>(
            "odometry/contact", 10, std::bind(&CorgiOdometryNode::contact_cb, this, std::placeholders::_1));

        // Initialize
        initialize();

        // Timer for main loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ODOM_ESTIMATOR_RATE),
            std::bind(&CorgiOdometryNode::timer_callback, this));
    }

private:
    void initialize()
    {
        dt_ = 1.0 / float(ODOM_ESTIMATOR_RATE);
        
        prev_v_.x = 0.0;
        prev_v_.y = 0.0;
        prev_v_.z = 0.0;

        // IMU data (Observation)
        u_ = std::make_shared<U>(J_, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), dt_);

        // Legs model
        lf_leg_ = std::make_shared<Leg>(Eigen::Vector3f( MOTOR_OFFSET_X,  MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
        rf_leg_ = std::make_shared<Leg>(Eigen::Vector3f( MOTOR_OFFSET_X, -MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
        rh_leg_ = std::make_shared<Leg>(Eigen::Vector3f(-MOTOR_OFFSET_X, -MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
        lh_leg_ = std::make_shared<Leg>(Eigen::Vector3f(-MOTOR_OFFSET_X,  MOTOR_OFFSET_Y, MOTOR_OFFSET_Z), WHEEL_RADIUS, WHEEL_WIDTH);
        
        // Legs encoder
        encoder_lf_ = std::make_shared<Encoder>(&motor_state_.module_a, &imu_, false);
        encoder_rf_ = std::make_shared<Encoder>(&motor_state_.module_b, &imu_, true);
        encoder_rh_ = std::make_shared<Encoder>(&motor_state_.module_c, &imu_, true);
        encoder_lh_ = std::make_shared<Encoder>(&motor_state_.module_d, &imu_, false);

        // Dynamic predictor
        lf_ = std::make_shared<DP>(J_ + 1, *lf_leg_, u_.get());
        rf_ = std::make_shared<DP>(J_ + 1, *rf_leg_, u_.get());
        rh_ = std::make_shared<DP>(J_ + 1, *rh_leg_, u_.get());
        lh_ = std::make_shared<DP>(J_ + 1, *lh_leg_, u_.get());

        if(SIM){
            rot_ <<  1,  0,  0, 
                     0,  1,  0,
                     0,  0,  1;
        }
        else{
            rot_ <<  1,  0,  0, 
                     0, -1,  0,
                     0,  0, -1;
        }
        
        v_init_ << 0, 0, 0;

        // Initial state
        x_ = Eigen::VectorXf::Zero(6 * J_);
        for (int i = 0; i < J_; i++) {
            x_(i * 3) = v_init_(0);
            x_(i * 3 + 1) = v_init_(1);
            x_(i * 3 + 2) = v_init_(2);
            x_((i + J_) * 3) = 0;
            x_((i + J_) * 3 + 1) = 0;
            x_((i + J_) * 3 + 2) = 0;
        }

        // PKLD
        lf_pkld_ = std::make_shared<PKLD>(dt_, lf_.get());
        rf_pkld_ = std::make_shared<PKLD>(dt_, rf_.get());
        rh_pkld_ = std::make_shared<PKLD>(dt_, rh_.get());
        lh_pkld_ = std::make_shared<PKLD>(dt_, lh_.get());

        // GKLD
        filter_ = std::make_shared<GKLD>(J_, dt_, u_.get());
        filter_->threshold = THRESHOLD;
        filter_->init(x_);

        p_ << 0, 0, 0;
        filtered_position_ << 0, 0, 0;

        Eigen::initParallel();
    }

    void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg)
    {
        trigger_ = msg->enable;

        if (RECORD_DATA){output_file_name_ = msg->output_filename;}

        if (trigger_ && output_file_name_ != "") {
            output_file_path_ = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros_ws/output_data/" + output_file_name_;

            int index = 1;
            std::string file_path_with_extension = output_file_path_ + "_odom.csv";
            while (logger.file_exists(file_path_with_extension)) {
                file_path_with_extension = output_file_path_ + "_" + std::to_string(index) + "_odom.csv";
                index++;
            }

            if (index != 1) {
                output_file_name_ += "_" + std::to_string(index-1) + "_odom.csv";
                output_file_path_ += "_" + std::to_string(index-1) + "_odom.csv";
            }
            else {
                output_file_name_ += "_odom.csv";
                output_file_path_ += "_odom.csv";
            }

            if (!logger.init) {
                logger.initCSV(output_file_path_, odo_headers);
                RCLCPP_INFO(this->get_logger(), "Saving data to %s", output_file_name_.c_str());
            }
        }
        else {
            if(logger.init){
                logger.finalizeCSV();
                RCLCPP_INFO(this->get_logger(), "Saved data to %s", output_file_name_.c_str());
            }
        }
    }

    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg)
    {
        motor_state_ = *msg;
    }

    void imu_cb(const corgi_msgs::msg::ImuStamped::SharedPtr msg)
    {
        imu_ = *msg;
        imu_.angular_velocity.x = 0.0;
    }

    void contact_cb(const corgi_msgs::msg::ContactStateStamped::SharedPtr msg)
    {
        exclude_[0] = !msg->module_a.contact;
        exclude_[1] = !msg->module_b.contact;
        exclude_[2] = !msg->module_c.contact;
        exclude_[3] = !msg->module_d.contact;
    }

    void timer_callback()
    {
        if(trigger_){
            if (!initialized_) {
                /*Initialization : input first data*/
                encoder_lf_->init(dt_);
                encoder_rf_->init(dt_);
                encoder_rh_->init(dt_);
                encoder_lh_->init(dt_);
                
                lf_->init(encoder_lf_->GetState(), 0);
                rf_->init(encoder_rf_->GetState(), 0);
                rh_->init(encoder_rh_->GetState(), 0);
                lh_->init(encoder_lh_->GetState(), 0);
                
                filter_->push_pkld(lf_pkld_.get());
                filter_->push_pkld(rf_pkld_.get());
                filter_->push_pkld(rh_pkld_.get());
                filter_->push_pkld(lh_pkld_.get());

                q_init_ = Eigen::Quaternionf(imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z);
                R_init_ = q_init_.toRotationMatrix();
                R_init_ = rot_ * R_init_;

                initialized_ = true;
            }
            
            // Update encoder states
            a_ = Eigen::Vector3f(imu_.linear_acceleration.x, imu_.linear_acceleration.y, imu_.linear_acceleration.z);
            w_ = Eigen::Vector3f(imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z);
            q_ = Eigen::Quaternionf(imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z);

            encoder_lf_->UpdateState(dt_);
            encoder_rf_->UpdateState(dt_);
            encoder_rh_->UpdateState(dt_);
            encoder_lh_->UpdateState(dt_);

            R_ = ESTIMATE_POSITION_FRAME ? q_.toRotationMatrix() : Eigen::Matrix3f::Identity();
            u_->push_data(a_, w_, dt_);
            lf_->push_data(encoder_lf_->GetState(), w_, dt_, alpha_lf_);
            rf_->push_data(encoder_rf_->GetState(), w_, dt_, alpha_rf_);
            rh_->push_data(encoder_rh_->GetState(), w_, dt_, alpha_rh_);
            lh_->push_data(encoder_lh_->GetState(), w_, dt_, alpha_lh_);

            filter_->predict();

            if (KLD){
                filter_->valid();
            }
            else{
                filter_->valid(exclude_);
            }
            
            x_ = filter_->state();
            
            P_cov_ = filter_->Y_inv.block<3, 3>(3*J_-3, 3*J_-3);
            p_ += rot_ * R_ * R_init_.transpose() * x_.segment(3 * J_ - 3, 3) * dt_;
            
            RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
            RCLCPP_INFO(this->get_logger(), "Estimated Position: %f, %f, %f", p_(0), p_(1), p_(2));
            RCLCPP_INFO(this->get_logger(), "Estimated Velocity: %f, %f, %f", x_(3 * J_ - 3), x_(3 * J_ - 2), x_(3 * J_ - 1));

            // Publish the estimated velocity and position
            geometry_msgs::msg::Vector3 velocity_msg;
            velocity_msg.x = x_(3 * J_ - 3);
            velocity_msg.y = x_(3 * J_ - 2);
            velocity_msg.z = x_(3 * J_ - 1);
            velocity_pub_->publish(velocity_msg);

            geometry_msgs::msg::Vector3 position_msg;
            position_msg.x = p_(0);
            position_msg.y = p_(1);
            position_msg.z = p_(2);
            position_pub_->publish(position_msg);

            if (PUB_CONTACT){
                corgi_msgs::msg::ContactStateStamped contact_msg;
                contact_msg.module_a.contact = !filter_->exclude[0];
                contact_msg.module_b.contact = !filter_->exclude[1];
                contact_msg.module_c.contact = !filter_->exclude[2];
                contact_msg.module_d.contact = !filter_->exclude[3];
                contact_msg.module_a.score = filter_->scores[0];
                contact_msg.module_b.score = filter_->scores[1];
                contact_msg.module_c.score = filter_->scores[2];
                contact_msg.module_d.score = filter_->scores[3];
                contact_pub_->publish(contact_msg);
            }

            if (RECORD_DATA){
                Eigen::VectorXf estimate_state = Eigen::VectorXf::Zero(ODOM_DATA_SIZE);
                estimate_state.segment(0, 3) = x_.segment(3 * J_ - 3, 3);
                estimate_state.segment(3, 3) = p_;
                estimate_state.segment(6, 3) = 1. / dt_ / (float) J_ * lf_->z(dt_);
                estimate_state.segment(9, 3) = 1. / dt_ / (float) J_ * rf_->z(dt_);
                estimate_state.segment(12, 3) = 1. / dt_ / (float) J_ * rh_->z(dt_);
                estimate_state.segment(15, 3) = 1. / dt_ / (float) J_ * lh_->z(dt_);
                estimate_state.segment(18, 3) = x_.segment(6 * J_ - 3, 3);
                estimate_state.segment(21, 4) = Eigen::Vector4f(!filter_->exclude[0], !filter_->exclude[1], !filter_->exclude[2], !filter_->exclude[3]);
                estimate_state.segment(25, 4) = Eigen::Vector<float, 4>(filter_->scores[0], filter_->scores[1], filter_->scores[2], filter_->scores[3]);
                estimate_state(29) = filter_->threshold;
                estimate_state.segment(30, 9) = Eigen::Map<const Eigen::VectorXf>(P_cov_.data(), P_cov_.size());

                if (FILTE_VEL){
                    geometry_msgs::msg::Vector3 filtered_velocity_msg;
                    float cutoff_freq = FILTE_VEL_CUT_OFF_FREQ;
                    filtered_velocity_msg = low_pass_filter(velocity_msg, prev_v_, cutoff_freq, ODOM_ESTIMATOR_RATE);
                    prev_v_ = filtered_velocity_msg;
                    filtered_velocity_pub_->publish(filtered_velocity_msg);
                    
                    Eigen::Vector<float, 3> filtered_velocity;
                    filtered_velocity << filtered_velocity_msg.x, filtered_velocity_msg.y, filtered_velocity_msg.z;
                    filtered_position_ += rot_ * R_ * R_init_.transpose() * filtered_velocity * dt_;

                    geometry_msgs::msg::Vector3 filtered_position_msg;
                    filtered_position_msg.x = filtered_position_(0);
                    filtered_position_msg.y = filtered_position_(1);
                    filtered_position_msg.z = filtered_position_(2);
                    filtered_position_pub_->publish(filtered_position_msg);

                    estimate_state.segment(39, 3) = Eigen::Vector<float, 3>(filtered_velocity_msg.x, filtered_velocity_msg.y, filtered_velocity_msg.z);
                    estimate_state.segment(42, 3) = Eigen::Vector<float, 3>(filtered_position_msg.x, filtered_position_msg.y, filtered_position_msg.z);
                }
                
                logger.logState(estimate_state);
            }
            counter_++;
        }
        
        if(counter_ > 0 && !trigger_){
            rclcpp::shutdown();
        }
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr filtered_position_pub_;
    rclcpp::Publisher<corgi_msgs::msg::ContactStateStamped>::SharedPtr contact_pub_;

    // Subscribers
    rclcpp::Subscription<corgi_msgs::msg::TriggerStamped>::SharedPtr trigger_sub_;
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImuStamped>::SharedPtr imu_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ContactStateStamped>::SharedPtr contact_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    int J_ = ODOM_ESTIMATION_TIME_RANGE;
    bool trigger_ = false;
    float dt_;
    bool initialized_ = false;
    Eigen::VectorXf x_;
    Eigen::Vector3f p_;
    Eigen::Quaternionf q_init_;
    Eigen::Matrix3f R_init_;
    Eigen::Matrix3f R_;
    Eigen::Matrix3f rot_;
    Eigen::Vector3f v_init_;
    Eigen::Vector3f a_;
    Eigen::Vector3f w_;
    Eigen::Quaternionf q_;
    geometry_msgs::msg::Vector3 prev_v_;
    Eigen::Vector3f filtered_position_;
    bool exclude_[4] = {false, false, false, false};
    int counter_ = 0;
    float alpha_lf_ = -100;
    float alpha_rf_ = -100;
    float alpha_rh_ = -100;
    float alpha_lh_ = -100;
    Eigen::Matrix3f P_cov_;
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::ImuStamped imu_;
    std::string output_file_path_;
    std::string output_file_name_ = "";

    // Shared pointers for filter components
    std::shared_ptr<U> u_;
    std::shared_ptr<Leg> lf_leg_, rf_leg_, rh_leg_, lh_leg_;
    std::shared_ptr<Encoder> encoder_lf_, encoder_rf_, encoder_rh_, encoder_lh_;
    std::shared_ptr<DP> lf_, rf_, rh_, lh_;
    std::shared_ptr<PKLD> lf_pkld_, rf_pkld_, rh_pkld_, lh_pkld_;
    std::shared_ptr<GKLD> filter_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorgiOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
