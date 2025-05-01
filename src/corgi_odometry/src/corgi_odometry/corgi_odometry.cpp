#include "corgi_odometry.hpp"

using namespace estimation_model;

// Constants
#define SAMPLE_RATE 500.0 //Hz
#define THRESHOLD 0.08 //threshold of KLD
#define J 10 //sample time (matrix size)
#define MOTOR_OFFSET_X 0.222
#define MOTOR_OFFSET_Y 0.193
#define MOTOR_OFFSET_Z 0.0
#define WHEEL_RADIUS 0.1
#define GRAVITY 9.80665
#define lpf_alpha 0.5 //low pass filter alpha
#define SIM True //simulation mode
bool sim = true;
const float WHEEL_WIDTH = sim ? 0.012 : 0.019;
// CSV file parameters
std::vector<std::string> headers = {
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
DataProcessor::CsvLogger logger;
std::string filepath;
std::string filename;

//Classes
class Encoder{
    public:
        Encoder(corgi_msgs::MotorState* m, sensor_msgs::Imu* i, bool opposite): module(m), imu(i), opposite(opposite) {}
        void UpdateState(float dt){
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

        void init(float dt){
            theta = module->theta;
            if(opposite){
                beta  = -module->beta;
            }
            else{
                beta  = module->beta;
            }
            UpdateState(dt);
        }
        //const reference, prevent modified
        const Eigen::Matrix<float, 5, 1>& GetState() const{
            return state;
        }
    private:
        corgi_msgs::MotorState* module;
        sensor_msgs::Imu* imu;
        bool opposite;
        float theta;
        float beta;
        float theta_prev;
        float beta_prev;
        float theta_d;
        float beta_d;
        float w_y;
        Eigen::Vector<float, 5> state;
};

// Variables
bool trigger = false;
float dt;
bool initialized = false;
Eigen::VectorXf x = Eigen::VectorXf::Zero(6 * J);
Eigen::Vector3f p;
Eigen::Quaternionf q_init;
Eigen::Matrix3f R_init;
Eigen::Matrix3f R;
Eigen::Matrix3f rot;
Eigen::Vector3f v_init;
Eigen::Vector3f a;
Eigen::Vector3f w;
Eigen::Quaternionf q;
geometry_msgs::Vector3 prev_v;
double prev_time;
Eigen::Vector3f filtered_position;
bool exclude[4];

int counter = 0;
//calculate the angle by lidar, need to implement lidar sensor to get the value
//set to -100 to disable lidar
float alpha_lf = -100;
float alpha_rf = -100;
float alpha_rh = -100;
float alpha_lh = -100;

Eigen::Matrix3f P_cov;
corgi_msgs::MotorStateStamped motor_state;
sensor_msgs::Imu imu;

// Callbacks
void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void imu_cb(const sensor_msgs::Imu msg){
    imu = msg;
    imu.angular_velocity.x = 0.0;
}

void contact_cb(const corgi_msgs::ContactStateStamped msg){
    exclude[0] = !msg.module_a.contact;
    exclude[1] = !msg.module_b.contact;
    exclude[2] = !msg.module_c.contact;
    exclude[3] = !msg.module_d.contact;
}

geometry_msgs::Vector3 low_pass_filter(const geometry_msgs::Vector3 &input, const geometry_msgs::Vector3 &prev_input, float cutoff_freq, float sample_rate) {
    // Calculate the alpha value for the low-pass filter
    double alpha = 1.0 / (1.0 + (cutoff_freq / sample_rate));
    geometry_msgs::Vector3 output;
    output.x = alpha * input.x + (1 - alpha) * prev_input.x;
    output.y = alpha * input.y + (1 - alpha) * prev_input.y;
    output.z = alpha * input.z + (1 - alpha) * prev_input.z;
    return output;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_odometry");

    ros::NodeHandle nh;

    if (argc == 2){
        filepath = std::getenv("HOME");
        filename = argv[1];
        filepath += "/corgi_ws/corgi_ros_ws/src/corgi_odometry/data/";
        filepath += filename+ ".csv";
        ROS_INFO("Saving data to %s", filepath.c_str());
        // Initialize the CSV file.
        if (!logger.initCSV(filepath, headers)) {
            return -1;  // Exit if the CSV file could not be created.
        }
    }
    // ROS Publishers
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Vector3>("odometry/velocity", 10);
    ros::Publisher position_pub = nh.advertise<geometry_msgs::Vector3>("odometry/position", 10);
    // ros::Publisher filtered_velocity_pub = nh.advertise<geometry_msgs::Vector3>("odometry/filtered_velocity", 10);
    // ros::Publisher filtered_position_pub = nh.advertise<geometry_msgs::Vector3>("odometry/filtered_position", 10);

    // ROS Subscribers
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", SAMPLE_RATE, trigger_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", SAMPLE_RATE, motor_state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", SAMPLE_RATE, imu_cb);
    ros::Subscriber contact_sub = nh.subscribe<corgi_msgs::ContactStateStamped>("odometry/contact", SAMPLE_RATE, contact_cb);

    Eigen::initParallel();
    ros::Rate rate(SAMPLE_RATE);

    /* Estimate model initialization */

    //initial time
    dt = 1.0 / float(SAMPLE_RATE);
    prev_time = ros::Time::now().toSec();

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

    if(sim){
        rot <<  1,  0,  0, 
                0,  1,  0,
                0,  0,  1;
    }
    else{
        //Rotate imu data to body frame
        rot <<  1,  0,  0, 
                0,  -1,  0,
                0,  0,  -1;
    }
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

    while (ros::ok()){
        ros::spinOnce();

        // FIXME: dt calculation cannot work by calculate dynamically
        // double current_time = ros::Time::now().toSec();
        // dt = float(current_time - prev_time);
        // prev_time = current_time;
        // ROS_INFO("dt: %f", dt);
        dt = 1.0 / float(SAMPLE_RATE);

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
                filtered_position << 0, 0, 0;
                //quaternion
                q_init = Eigen::Quaternionf(imu.orientation.w,imu.orientation.x, imu.orientation.y, imu.orientation.z);
                R_init = q_init.toRotationMatrix();
                R_init = rot * R_init;

                initialized = true;
            }
            //Update encoder states
            a = Eigen::Vector3f(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
            w = Eigen::Vector3f(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
            q = Eigen::Quaternionf(imu.orientation.w,imu.orientation.x, imu.orientation.y, imu.orientation.z);

            encoder_lf.UpdateState(dt);
            encoder_rf.UpdateState(dt);
            encoder_rh.UpdateState(dt);
            encoder_lh.UpdateState(dt);

            R = q.toRotationMatrix();
            u.push_data(a, w, dt);
            lf.push_data(encoder_lf.GetState(), w, dt, alpha_lf);
            rf.push_data(encoder_rf.GetState(), w, dt, alpha_rf);
            rh.push_data(encoder_rh.GetState(), w, dt, alpha_rh);
            lh.push_data(encoder_lh.GetState(), w, dt, alpha_lh);

            filter.predict();
            // filter.valid();
            // implement the valid function w/ known contact leg
            filter.valid(exclude);
            x = filter.state();
            
            P_cov = filter.Y_inv.block<3, 3>(3*J-3, 3*J-3);
            p += rot * R * R_init.transpose() * x.segment(3 * J - 3, 3) * dt;
            
            // Print the counter number using ROS_INFO
            ROS_INFO("Counter: %d", counter);
            ROS_INFO("Estimated Position: %f, %f, %f", p(0), p(1), p(2));
            ROS_INFO("Estimated Velocity: %f, %f, %f", x(3 * J - 3), x(3 * J - 2), x(3 * J - 1));

            // // Publish contact state (1 for contact, 0 for no contact, higher score for non-contact)
            // corgi_msgs::ContactStateStamped contact_msg;
            // contact_msg.module_a.contact = !filter.exclude[0];
            // contact_msg.module_b.contact = !filter.exclude[1];
            // contact_msg.module_c.contact = !filter.exclude[2];
            // contact_msg.module_d.contact = !filter.exclude[3];
            // contact_msg.module_a.score = filter.scores[0];
            // contact_msg.module_b.score = filter.scores[1];
            // contact_msg.module_c.score = filter.scores[2];
            // contact_msg.module_d.score = filter.scores[3];
            // contact_pub.publish(contact_msg);

            // Publish the estimated velocity and position
            geometry_msgs::Vector3 velocity_msg;
            velocity_msg.x = x(3 * J - 3);
            velocity_msg.y = x(3 * J - 2);
            velocity_msg.z = x(3 * J - 1);
            velocity_pub.publish(velocity_msg);

            geometry_msgs::Vector3 position_msg;
            position_msg.x = p(0);
            position_msg.y = p(1);
            position_msg.z = p(2);
            position_pub.publish(position_msg);

            // geometry_msgs::Vector3 filtered_velocity_msg;
            // float cutoff_freq = 10.0; //Hz
            // filtered_velocity_msg = low_pass_filter(velocity_msg, prev_v, cutoff_freq, SAMPLE_RATE);
            // prev_v = filtered_velocity_msg;
            // filtered_velocity_pub.publish(filtered_velocity_msg);
            
            // Eigen::Vector<float, 3> filtered_velocity;
            // filtered_velocity << filtered_velocity_msg.x, filtered_velocity_msg.y, filtered_velocity_msg.z;
            // filtered_position += rot * R * R_init.transpose() * filtered_velocity * dt;

            // geometry_msgs::Vector3 filtered_position_msg;
            // filtered_position_msg.x = filtered_position(0);
            // filtered_position_msg.y = filtered_position(1);
            // filtered_position_msg.z = filtered_position(2);
            // filtered_position_pub.publish(filtered_position_msg);

            // Store the estimated state to a csv file
            Eigen::VectorXf estimate_state = Eigen::VectorXf::Zero(39);
            estimate_state.segment(0, 3) = x.segment(3 * J - 3, 3);                                 //velocity
            estimate_state.segment(3, 3) = p;                                                       //position
            estimate_state.segment(6, 3) = 1. / dt / (float) J * lf.z(dt);                          //lf leg velocity
            estimate_state.segment(9, 3) = 1. / dt / (float) J * rf.z(dt);                          //rf leg velocity
            estimate_state.segment(12, 3) = 1. / dt / (float) J * rh.z(dt);                         //rh leg velocity
            estimate_state.segment(15, 3) = 1. / dt / (float) J * lh.z(dt);                         //lh leg velocity
            estimate_state.segment(18, 3) = x.segment(6 * J - 3, 3);                                //bias
            estimate_state.segment(21, 4) = Eigen::Vector4f(!filter.exclude[0], !filter.exclude[1], !filter.exclude[2], !filter.exclude[3]);            //contact
            estimate_state.segment(25, 4) = Eigen::Vector<float, 4>(filter.scores[0], filter.scores[1], filter.scores[2], filter.scores[3]);        //contact score
            estimate_state(29) = filter.threshold;                                                                                                  //threshold
            estimate_state.segment(30, 9) = Eigen::Map<const Eigen::VectorXf>(P_cov.data(), P_cov.size());                                          //covariance
            // estimate_state.segment(39, 3) = Eigen::Vector<float, 3>(filtered_velocity_msg.x, filtered_velocity_msg.y, filtered_velocity_msg.z);     //filtered velocity
            // estimate_state.segment(42, 3) = Eigen::Vector<float, 3>(filtered_position_msg.x, filtered_position_msg.y, filtered_position_msg.z);     //filtered position                                                                                       
            
            
            if(argc == 2){logger.logState(estimate_state);}

            counter ++;
        }
        
        if(counter > 0 && !trigger){
            break;
        }
        rate.sleep();
    }

    if(argc == 2){
        logger.finalizeCSV();
        ROS_INFO("Saved data to %s", filepath.c_str());
    }
    ros::shutdown();
    
    return 0;
}
