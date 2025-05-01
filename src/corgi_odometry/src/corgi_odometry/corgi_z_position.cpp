#include "corgi_odometry.hpp"

LegModel legmodel(SIM);

// Constants
constexpr double Z_POS_ANALYSIS_RATE = 1000.0;

// Variables
bool trigger = false;
corgi_msgs::MotorStateStamped motor_state;
sensor_msgs::Imu imu;
corgi_msgs::ContactStateStamped contact_state;
std_msgs::Float64 prev_z_COM;

std::vector<std::string> headers = {
    "estimate_z_position",
    "a.contact","b.contact","c.contact","d.contact",
    "a.score","b.score","c.score","d.score",
};
DataProcessor::CsvLogger logger;


// Callbacks
void trigger_cb(const corgi_msgs::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::MotorStateStamped state){
    motor_state = state;
}

void imu_cb(const sensor_msgs::Imu msg){
    imu = msg;
}

void contact_cb(const corgi_msgs::ContactStateStamped msg){
    contact_state = msg;
}

double estimate_z(double theta, double beta) {
    legmodel.forward(theta, beta);
    legmodel.contact_map(theta, beta);
    return -legmodel.contact_p[1];
}

// Function to compute Euler angles from a quaternion using ZYX order.
void quaternionToEuler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    // Normalize the quaternion (if not already normalized)
    Eigen::Quaterniond q_norm = q.normalized();

    // Calculate roll (x-axis rotation)
    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    // Calculate pitch (y-axis rotation)
    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    // Calculate yaw (z-axis rotation)
    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

double median(std::vector<double> &v) {
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    if (n % 2 == 0) {
        return (v[n / 2 - 1] + v[n / 2]) / 2.0;
    } else {
        return v[n / 2];
    }
}

double low_pass_filter(double value, double prev_value, double cutoff_freq, double sample_rate) {
    double alpha = 1.0 / (1.0 + (cutoff_freq / sample_rate));
    return alpha * value + (1.0 - alpha) * prev_value;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corgi_z_position");

    ros::NodeHandle nh;

    // ROS Publishers
    ros::Publisher z_position_pub = nh.advertise<std_msgs::Float64>("odometry/z_position_hip", 10);
    // ROS Subscribers
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", Z_POS_ANALYSIS_RATE, trigger_cb);
    ros::Subscriber motor_state_sub = nh.subscribe<corgi_msgs::MotorStateStamped>("motor/state", Z_POS_ANALYSIS_RATE, motor_state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", Z_POS_ANALYSIS_RATE, imu_cb);
    ros::Subscriber contact_sub = nh.subscribe<corgi_msgs::ContactStateStamped>("odometry/contact", Z_POS_ANALYSIS_RATE, contact_cb);
    
    std::string filepath = std::getenv("HOME");
    filepath += "/corgi_ws/corgi_ros_ws/src/corgi_odometry/data/z_test.csv";
    logger.initCSV(filepath, headers);

    ros::Rate rate(Z_POS_ANALYSIS_RATE);

    Eigen::Quaterniond q;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    prev_z_COM.data = 0.0;

    std::vector<corgi_msgs::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::ContactState*> contact_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
    };

    double z_leg[4];

    while (ros::ok()){

        ros::spinOnce();

        q = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
        quaternionToEuler(q, roll, pitch, yaw);
        if(!SIM){
            pitch = -pitch;
            roll = -roll;
        }

        if(trigger){
            for (int i=0; i<4; i++){
    
                if (i == 1 || i == 2) {
                    z_leg[i] = estimate_z(motor_state_modules[i]->theta, motor_state_modules[i]->beta-pitch);
                }
                else {
                    z_leg[i] = estimate_z(motor_state_modules[i]->theta, motor_state_modules[i]->beta+pitch);
                }
            }

            std::vector<double> contact_heights;
            for (int i = 0; i < 4; i++) {
                if (contact_modules[i]->contact) {
                    contact_heights.push_back(z_leg[i]);
                }
            }

            std_msgs::Float64 z_COM;
            if (contact_heights.empty()) {
                z_COM.data = prev_z_COM.data;
            } 
            else {
                z_COM.data = median(contact_heights);
            }
            z_COM.data = low_pass_filter(z_COM.data, prev_z_COM.data, 10, Z_POS_ANALYSIS_RATE);
            prev_z_COM.data = z_COM.data;

            z_position_pub.publish(z_COM);
            ROS_INFO("z_COM: %f", z_COM);

            Eigen::VectorXf state = Eigen::VectorXf::Zero(9); //total 10 columns
            state(0) = z_COM.data;
            for (int i = 0; i < 4; i++) {
                state(1 + i) = contact_modules[i]->contact;
            }
            for (int i = 0; i < 4; i++) {
                state(5 + i) = contact_modules[i]->score;
            }
            logger.logState(state);

            rate.sleep();
        }

        logger.finalizeCSV();

    }
    
    ros::shutdown();
    
    return 0;
}
