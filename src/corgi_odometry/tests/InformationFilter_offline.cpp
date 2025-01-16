/**
 * @file InformationFilter_offline.cpp
 * 
 * @author peichunhuang
 * @modified by Kenny-Huang
 */
#include "KLD_estimation/InformationFilter.hpp"
#include "KLD_estimation/csv_reader.hpp"
#include <random>
using namespace estimation_model;

//define constant
const float INPUT_CSV_RATE = 1000;
const float SAMPLE_RATE = 200;
const float SAMPLE_RATIO = INPUT_CSV_RATE / SAMPLE_RATE;

template<size_t n>
Eigen::Vector<float, n> random_vector() {
    std::random_device rd;
    std::mt19937 gen(rd());  //here you could also set a seed
    std::uniform_real_distribution<float> dis(-1, 1);
    Eigen::Vector<float, n> V = Eigen::Vector<float, n>().NullaryExpr([&](){return dis(gen);});
    return V;
}

float random_number() {
    std::random_device rd;
    std::mt19937 gen(rd());  //here you could also set a seed
    std::uniform_real_distribution<float> dis(-1, 1);
    return dis(gen);
}


int main (int argc, char* argv[]) {

    std::string filename = std::string(argv[1]);
    std::string filepath = std::getenv("HOME");
    filepath += "/corgi_ws/corgi_ros_ws/src/corgi_odometry/data/";
    DataProcessor::DataFrame df =  DataProcessor::read_csv(filepath+filename+".csv");
    int n = df.row;
    Eigen::initParallel();
    // float thres = 1e-10;
    float thres = 0.08;
    if (argc == 3) {
        thres = std::atof(argv[2]);
    }
    std::cout << "thres : " << thres << "\n";  //add threshold of KLD
    int start_index = 0;
    while (1) {
        start_index ++;
        if (df.iloc("trigger", start_index))  // FIXME: if cannot find "trigger", function return first colume 
            break;
    }
    Eigen::MatrixXf estimate_state = Eigen::MatrixXf::Zero((float)(n - start_index)/SAMPLE_RATIO, 46);  //TODO: refactor to make it more readable, "magic number" 46

    // read data from csv
    Eigen::Vector3f a(df.iloc("imu_lin_acc_x", start_index), df.iloc("imu_lin_acc_y", start_index), df.iloc("imu_lin_acc_z", start_index)-9.8);
    Eigen::Quaternionf q(df.iloc("imu_orien_w", start_index), df.iloc("imu_orien_x", start_index), df.iloc("imu_orien_y", start_index), df.iloc("imu_orien_z", start_index));
    Eigen::Vector<float, 5> encoder_lf(df.iloc("state_theta_a", start_index), df.iloc("state_beta_a", start_index), 0, df.iloc("imu_ang_vel_y", start_index), 0);
    Eigen::Vector<float, 5> encoder_rf(df.iloc("state_theta_b", start_index), -df.iloc("state_beta_b", start_index), 0, df.iloc("imu_ang_vel_y", start_index), 0);
    Eigen::Vector<float, 5> encoder_rh(df.iloc("state_theta_c", start_index), -df.iloc("state_beta_c", start_index), 0, df.iloc("imu_ang_vel_y", start_index), 0);
    Eigen::Vector<float, 5> encoder_lh(df.iloc("state_theta_d", start_index), df.iloc("state_beta_d", start_index), 0, df.iloc("imu_ang_vel_y", start_index), 0);

    //initialize motor data to calculate encoder angular velocity
    //prev theta and beta
    Eigen::Vector<float, 2> motor_data_lf_prev = Eigen::Vector<float, 2>(df.iloc("state_theta_a", start_index), df.iloc("state_beta_a", start_index));
    Eigen::Vector<float, 2> motor_data_rf_prev = Eigen::Vector<float, 2>(df.iloc("state_theta_b", start_index), -df.iloc("state_beta_b", start_index));
    Eigen::Vector<float, 2> motor_data_rh_prev = Eigen::Vector<float, 2>(df.iloc("state_theta_c", start_index), -df.iloc("state_beta_c", start_index));
    Eigen::Vector<float, 2> motor_data_lh_prev = Eigen::Vector<float, 2>(df.iloc("state_theta_d", start_index), df.iloc("state_beta_d", start_index));
    //theta_d and beta_d
    Eigen::Vector<float, 2> motor_data_d_lf = Eigen::Vector<float, 2>(0, 0);
    Eigen::Vector<float, 2> motor_data_d_rf = Eigen::Vector<float, 2>(0, 0);
    Eigen::Vector<float, 2> motor_data_d_rh = Eigen::Vector<float, 2>(0, 0);
    Eigen::Vector<float, 2> motor_data_d_lh = Eigen::Vector<float, 2>(0, 0);
    

    Eigen::Vector3f v_init(0, 0, 0) ;
    const int j = 10; // sample time (matrix size)
    float dt = 1 / (float)SAMPLE_RATE;
    U u(j, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), dt); //IMU data
    Leg lf_leg(Eigen::Vector3f(0.222, 0.193, 0), 0.1, 0.012);
    Leg rf_leg(Eigen::Vector3f(0.222, -0.193, 0), 0.1, 0.012);
    Leg rh_leg(Eigen::Vector3f(-0.222, -0.193, 0), 0.1, 0.012);
    Leg lh_leg(Eigen::Vector3f(-0.222, 0.193, 0), 0.1, 0.012);
    DP lf(j + 1, lf_leg, encoder_lf, 0, &u);
    DP rf(j + 1, rf_leg, encoder_rf, 0, &u);
    DP rh(j + 1, rh_leg, encoder_rh, 0, &u);
    DP lh(j + 1, lh_leg, encoder_lh, 0, &u);

    // T265 data
    // Eigen::Quaternionf qt265(df.iloc("t265.qw", start_index), df.iloc("t265.qx", start_index), df.iloc("t265.qy", start_index), df.iloc("t265.qz", start_index));
    // Eigen::Matrix3f t265_to_imu;
    // t265_to_imu << 0, 0, -1, 1, 0, 0, 0, -1, 0;
    // T265 t265(&lf, t265_to_imu, Eigen::Vector3f(0, 0, 0.285), j + 1, 
    // Eigen::Vector3f(df.iloc("t265.x", 0), df.iloc("t265.y", 0), df.iloc("t265.z", 0)), 
    // qt265.toRotationMatrix());

    // initial state
    Eigen::VectorXf x = Eigen::VectorXf::Zero(6 * j);
    for (int i = 0; i < j; i++) {
        x(i * 3) = v_init(0);
        x(i * 3 + 1) = v_init(1);
        x(i * 3 + 2) = v_init(2);
        x((i + j) * 3) = 0;
        x((i + j) * 3 + 1) = 0;
        x((i + j) * 3 + 2) = 0;
    }

    PKLD lf_pkld(dt, &lf);
    PKLD rf_pkld(dt, &rf);
    PKLD rh_pkld(dt, &rh);
    PKLD lh_pkld(dt, &lh);
    // PKLD t265_pkld(dt, &t265);

    GKLD filter(j, dt, &u);
    filter.threshold = thres;
    filter.init(x) ;
    filter.push_pkld(&lf_pkld) ;
    filter.push_pkld(&rf_pkld) ;
    filter.push_pkld(&rh_pkld) ;
    filter.push_pkld(&lh_pkld) ;
    int counter = 0;
    Eigen::Vector3f p(0, 0, 0);

    Eigen::Quaternionf q_init(df.iloc("imu_orien_w", start_index), df.iloc("imu_orien_x", start_index), df.iloc("imu_orien_y", start_index), df.iloc("imu_orien_z", start_index));
    Eigen::Matrix3f R_init = q_init.toRotationMatrix();
    Eigen::Matrix3f rot;
    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1; // imu to body frame
    int sec = 0, usec = 0;
    R_init = rot * R_init;
    // Eigen::Vector3f t265_pose = Eigen::Vector3f(df.iloc("t265.x", start_index), df.iloc("t265.y", start_index), df.iloc("t265.z", start_index));
    for (int i = start_index+1; i < n; i+=SAMPLE_RATIO) {
        std::cout << counter << "\n";
        // Eigen::Quaternionf qt265(df.iloc("t265.qw", i-1), df.iloc("t265.qx", i-1), df.iloc("t265.qy", i-1), df.iloc("t265.qz", i-1));
        Eigen::Vector3f a(df.iloc("imu_lin_acc_x", i), df.iloc("imu_lin_acc_y", i), df.iloc("imu_lin_acc_z", i)-9.8);
        Eigen::Quaternionf q(df.iloc("imu_orien_w", i-SAMPLE_RATIO), df.iloc("imu_orien_x", i-SAMPLE_RATIO), df.iloc("imu_orien_y", i-SAMPLE_RATIO), df.iloc("imu_orien_z", i-SAMPLE_RATIO));
        Eigen::Vector3f w(df.iloc("imu_ang_vel_x", i), df.iloc("imu_ang_vel_y", i), df.iloc("imu_ang_vel_z", i));

        // calculate encoder angular velocity
        motor_data_d_lf(0) = (df.iloc("state_theta_a", i) - motor_data_lf_prev(0))/dt;
        motor_data_d_lf(1) = (df.iloc("state_beta_a", i) - motor_data_lf_prev(1))/dt;
        motor_data_d_rf(0) = (df.iloc("state_theta_b", i) - motor_data_rf_prev(0))/dt;
        motor_data_d_rf(1) = (-df.iloc("state_beta_b", i) - motor_data_rf_prev(1))/dt;
        motor_data_d_rh(0) = (df.iloc("state_theta_c", i) - motor_data_rh_prev(0))/dt;
        motor_data_d_rh(1) = (-df.iloc("state_beta_c", i) - motor_data_rh_prev(1))/dt;
        motor_data_d_lh(0) = (df.iloc("state_theta_d", i) - motor_data_lh_prev(0))/dt;
        motor_data_d_lh(1) = (df.iloc("state_beta_d", i) - motor_data_lh_prev(1))/dt;
        motor_data_lf_prev(0) = df.iloc("state_theta_a", i);
        motor_data_lf_prev(1) = df.iloc("state_beta_a", i);
        motor_data_rf_prev(0) = df.iloc("state_theta_b", i);
        motor_data_rf_prev(1) = -df.iloc("state_beta_b", i);
        motor_data_rh_prev(0) = df.iloc("state_theta_c", i);
        motor_data_rh_prev(1) = -df.iloc("state_beta_c", i);
        motor_data_lh_prev(0) = df.iloc("state_theta_d", i);
        motor_data_lh_prev(1) = df.iloc("state_beta_d", i);

        Eigen::Vector<float, 5> encoder_lf(motor_data_lf_prev(0), motor_data_lf_prev(1), motor_data_d_lf(1), df.iloc("imu_ang_vel_y", i), motor_data_d_lf(0));
        Eigen::Vector<float, 5> encoder_rf(motor_data_rf_prev(0), motor_data_rf_prev(1), motor_data_d_rf(1), df.iloc("imu_ang_vel_y", i), motor_data_d_rf(0));
        Eigen::Vector<float, 5> encoder_rh(motor_data_rh_prev(0), motor_data_rh_prev(1), motor_data_d_rh(1), df.iloc("imu_ang_vel_y", i), motor_data_d_rh(0));
        Eigen::Vector<float, 5> encoder_lh(motor_data_lh_prev(0), motor_data_lh_prev(1), motor_data_d_lh(1), df.iloc("imu_ang_vel_y", i), motor_data_d_lh(0));

        Eigen::Matrix3f R = q.toRotationMatrix();
        u.push_data(a, w, dt);

        // lidar
        float alpha_lf = -100; 
        float alpha_rf = -100;
        float alpha_rh = -100;
        float alpha_lh = -100;

        // float alpha_lf, alpha_rf, alpha_rh, alpha_lh;
        // if (counter % 50) {
        //     alpha_lf = -100;
        //     alpha_rf = -100;
        //     alpha_rh = -100;
        //     alpha_lh = -100;
        // }
        // else {
        //     alpha_lf = - atan2(df.iloc("lf.dist", i) - df.iloc("lh.dist", i) , 0.357);
        //     alpha_rf = - atan2(df.iloc("rf.dist", i) - df.iloc("rh.dist", i) , 0.357);
        //     alpha_rh = - atan2(df.iloc("rf.dist", i) - df.iloc("rh.dist", i) , 0.357);
        //     alpha_lh = - atan2(df.iloc("lf.dist", i) - df.iloc("lh.dist", i) , 0.357);
        // }
        lf.push_data(encoder_lf, w, dt, alpha_lf);
        rf.push_data(encoder_rf, w, dt, alpha_rf);
        rh.push_data(encoder_rh, w, dt, alpha_rh);
        lh.push_data(encoder_lh, w, dt, alpha_lh);

        // // t265
        // if ((t265_pose - Eigen::Vector3f(df.iloc("t265.x", i), df.iloc("t265.y", i), df.iloc("t265.z", i))).norm() > 0)
        //     usec += 1; // FIXME: logic error, this parameter is used to check if the t265 data is updated
        // t265_pose = Eigen::Vector3f(df.iloc("t265.x", i), df.iloc("t265.y", i), df.iloc("t265.z", i));
        // t265.push_data(t265_pose,
        // qt265.toRotationMatrix(), Eigen::Matrix3f::Identity() * 9e-6, sec, usec);


        filter.predict();

        // if (t265.is_update()) {
        //     filter.certain_valid(&t265_pkld);
        // }
        
        filter.valid();
        x = filter.state();
        Eigen::Matrix3f P_cov = filter.Y_inv.block<3, 3>(3*j-3, 3*j-3);
        p += rot * R * R_init.transpose() * x.segment(3 * j - 3, 3) * dt;
        estimate_state.row(counter).segment(0, 3) = x.segment(3 * j - 3, 3);
        estimate_state.row(counter).segment(3, 3) = p;
        estimate_state.row(counter).segment(6, 3) = 1. / dt / (float) j * lf.z(dt);
        estimate_state.row(counter).segment(9, 3) = 1. / dt / (float) j * rf.z(dt);
        estimate_state.row(counter).segment(12, 3) = 1. / dt / (float) j * rh.z(dt);
        estimate_state.row(counter).segment(15, 3) = 1. / dt / (float) j * lh.z(dt);
        // estimate_state.row(counter).segment(18, 3) = 1. / dt / (float) j * t265.z(dt);
        estimate_state.row(counter).segment(21, 3) = x.segment(6 * j - 3, 3);
        estimate_state.row(counter).segment(24, 4) = Eigen::Vector4f(filter.exclude[0], filter.exclude[1], filter.exclude[2], filter.exclude[3]);
        estimate_state.row(counter).segment(28, 4) = Eigen::Vector<float, 4>(filter.scores[0], filter.scores[1], filter.scores[2], filter.scores[3]);
        // estimate_state.row(counter).segment(32, 4) = Eigen::Vector4f(df.iloc("lf.contact", i), df.iloc("rf.contact", i), df.iloc("rh.contact", i), df.iloc("lh.contact", i));
        estimate_state.row(counter)(36) = filter.threshold;
        estimate_state.row(counter).segment(37, 9) = Eigen::Map<const Eigen::VectorXf>(P_cov.data(), P_cov.size());
        counter ++;
    }
    
    std::vector<std::string> cols = {
        "v_.x", "v_.y", "v_.z", 
        "p.x", "p.y", "p.z", 
        "zLF.x", "zLF.y", "zLF.z", 
        "zRF.x", "zRF.y", "zRF.z", 
        "zRH.x", "zRH.y", "zRH.z", 
        "zLH.x", "zLH.y", "zLH.z", 
        "zP.x", "zP.y", "zP.z", 
        "ba.x", "ba.y", "ba.z", 
        "lf.contact","rf.contact","rh.contact","lh.contact",
        "lf.cscore","rf.cscore","rh.cscore","lh.cscore",
        "lf.c","rf.c","rh.c","lh.c",
        "threshold",
        "cov.xx", "cov.xy", "cov.xz", "cov.yx", "cov.yy", "cov.yz", "cov.zx", "cov.zy", "cov.zz"
    };
    DataProcessor::write_csv(estimate_state, filepath+"out_"+filename+"_kld"+".csv", cols);
    return 0;
}