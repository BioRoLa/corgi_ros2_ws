#include "OfflineTestNode.hpp"
#include "Config.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <filesystem>

namespace corgi {

OfflineTestNode::OfflineTestNode(const Params& params)
    : params_(params) {}

RawRecord OfflineTestNode::to_raw(const CSVReader::RobotData& d) {
    RawRecord r;
    r.sim_pos_x = d.sim_pos_x;  r.sim_pos_y = d.sim_pos_y;  r.sim_pos_z = d.sim_pos_z;
    r.sim_orien_x = d.sim_orien_x; r.sim_orien_y = d.sim_orien_y;
    r.sim_orien_z = d.sim_orien_z; r.sim_orien_w = d.sim_orien_w;
    r.imu_orien_x = d.imu_orien_x; r.imu_orien_y = d.imu_orien_y;
    r.imu_orien_z = d.imu_orien_z; r.imu_orien_w = d.imu_orien_w;
    r.imu_ang_vel_x = d.imu_ang_vel_x; r.imu_ang_vel_y = d.imu_ang_vel_y; r.imu_ang_vel_z = d.imu_ang_vel_z;
    r.imu_lin_acc_x = d.imu_lin_acc_x; r.imu_lin_acc_y = d.imu_lin_acc_y; r.imu_lin_acc_z = d.imu_lin_acc_z;
    r.state_theta_a = d.state_theta_a; r.state_beta_a = d.state_beta_a;
    r.state_vel_r_a = d.state_vel_r_a; r.state_vel_l_a = d.state_vel_l_a;
    r.state_trq_r_a = d.state_trq_r_a; r.state_trq_l_a = d.state_trq_l_a;
    r.state_theta_b = d.state_theta_b; r.state_beta_b = d.state_beta_b;
    r.state_vel_r_b = d.state_vel_r_b; r.state_vel_l_b = d.state_vel_l_b;
    r.state_trq_r_b = d.state_trq_r_b; r.state_trq_l_b = d.state_trq_l_b;
    r.state_theta_c = d.state_theta_c; r.state_beta_c = d.state_beta_c;
    r.state_vel_r_c = d.state_vel_r_c; r.state_vel_l_c = d.state_vel_l_c;
    r.state_trq_r_c = d.state_trq_r_c; r.state_trq_l_c = d.state_trq_l_c;
    r.state_theta_d = d.state_theta_d; r.state_beta_d = d.state_beta_d;
    r.state_vel_r_d = d.state_vel_r_d; r.state_vel_l_d = d.state_vel_l_d;
    r.state_trq_r_d = d.state_trq_r_d; r.state_trq_l_d = d.state_trq_l_d;
    return r;
}

int OfflineTestNode::run() {
    const bool quiet = params_.quiet;
    const double dt = Config::DT;
    const size_t start_index = static_cast<size_t>(params_.start_index);
    const size_t max_processed = params_.max_processed;
    const size_t rmse_skip = params_.rmse_skip;

    // ── Load CSV ────────────────────────────────────────────────
    const auto csv_file_path = (std::filesystem::path(__FILE__).parent_path().parent_path()
                                / "data" / (params_.csv_filename + ".csv")).string();
    if (!quiet) std::cout << "Loading data...\n";
    CSVReader reader;
    auto data = reader.read_csv(csv_file_path);
    if (!quiet) std::cout << "✓ Loaded " << data.size() << " records\n";

    // ── Processor & Pipeline ────────────────────────────────────
    DataProcessor processor(dt);
    EstimationPipeline pipeline(params_);

    // ── IMU noise simulator (optional) ──────────────────────────
    std::unique_ptr<ImuNoiseSimulator> imu_noise_sim;
    if (params_.simulate_imu_noise) {
        imu_noise_sim = std::make_unique<ImuNoiseSimulator>(dt, params_.imu_noise_seed);
        if (!quiet) std::cout << "IMU noise simulation ENABLED (seed=" << params_.imu_noise_seed
                              << ", fs=" << (1.0 / dt) << " Hz)\n";
    }

    // ── Pre-compute GT velocity: central-diff + IIR LPF ────────
    const size_t N_data = data.size();
    std::vector<double> gt_vx_w(N_data, 0.0), gt_vy_w(N_data, 0.0), gt_vz_w(N_data, 0.0);
    for (size_t k = 0; k < N_data; ++k) {
        size_t km = (k == 0) ? 0 : k - 1;
        size_t kp = (k == N_data - 1) ? N_data - 1 : k + 1;
        double denom = (k == 0 || k == N_data - 1) ? dt : 2.0 * dt;
        gt_vx_w[k] = (data[kp].sim_pos_x - data[km].sim_pos_x) / denom;
        gt_vy_w[k] = (data[kp].sim_pos_y - data[km].sim_pos_y) / denom;
        gt_vz_w[k] = (data[kp].sim_pos_z - data[km].sim_pos_z) / denom;
    }
    const double gt_vel_alpha = 1.0 - std::exp(-2.0 * M_PI * params_.gt_velocity_lpf_cutoff * dt);
    for (size_t k = 1; k < N_data; ++k) {
        gt_vx_w[k] = (1.0 - gt_vel_alpha) * gt_vx_w[k - 1] + gt_vel_alpha * gt_vx_w[k];
        gt_vy_w[k] = (1.0 - gt_vel_alpha) * gt_vy_w[k - 1] + gt_vel_alpha * gt_vy_w[k];
        gt_vz_w[k] = (1.0 - gt_vel_alpha) * gt_vz_w[k - 1] + gt_vel_alpha * gt_vz_w[k];
    }

    // ── Output CSV ──────────────────────────────────────────────
    const auto output_dir = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "output_data";
    std::filesystem::create_directories(output_dir);
    const auto esekf_out_path = output_dir / (params_.csv_filename + "_esekf.csv");
    std::ofstream esekf_out(esekf_out_path);
    esekf_out << "Index,sim_pos_x,sim_pos_y,sim_pos_z,"
              << "est_pos_x,est_pos_y,est_pos_z,"
              << "est_vel_x,est_vel_y,est_vel_z,"
              << "contact_a,contact_b,contact_c,contact_d,"
              << "z_avg_x,z_avg_y,z_avg_z,"
              << "ba_x,ba_y,ba_z,bw_x,bw_y,bw_z,bv_x,bv_y,bv_z,"
              << "est_qw,est_qx,est_qy,est_qz,"
              << "gt_qw,gt_qx,gt_qy,gt_qz,"
              << "d2_a,d2_b,d2_c,d2_d,"
              << "rejected_a,rejected_b,rejected_c,rejected_d,"
              << "innov_x_a,innov_y_a,innov_z_a,"
              << "innov_x_b,innov_y_b,innov_z_b,"
              << "innov_x_c,innov_y_c,innov_z_c,"
              << "innov_x_d,innov_y_d,innov_z_d,"
              << "S_xx_a,S_yy_a,S_zz_a,"
              << "S_xx_b,S_yy_b,S_zz_b,"
              << "S_xx_c,S_yy_c,S_zz_c,"
              << "S_xx_d,S_yy_d,S_zz_d,"
              << "P_vx,P_vy,P_vz,"
              << "imu_pos_x,imu_pos_y,imu_pos_z,imu_vel_bx,imu_vel_by,imu_vel_bz\n";
    esekf_out << std::fixed << std::setprecision(8);

    // ── GT offset (ESEKF starts at origin) ──────────────────────
    const double gt_offset_x = data[start_index].sim_pos_x;
    const double gt_offset_y = data[start_index].sim_pos_y;
    const double gt_offset_z = data[start_index].sim_pos_z;

    // ── RMSE accumulators ───────────────────────────────────────
    RmseAccumulator pos_rmse, vel_rmse;
    RmseAccumulator imu_pos_rmse, imu_vel_rmse;

    // IMU pure integration state
    Eigen::Vector3f imu_vel_w = Eigen::Vector3f::Zero();
    Eigen::Vector3f imu_pos_w = Eigen::Vector3f::Zero();

    if (!quiet) std::cout << "\nStarting data processing...\n";
    if (!quiet) std::cout << "Noise params: sigma_a=[" << params_.sigma_a.transpose()
                          << "], sigma_leg_vec=[" << params_.sigma_leg_vec.transpose() << "]\n";
    if (!quiet) std::cout << "GMO input:   " << (params_.use_esekf_state ? "ESEKF estimated state" : "Ground truth (sim_pos)") << "\n";

    auto start_time = std::chrono::high_resolution_clock::now();
    size_t processed_count = 0;

    for (size_t i = start_index; i < data.size() && processed_count < max_processed; ++i) {
        if (!quiet && i % 100 == 0) {
            std::cout << "Processing index: " << i << " / " << data.size() << "\r" << std::flush;
        }

        const auto& d = data[i];
        RawRecord raw = to_raw(d);

        // Process raw data → generalized coordinates
        auto processed = processor.process_record(raw);

        // Override with ESEKF state if requested
        if (params_.use_esekf_state && pipeline.initialized()) {
            pipeline.override_processed_with_esekf(processed);
        }

        // Initialize ESEKF on first iteration
        if (!pipeline.initialized()) {
            Eigen::Quaternionf q0(
                static_cast<float>(d.imu_orien_w),
                static_cast<float>(d.imu_orien_x),
                static_cast<float>(d.imu_orien_y),
                static_cast<float>(d.imu_orien_z));
            pipeline.init(q0);
            if (!quiet) std::cout << "\nES-EKF initialized at index " << i << "\n";
        }

        // IMU measurements
        Eigen::Vector3f a_m(
            static_cast<float>(d.imu_lin_acc_x),
            static_cast<float>(d.imu_lin_acc_y),
            static_cast<float>(d.imu_lin_acc_z));
        Eigen::Vector3f w_m(
            static_cast<float>(d.imu_ang_vel_x),
            static_cast<float>(d.imu_ang_vel_y),
            static_cast<float>(d.imu_ang_vel_z));

        if (imu_noise_sim) {
            imu_noise_sim->apply(a_m, w_m);
        }

        // GT quaternion for IMU pure integration & RMSE
        Eigen::Quaternionf q_gt(
            static_cast<float>(d.sim_orien_w),
            static_cast<float>(d.sim_orien_x),
            static_cast<float>(d.sim_orien_y),
            static_cast<float>(d.sim_orien_z));
        q_gt.normalize();
        Eigen::Vector3f g_w(0.0f, 0.0f, -9.81f);

        // IMU pure integration
        {
            Eigen::Vector3f a_world_imu = q_gt.toRotationMatrix() * a_m + g_w;
            imu_vel_w += a_world_imu * static_cast<float>(dt);
            imu_pos_w += imu_vel_w * static_cast<float>(dt);
        }

        // ── Pipeline step ───────────────────────────────────────
        auto result = pipeline.step(processed, a_m, w_m, raw, i);

        // ── Accumulate RMSE ─────────────────────────────────────
        {
            const auto& st = result.state;
            double ep_x = st.p.x() - (d.sim_pos_x - gt_offset_x);
            double ep_y = st.p.y() - (d.sim_pos_y - gt_offset_y);
            double ep_z = st.p.z() - (d.sim_pos_z - gt_offset_z);

            Eigen::Quaternionf q_gt_rmse(
                static_cast<float>(d.sim_orien_w),
                static_cast<float>(d.sim_orien_x),
                static_cast<float>(d.sim_orien_y),
                static_cast<float>(d.sim_orien_z));
            q_gt_rmse.normalize();
            Eigen::Vector3f gt_vw_i(
                static_cast<float>(gt_vx_w[i]),
                static_cast<float>(gt_vy_w[i]),
                static_cast<float>(gt_vz_w[i]));
            Eigen::Vector3f gt_vb_i = q_gt_rmse.toRotationMatrix().transpose() * gt_vw_i;

            if (processed_count >= rmse_skip) {
                double ev_x = st.v.x() - gt_vb_i.x();
                double ev_y = st.v.y() - gt_vb_i.y();
                double ev_z = st.v.z() - gt_vb_i.z();
                vel_rmse.add(ev_x, ev_y, ev_z);
                pos_rmse.add(ep_x, ep_y, ep_z);

                Eigen::Vector3f imu_vel_b_rmse = q_gt_rmse.toRotationMatrix().transpose() * imu_vel_w;
                imu_vel_rmse.add(imu_vel_b_rmse.x() - gt_vb_i.x(),
                                 imu_vel_b_rmse.y() - gt_vb_i.y(),
                                 imu_vel_b_rmse.z() - gt_vb_i.z());
                imu_pos_rmse.add(imu_pos_w.x() - (d.sim_pos_x - gt_offset_x),
                                 imu_pos_w.y() - (d.sim_pos_y - gt_offset_y),
                                 imu_pos_w.z() - (d.sim_pos_z - gt_offset_z));
            }
        }

        // ── Log CSV row ─────────────────────────────────────────
        {
            const auto& st = result.state;
            const auto& diag = result.diag;
            const auto& Pcov = result.P;
            esekf_out << i << ","
                      << d.sim_pos_x << "," << d.sim_pos_y << "," << d.sim_pos_z << ","
                      << st.p.x() << "," << st.p.y() << "," << st.p.z() << ","
                      << st.v.x() << "," << st.v.y() << "," << st.v.z() << ","
                      << result.contacts[0] << "," << result.contacts[1] << ","
                      << result.contacts[2] << "," << result.contacts[3] << ","
                      << result.z_avg.x() << "," << result.z_avg.y() << "," << result.z_avg.z() << ","
                      << st.ba.x() << "," << st.ba.y() << "," << st.ba.z() << ","
                      << st.bw.x() << "," << st.bw.y() << "," << st.bw.z() << ","
                      << st.bv.x() << "," << st.bv.y() << "," << st.bv.z() << ","
                      << st.q.w() << "," << st.q.x() << "," << st.q.y() << "," << st.q.z() << ","
                      << d.imu_orien_w << "," << d.imu_orien_x << ","
                      << d.imu_orien_y << "," << d.imu_orien_z << ",";
            for (int j = 0; j < 4; ++j) esekf_out << diag[j].d_squared << ",";
            for (int j = 0; j < 4; ++j) esekf_out << (diag[j].rejected ? 1 : 0) << ",";
            for (int j = 0; j < 4; ++j)
                esekf_out << diag[j].innovation.x() << "," << diag[j].innovation.y() << "," << diag[j].innovation.z() << ",";
            for (int j = 0; j < 4; ++j)
                esekf_out << diag[j].S_diag.x() << "," << diag[j].S_diag.y() << "," << diag[j].S_diag.z() << ",";
            esekf_out << Pcov(estimation_model::V_IDX, estimation_model::V_IDX) << ","
                      << Pcov(estimation_model::V_IDX + 1, estimation_model::V_IDX + 1) << ","
                      << Pcov(estimation_model::V_IDX + 2, estimation_model::V_IDX + 2) << ",";
            {
                Eigen::Vector3f imu_vel_b_log = q_gt.toRotationMatrix().transpose() * imu_vel_w;
                esekf_out << imu_pos_w.x() << "," << imu_pos_w.y() << "," << imu_pos_w.z() << ","
                          << imu_vel_b_log.x() << "," << imu_vel_b_log.y() << "," << imu_vel_b_log.z() << "\n";
            }
        }

        processed_count++;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    esekf_out.close();

    if (!quiet) {
        std::cout << "\nProcessing complete\n";
        std::cout << "Disturbance results: output_data/" << params_.csv_filename << "_result.csv\n";
        std::cout << "ESEKF results:       " << esekf_out_path.string() << "\n";
    }

    if (!quiet) {
        std::cout << "\n========== Performance Statistics ==========";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "\nProcessed records: " << processed_count;
        std::cout << "\nElapsed time: " << elapsed.count() << " seconds";
        std::cout << "\nAverage time per record: " << (elapsed.count() / processed_count) << " seconds";
        std::cout << "\nProcessing speed: " << (processed_count / elapsed.count()) << " records/second";
        std::cout << "\n==========================================\n";
    }

    const auto& final_st = pipeline.nominal();
    if (!quiet) {
        std::cout << "\n========== ESEKF Final State ==========\n";
        std::cout << "Position: [" << final_st.p.x() << ", " << final_st.p.y() << ", " << final_st.p.z() << "]\n";
        std::cout << "Velocity: [" << final_st.v.x() << ", " << final_st.v.y() << ", " << final_st.v.z() << "]\n";
        std::cout << "Bias_a:   [" << final_st.ba.x() << ", " << final_st.ba.y() << ", " << final_st.ba.z() << "]\n";
        std::cout << "Bias_w:   [" << final_st.bw.x() << ", " << final_st.bw.y() << ", " << final_st.bw.z() << "]\n";
        std::cout << "Bias_v:   [" << final_st.bv.x() << ", " << final_st.bv.y() << ", " << final_st.bv.z() << "]\n";
        const size_t final_idx = start_index + processed_count - 1;
        double gt_final_x = data[final_idx].sim_pos_x - gt_offset_x;
        double gt_final_y = data[final_idx].sim_pos_y - gt_offset_y;
        double gt_final_z = data[final_idx].sim_pos_z - gt_offset_z;
        std::cout << "GT (off): [" << gt_final_x << ", " << gt_final_y << ", " << gt_final_z << "]\n";
        std::cout << "GT - Est: ["
                  << (gt_final_x - final_st.p.x()) << ", "
                  << (gt_final_y - final_st.p.y()) << ", "
                  << (gt_final_z - final_st.p.z()) << "]\n";
        std::cout << "==========================================\n";
    }

    // RMSE statistics
    std::cout << std::fixed << std::setprecision(6);
    if (pos_rmse.count() > 0) {
        auto p = pos_rmse.rmse();
        auto v = vel_rmse.rmse();
        std::cout << "\n========== RMSE Statistics ==========\n";
        std::cout << "Position RMSE (m):  [" << p.x() << ", " << p.y() << ", " << p.z() << "]\n";
        std::cout << "Velocity RMSE body(m/s):[" << v.x() << ", " << v.y() << ", " << v.z() << "]\n";
        std::cout << "Position RMSE total: " << pos_rmse.total_rmse() << " m\n";
        std::cout << "Velocity RMSE total: " << vel_rmse.total_rmse() << " m/s\n";
        auto ip = imu_pos_rmse.rmse();
        auto iv = imu_vel_rmse.rmse();
        std::cout << "--- IMU pure integration (GT q, no bias) ---\n";
        std::cout << "IMU Pos RMSE (m):     [" << ip.x() << ", " << ip.y() << ", " << ip.z() << "]\n";
        std::cout << "IMU Vel RMSE body(m/s):[" << iv.x() << ", " << iv.y() << ", " << iv.z() << "]\n";
        std::cout << "IMU Pos RMSE total:  " << imu_pos_rmse.total_rmse() << " m\n";
        std::cout << "IMU Vel RMSE total:  " << imu_vel_rmse.total_rmse() << " m/s\n";
        std::cout << "====================================\n";
    }

    return 0;
}

}  // namespace corgi
