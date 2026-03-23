#pragma once

#include <array>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "common/Params.hpp"
#include "estimation/SchmittTrigger.hpp"
#include "general_momentum_observer/DataProcessor.hpp"
#include "general_momentum_observer/DisturbanceObserver.hpp"
#include "kinematic/ContactMap.hpp"
#include "kinematic/Leg.hpp"
#include "es_ekf/ESEKF.hpp"
#include "common/Config.hpp"

namespace corgi {

/**
 * @brief Result of a single estimation pipeline step.
 */
struct StepResult {
    estimation_model::NominalState state;
    std::array<bool, 4> contacts{};
    Eigen::VectorXd disturbance;
    std::array<estimation_model::LegUpdateDiag, 4> diag{};
    Eigen::MatrixXf P;
    Eigen::Vector3f z_avg = Eigen::Vector3f::Zero();
    Eigen::Vector3f pred_vel = Eigen::Vector3f::Zero();  // velocity after predict, before update
};

/**
 * @brief Shared estimation pipeline: GMO → Contact → ESEKF.
 *
 * Pure algorithm — no ROS 2 dependency.
 * Used by both OfflineTestNode and LegOdometryNode.
 */
class EstimationPipeline {
public:
    explicit EstimationPipeline(const Params& params);

    /// Initialize ESEKF with an orientation quaternion (call once).
    void init(const Eigen::Quaternionf& q0);

    /// Run one pipeline step: GMO → contact detection → ESEKF predict/update.
    /// @param processed  Generalized coordinates from DataProcessor
    /// @param a_m        Raw accelerometer measurement (body frame)
    /// @param w_m        Raw gyroscope measurement (body frame)
    /// @param raw        Full raw record for leg encoder extraction
    /// @param index      Data index (for observer logging)
    StepResult step(const DataProcessor::ProcessedData& processed,
                    const Eigen::Vector3f& a_m,
                    const Eigen::Vector3f& w_m,
                    const RawRecord& raw,
                    size_t index);

    bool initialized() const { return initialized_; }
    const estimation_model::NominalState& nominal() const { return esekf_.nominal(); }
    const Eigen::MatrixXf& covariance() const { return esekf_.covariance(); }

    /// Override base position/velocity in ProcessedData with ESEKF estimate.
    void override_processed_with_esekf(DataProcessor::ProcessedData& processed) const;

private:
    static Leg createLeg(double x_sign, double y_sign);

    Params params_;

    // Legs (LF, RF, RH, LH)
    Leg lf_leg_, rf_leg_, rh_leg_, lh_leg_;
    Leg* legs_[4];

    // Core estimators
    corgi::DisturbanceObserver observer_;
    estimation_model::ContactMap contact_map_;
    estimation_model::ESEKF esekf_;

    // Contact detection
    std::array<ContactSchmittTrigger, 4> contact_triggers_;

    bool initialized_ = false;
};

}  // namespace corgi
