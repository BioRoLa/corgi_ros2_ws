#pragma once

#include <string>
#include <vector>
#include <memory>

#include "Params.hpp"
#include "CSVReader.hpp"
#include "RmseAccumulator.hpp"
#include "EstimationPipeline.hpp"
#include "general_momentum_observer/DataProcessor.hpp"
#include "ImuNoiseSimulator.hpp"

namespace corgi {

/**
 * @brief Offline estimation test node — no ROS 2 dependency.
 *
 * Loads CSV data, runs the estimation pipeline, computes RMSE
 * against ground truth, and writes diagnostic CSV outputs.
 */
class OfflineTestNode {
public:
    explicit OfflineTestNode(const Params& params);

    /// Run the full offline test. Returns 0 on success.
    int run();

private:
    /// Convert CSVReader::RobotData → RawRecord
    static RawRecord to_raw(const CSVReader::RobotData& d);

    Params params_;
};

}  // namespace corgi
