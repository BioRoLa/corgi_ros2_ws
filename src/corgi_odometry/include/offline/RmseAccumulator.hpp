#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>

namespace corgi {

/**
 * @brief Accumulates per-axis squared errors and computes RMSE.
 */
class RmseAccumulator {
public:
    void add(double ex, double ey, double ez) {
        sum_x_ += ex * ex;
        sum_y_ += ey * ey;
        sum_z_ += ez * ez;
        ++count_;
    }

    void add(const Eigen::Vector3f& err) {
        add(static_cast<double>(err.x()),
            static_cast<double>(err.y()),
            static_cast<double>(err.z()));
    }

    Eigen::Vector3d rmse() const {
        if (count_ == 0) return Eigen::Vector3d::Zero();
        double n = static_cast<double>(count_);
        return {std::sqrt(sum_x_ / n),
                std::sqrt(sum_y_ / n),
                std::sqrt(sum_z_ / n)};
    }

    double total_rmse() const {
        if (count_ == 0) return 0.0;
        double n = static_cast<double>(count_);
        return std::sqrt((sum_x_ + sum_y_ + sum_z_) / n);
    }

    size_t count() const { return count_; }

    void reset() {
        sum_x_ = sum_y_ = sum_z_ = 0.0;
        count_ = 0;
    }

private:
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;
    double sum_z_ = 0.0;
    size_t count_ = 0;
};

}  // namespace corgi
