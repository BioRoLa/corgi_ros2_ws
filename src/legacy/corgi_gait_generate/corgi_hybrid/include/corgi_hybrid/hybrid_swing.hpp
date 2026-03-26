#include "corgi_utils/leg_model.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <stdexcept>
#include <complex>
#include <iomanip>
#include <Eigen/Dense>
#include <utility>

struct SwingPoint {
    double time;
    double theta;
    double beta;
};

enum class SwingType {
    LINEAR,
    CUBIC,
    FIVETIMES,
    OPTIMIZE
};


class HybridSwing {
public:
    static std::vector<SwingPoint> generate(LegModel& leg, SwingType type, 
                                            double theta_start, double theta_end, 
                                            double beta_start, double beta_end, 
                                            int rim, double alpha, 
                                            Eigen::Vector2d body_vel, Eigen::Vector2d ground_tangent,
                                            int points = 100);
private:
    static Eigen::Vector2d pointOnRimByAlpha(
            LegModel &legmodel,
            double theta, double beta,
            int rim, double alpha);
    static double quintic(double t);
    static double clamp(double value, double min_val, double max_val);
    static double costFunctionAlpha(
        LegModel& leg,
        double theta, double beta,
        double alpha_target,
        double theta_prev, double beta_prev,
        double w_reg = 1e-2,
        double w_theta = 1e3
    );
    static std::pair<double,double> solveThetaBetaForAlpha(
        LegModel& leg,
        double alpha_target,
        double theta_init, double beta_init,
        int max_iters = 50,
        double lr = 1e-2,
        double tol = 1e-6
    );
};
