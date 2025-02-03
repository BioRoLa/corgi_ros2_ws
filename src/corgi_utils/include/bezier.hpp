#ifndef BEZIER_HPP
#define BEZIER_HPP

#include <vector>
#include <array>
#include <cmath>

class Bezier {
    public:
        Bezier() {};    // Default constructor
        Bezier(const std::vector<std::array<double, 2>>& control_pts);

        std::array<double, 2> getBzPoint(double t, double offset_x = 0, double offset_y = 0);

    private:
        std::vector<std::array<double, 2>> control_pts;
        std::vector<int> bz_cff;

        int fact(int n);
        int comb(int n, int k);
        std::vector<int> bz_coeff(const std::vector<std::array<double, 2>>& cp_list);
        std::vector<double> bzt_coeff(const std::vector<std::array<double, 2>>& cp_list, double t);
};//end class Bezier

class SwingProfile {
    public:
        SwingProfile() {};  // Default constructor
        SwingProfile(std::array<double, 2> p_l, std::array<double, 2> p_t, double step_height, int direction);

        std::array<double, 2> getFootendPoint(double t_duty);

        int getDirection();

    private:
        double L, h, dh, dL1, dL2, dL3, dL4;
        double offset_x, offset_y, diff_h;
        int direction;
        std::vector<std::array<double, 2>> control_points;
        Bezier bezier;

        void getControlPoints();
};//end class SwingProfile

#endif // BEZIER_HPP