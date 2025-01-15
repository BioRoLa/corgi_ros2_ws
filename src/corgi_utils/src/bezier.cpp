#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>

class Bezier {
    public:
        Bezier(const std::vector<std::array<double, 2>>& control_pts) : control_pts(control_pts) {
            bz_cff = bz_coeff(control_pts);
        }

        double* getBzPoint(double t, double offset_x = 0, double offset_y = 0) {
            auto bzt_cff = bzt_coeff(control_pts, t);
            double x = 0;
            double y = 0;
            for (size_t i = 0; i < control_pts.size(); ++i) {
                x += bzt_cff[i] * bz_cff[i] * control_pts[i][0];
                y += bzt_cff[i] * bz_cff[i] * control_pts[i][1];
            }
            x += offset_x;
            y += offset_y;
            double xy[2] = {x, y};
            return xy;
        }

    private:
        std::vector<std::array<double, 2>> control_pts;
        std::vector<int> bz_cff;

        int fact(int n) {
            return (n == 0 || n == 1) ? 1 : n * fact(n - 1);
        }

        int comb(int n, int k) {
            return fact(n) / (fact(k) * fact(n - k));
        }

        std::vector<int> bz_coeff(const std::vector<std::array<double, 2>>& cp_list) {
            int sz = cp_list.size();
            std::vector<int> bzc(sz);
            for (int i = 0; i < sz; ++i) {
                bzc[i] = comb(sz - 1, i);
            }
            return bzc;
        }

        std::vector<double> bzt_coeff(const std::vector<std::array<double, 2>>& cp_list, double t) {
            int sz = cp_list.size();
            std::vector<double> bzc(sz);
            for (int i = 0; i < sz; ++i) {
                double ord_t_1 = static_cast<double>((sz - 1) - i);
                double ord_t = static_cast<double>(i);
                bzc[i] = std::pow((1 - t), ord_t_1) * std::pow(t, ord_t);
            }
            return bzc;
        }
};

class SwingProfile {
    public:
        SwingProfile(double L, double h, double dh, double dL1, double dL2, double dL3, double dL4, double offset_x = 0, double offset_y = 0, double diff_h = 0)
            : L(L), h(h), dh(dh), dL1(dL1), dL2(dL2), dL3(dL3), dL4(dL4), offset_x(offset_x), offset_y(offset_y), diff_h(diff_h), bezier(control_points) {
            getControlPoint();
        }

        double* getFootendPoint(double t_duty) {
            return bezier.getBzPoint(t_duty, offset_x, offset_y);
        }

    private:
        double L, h, dh, dL1, dL2, dL3, dL4;
        std::vector<std::array<double, 2>> control_points;
        double offset_x, offset_y, diff_h;
        Bezier bezier;

        void getControlPoint() {
            std::array<double, 2> c0 = {0, 0};
            std::array<double, 2> c1 = {c0[0] - dL1, c0[1]};
            std::array<double, 2> c2 = {c1[0] - dL2, c1[1] + h};
            std::array<double, 2> c3 = c2;
            std::array<double, 2> c4 = c2;
            std::array<double, 2> c5 = {c4[0] + 0.5 * L + dL1 + dL2, c4[1]};
            std::array<double, 2> c6 = c5;
            std::array<double, 2> c7 = {c5[0], c5[1] + dh};
            std::array<double, 2> c8 = {c7[0] + 0.5 * L + dL3 + dL4, c7[1]};
            std::array<double, 2> c9 = c8;
            std::array<double, 2> c10 = {c8[0] - dL4, c8[1] - h - dh + diff_h};
            std::array<double, 2> c11 = {c10[0] - dL3, c10[1]};

            control_points = {c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11};
        }
};
