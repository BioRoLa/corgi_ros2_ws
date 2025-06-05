#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>

#include "plane_segmentation.hpp"

struct TrackedPlane {
    std::deque<double> recent_distances;

    void add_distance(double d, size_t max_size = 5) {
        recent_distances.push_back(d);
        if (recent_distances.size() > max_size)
            recent_distances.pop_front();
    }

    double average() const {
        if (recent_distances.empty()) return 0.0;
        double sum = 0.0;
        for (double d : recent_distances) sum += d;
        return sum / recent_distances.size();
    }

    bool is_close(double d, double threshold = 0.03) const {
        return std::abs(average() - d) <= threshold;
    }
};

struct PlaneTracker {
    std::vector<TrackedPlane> horizontal_planes;
    std::vector<TrackedPlane> vertical_planes;

    void update_planes(const std::vector<double>& new_distances, std::vector<TrackedPlane>& tracked_planes, bool accept_if_larger) {
        for (double d : new_distances) {
            bool matched = false;
            for (auto& tracked : tracked_planes) {
                if (tracked.is_close(d)) {
                    tracked.add_distance(d);
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                // 僅當距離比現有極值「更大/更小」才新增
                bool allow_insert = false;
                if (tracked_planes.empty()) {
                    allow_insert = true;  // 第一個平面直接接受
                } else {
                    double reference = tracked_planes.back().average();
                    allow_insert = accept_if_larger? (d > reference) : (d < reference);
                }

                if (allow_insert) {
                    TrackedPlane new_plane;
                    new_plane.add_distance(d);
                    tracked_planes.push_back(new_plane);
                }
            }
        }
    }

    void update(const PlaneDistances& new_distances) {
        update_planes(new_distances.horizontal, horizontal_planes, true);
        update_planes(new_distances.vertical, vertical_planes, false);
    }

    std::vector<double> get_horizontal_averages() const {
        std::vector<double> result;
        for (const auto& p : horizontal_planes)
            result.push_back(p.average());
        return result;
    }

    std::vector<double> get_vertical_averages() const {
        std::vector<double> result;
        for (const auto& p : vertical_planes)
            result.push_back(p.average());
        return result;
    }
};
