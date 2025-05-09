#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
#include <unordered_map>
#include <random>
#include <algorithm>

#include <Eigen/Dense>
#include "plane_segmentation.hpp"


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT_no_color;

PlaneSegmentation::PlaneSegmentation() :
    /* Initializer List */
    normals_(new pcl::PointCloud<pcl::Normal>)
{
    // Initialize
    pass_.setKeepOrganized(true);
    normal_estimator_.setNormalEstimationMethod(normal_estimator_.AVERAGE_3D_GRADIENT);
    // normal_estimator_.setNormalEstimationMethod(normal_estimator_.AVERAGE_DEPTH_CHANGE);
    // normal_estimator_.setNormalEstimationMethod(normal_estimator_.COVARIANCE_MATRIX);
    normal_estimator_.setMaxDepthChangeFactor(0.01f);
    normal_estimator_.setNormalSmoothingSize(10.0f);

}//end PlaneSegmentation


void PlaneSegmentation::init_tf() {
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}//end init_tf


PlaneDistances PlaneSegmentation::segment_planes(pcl::PointCloud<PointT>::Ptr cloud) {
    this->setInputCloud(cloud);
    this->computeNormals();
    this->group_by_normals();
    std::vector<double> h_plane_distances = this->segment_by_distances(centroid_z, h_point_idx);
    std::vector<double> v_plane_distances = this->segment_by_distances(centroid_x, v_point_idx);
    std::reverse(v_plane_distances.begin(), v_plane_distances.end());

    return {h_plane_distances, v_plane_distances};
}//end segment_planes


void PlaneSegmentation::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud) {
    /* Apply ROI filtering */
    pass_.setInputCloud(cloud);
    pass_.setFilterFieldName("x");
    pass_.setFilterLimits(0.20, 1.5);
    pass_.filter(*cloud);
    pass_.setFilterFieldName("y");
    pass_.setFilterLimits(-0.4, 0.4);
    pass_.filter(*cloud);
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(-0.5, 1.0);
    pass_.filter(*cloud);
    /* Transform from camera coord to world coord */
    pcl_ros::transformPointCloud("map", *cloud, *cloud, tf_buffer_);
    /* Set cloud */
    cloud_ = cloud;
    normal_estimator_.setInputCloud(cloud_);
}//end setInputCloud


void PlaneSegmentation::computeNormals() {
    normal_estimator_.compute(*normals_);
}//end computeNormals


void PlaneSegmentation::group_by_normals() {
    centroid_z = Eigen::Vector3f(0, 0, 1);  // initial normal vector for horizontal plane
    centroid_x = Eigen::Vector3f(-1, 0, 0); // initial normal vector for vertical   plane
    h_point_idx.clear();
    v_point_idx.clear();

    for (size_t i = 0; i < normals_->size(); i++) {
        Eigen::Vector3f normal(normals_->points[i].normal_x,
                            normals_->points[i].normal_y,
                            normals_->points[i].normal_z);
        double cos_z = normal.dot(centroid_z);
        double cos_x = normal.dot(centroid_x);

        if (cos_z < cos_threshold_) {   // belongs to horizontal planes
            h_point_idx.push_back(i);
        } else if (cos_x < cos_threshold_) {    // belongs to vertical planes
            v_point_idx.push_back(i);
        }//end if else
    }//end for

    // Update centroid
    centroid_z = computeCentroid(h_point_idx);
    centroid_x = computeCentroid(v_point_idx);
}//end group_by_normals


std::vector<double> PlaneSegmentation::segment_by_distances(Eigen::Vector3f centroid, const std::vector<int>& indices) {
    const double bin_width = 0.001; // 1mm
    const int one_bin_point_threshold = 100;    // 100 points
    const int total_point_threshold   = 1000;    // 5000 points
    const double merge_threshold = 0.05;    // 5cm
    const int num_clusters = 2;

    /* Calculate distances */
    std::vector<double> distances;
    for (size_t i = 0; i < cloud_->size(); i++) {
        const pcl::Normal& n = normals_->points[i];
        if (!std::isnan(n.normal_x) && !std::isnan(n.normal_y) && !std::isnan(n.normal_z)) {
            Eigen::Vector3f position(cloud_->points[i].x,
                cloud_->points[i].y,
                cloud_->points[i].z);
            double d = centroid.dot(position);  // projective distance
            distances.push_back(d);
        }//end if
    }//end for
    if (distances.empty()) return {};

    /* Create histogram by distances */
    // Calculate range of histogram
    auto [min_it, max_it] = std::minmax_element(distances.begin(), distances.end());
    double min_val = *min_it;
    double max_val = *max_it;
    int bin_count = static_cast<int>((max_val - min_val) / bin_width) + 1;
    std::vector<int> histogram(bin_count, 0);
    std::vector<double> bin_values(bin_count, 0);
    // Histogram 
    for (double d : distances) {
        int bin = static_cast<int>((d - min_val) / bin_width);
        histogram[bin]++;
        bin_values[bin] += d;
    }//end for
    
    /* 找出連續高密度 bin */
    bool in_range = false;
    std::vector<double> total_counts;
    std::vector<double> mean_distances;
    double sum_count;
    double sum_value;
    for (int i = 0; i < bin_count; ++i) {   // from smallest distance
        if (histogram[i] >= one_bin_point_threshold) {
            if (!in_range) {
                in_range = true;
                sum_count = histogram[i];
                sum_value = bin_values[i];
            } else {
                sum_count += histogram[i];
                sum_value += bin_values[i];
            }//end if else
        } else if (in_range) {
            if (sum_count >= total_point_threshold) {
                total_counts.push_back(sum_count);
                mean_distances.push_back(sum_value / sum_count);
            }//end if
            in_range = false;
        }//end if else
    }//end for
    if (in_range) {
        if (sum_count >= total_point_threshold) {
            total_counts.push_back(sum_count);
            mean_distances.push_back(sum_value / sum_count);
        }//end if
        in_range = false;
    }//end if

    /* Only keep max bin in a range (merge_threshold) */
    std::vector<bool> keep(mean_distances.size(), true);
    for (size_t i = 0; i < mean_distances.size(); i++) {
        for (size_t j = i + 1; j < mean_distances.size(); j++) {
            double dist = std::abs(mean_distances[i] - mean_distances[j]);
            if (dist < merge_threshold) {
                if (total_counts[i] >= total_counts[j]) {
                    keep[j] = false;
                } else {
                    keep[i] = false;
                }
            }
        }
    }
    std::vector<double> final_distances;
    for (size_t i = 0; i < mean_distances.size(); i++) {
        if (keep[i]) {
            final_distances.push_back(mean_distances[i]);
        }
    }


    return final_distances; // from smalleset to largest
}//end segment_by_distances


Eigen::Vector3f PlaneSegmentation::computeCentroid(const std::vector<int>& indices) {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int idx : indices) {
        const pcl::Normal& n = normals_->points[idx];
        centroid += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
    }//end for
    if (!indices.empty()) {
        centroid.normalize();
    }//end if 
    return centroid;
}//end computeCentroid