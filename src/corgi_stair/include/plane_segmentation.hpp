#ifndef PLANESEG_HPP
#define PLANESEG_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <vector>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT_no_color;

struct PlaneDistances {
    std::vector<double> horizontal;
    std::vector<double> vertical;
};

class PlaneSegmentation {
    public:
        PlaneSegmentation();

        void init_tf();
        PlaneDistances segment_planes(pcl::PointCloud<PointT>::Ptr cloud);
        void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud);
        void computeNormals();
        void group_by_normals();
        std::vector<double> segment_by_distances(Eigen::Vector3f centroid, const std::vector<int>& indices);
        Eigen::Vector3f computeCentroid(const std::vector<int>& indices);

        void visualize_planes();
        void visualize_normal();

    private:
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener* tf_listener_;
        pcl::PointCloud<PointT>::Ptr cloud_;
        pcl::PassThrough<PointT> pass_;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> normal_estimator_;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;

        Eigen::Vector3f centroid_z, centroid_x;
        std::vector<int> h_point_idx;   // point index in cloud blongs to horizontal planes
        std::vector<int> v_point_idx;   // point index in cloud blongs to vertical   planes
        const double cos_threshold_ = std::cos(pcl::deg2rad(10.0));

        ros::Publisher pub;
        ros::Publisher normal_pub;
        ros::NodeHandle nh;

        int last_marker_count_ = 0; // 用於記錄上次發佈的 marker 數量
};

#endif // PLANESEG_HPP
