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

#include "corgi_msgs/TriggerStamped.h"
#include "plane_segmentation.hpp"


PlaneSegmentation* plane_segmentation;
PlaneDistances plane_distances;
corgi_msgs::TriggerStamped trigger_msg;
int cloud_seq = 0;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    cloud_seq = msg->header.seq;
    /* Convert the ROS PointCloud2 message to PCL point cloud */
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    if (!cloud->isOrganized()) {
        ROS_WARN("Point cloud is not organized. Skipping frame.");
        return;
    }

    plane_distances = plane_segmentation->segment_planes(cloud);
}//end cloud_cb


void trigger_cb(const corgi_msgs::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb



int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_segmentation_node");
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub   = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1, trigger_cb);
    plane_segmentation = new PlaneSegmentation;
    plane_segmentation->init_tf();
    ros::Rate rate(10);

    std::ofstream csv("plane_distances.csv");
    csv << "cloud_seq,";
    csv << "Trigger," ;
    csv << "Horizontal0,"; for (int i = 1; i < 10; ++i) csv << "Horizontal" << i << ",";
    csv << "Vertical0,";  for (int i = 1; i < 10; ++i) csv << "Vertical"   << i << ",";
    csv << "\n";

    while (ros::ok()) {
        ros::spinOnce();

        csv << cloud_seq << ",";
        csv << (int)trigger_msg.enable << ",";
        for (int i=0; i<10; i++) {
            if (i < plane_distances.horizontal.size())
                csv << std::fixed << std::setprecision(4) << plane_distances.horizontal[i];
            else
                csv << "0";
            csv << ",";
        }//end for
        for (int i=0; i<10; i++) {
            if (i < plane_distances.vertical.size())
                csv << std::fixed << std::setprecision(4) << plane_distances.vertical[i];
            else 
                csv << "0";
            csv << ",";
        }//end for
        csv << "\n";

        rate.sleep();
    }//end while

    csv.close();
    ros::shutdown();
    return 0;
}//end main
