#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>

class MultiPlaneSegmentationNode {
public:
    MultiPlaneSegmentationNode() {
        sub_ = nh_.subscribe("/zedxm/zed_node/point_cloud/cloud_registered/PointCloud2", 1,
                             &MultiPlaneSegmentationNode::cloudCallback, this);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        if (!cloud->isOrganized()) {
            ROS_WARN_THROTTLE(5, "Input cloud is not organized. Skipping...");
            return;
        }

        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);
        ne.compute(*normals);

        // Plane segmentation
        pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
        mps.setMinInliers(1000);
        mps.setAngularThreshold(0.017453 * 2.0); // ~2 degrees
        mps.setDistanceThreshold(0.02); // 2cm
        mps.setInputNormals(normals);
        mps.setInputCloud(cloud);

        std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>> regions;
        mps.segment(regions);

        ROS_INFO_STREAM("Detected " << regions.size() << " planar regions");

        // Optional: visualize contours in terminal with bounding box info
        for (size_t i = 0; i < regions.size(); ++i) {
            Eigen::Vector3f centroid = regions[i].getCentroid();
            ROS_INFO_STREAM("Region " << i << " centroid: "
                                      << centroid[0] << ", " << centroid[1] << ", " << centroid[2]);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_plane_segmentation_node");
    MultiPlaneSegmentationNode node;
    ros::spin();
    return 0;
}
