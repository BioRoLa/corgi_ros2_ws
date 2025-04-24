#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);

    if (!cloud->isOrganized())
    {
        ROS_WARN("Point cloud is not organized. Skipping frame.");
        return;
    }

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(5.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // Plane segmentation
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(50);
    mps.setAngularThreshold(0.017453 * 2.0); // 2 degrees in radians
    mps.setDistanceThreshold(0.02);          // 2cm
    mps.setInputCloud(cloud);
    mps.setInputNormals(normals);

    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
    mps.segment(regions); // 原始版本僅需 regions，無需 refine

    pcl::PointCloud<PointT>::Ptr all_planes(new pcl::PointCloud<PointT>);

    for (size_t i = 0; i < regions.size(); ++i)
    {
        const pcl::PlanarRegion<PointT>& region = regions[i];
        pcl::PointCloud<PointT> contour;
        contour.points = region.getContour();
        contour.width = contour.points.size();
        contour.height = 1;
        contour.is_dense = true;

        *all_planes += contour; // 合併所有輪廓到一起
    }

    // 發布結果
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*all_planes, output);
    output.header = input->header;
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_plane_segmentation_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("multi_plane_segmentation", 1);
    ros::spin();
    return 0;
}
