#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/visualization/cloud_viewer.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Step 1: 轉換為有序點雲
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    if (!cloud->isOrganized())
    {
        ROS_WARN("點雲不是有序的，無法使用此方法");
        return;
    }

    // Step 2: 使用積分影像快速估算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);  // 深度變化容忍度
    ne.setNormalSmoothingSize(10.0f);   // 法向量平滑區域
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // Step 3: 使用多平面分割演算法
    std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>> regions;
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(1000);
    mps.setAngularThreshold(0.017453 * 2.0); // 2 degrees
    mps.setDistanceThreshold(0.02);          // 2cm
    mps.setInputCloud(cloud);
    mps.setInputNormals(normals);
    mps.segmentAndRefine(regions);

    // Step 4: 將平面合併後輸出
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < regions.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_cloud = regions[i].getContour();
        *output += *region_cloud;
    }

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*output, out_msg);
    out_msg.header = input->header;
    pub.publish(out_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_plane_segmentation_rgbd");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_planes", 1);
    ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);

    ros::spin();
    return 0;
}
