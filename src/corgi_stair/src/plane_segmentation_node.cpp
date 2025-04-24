#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>

ros::Publisher plane_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);

  if (!cloud->isOrganized()) {
    ROS_WARN("Point cloud is not organized.");
    return;
  }

  // 計算 normal
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  // 分割平面
  pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers(500);
  mps.setAngularThreshold(0.017453 * 2.0); // 2 degrees
  mps.setDistanceThreshold(0.02); // 2cm
  mps.setInputNormals(normals);
  mps.setInputCloud(cloud);

  std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>> regions;
  mps.segment(regions);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_planes(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto& region : regions) {
    const auto& contour = region.getContour();
    all_planes->points.insert(all_planes->points.end(), contour.getContour().points.begin(), contour.getContour().points.end());
}
  all_planes->width = all_planes->points.size();
  all_planes->height = 1;
  all_planes->is_dense = false;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*all_planes, output);
  output.header = input->header;
  plane_pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered/PointCloud2", 1, pointCloudCallback);
  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_planes", 1);

  ros::spin();
  return 0;
}
