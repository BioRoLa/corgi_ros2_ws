#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <cmath>

class StairDetector {
public:
  StairDetector() {
    // 訂閱 ZED X Mini 已註冊的點雲
    sub_ = nh_.subscribe("/zedxm/zed_node/point_cloud/cloud_registered",
                         1, &StairDetector::cloudCb, this);
    ROS_INFO("StairDetector initialized, waiting for point cloud...");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1) 轉成 PCL PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // 2) 下採樣 (可加速)
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*cloud);

    // 3) 移除地面平面
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_floor(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients floor_coeff;
    bool floor_found = removePlane(cloud, no_floor,
                                   Eigen::Vector3f(0,0,1),
                                   0.02f,
                                   floor_coeff);
    if (!floor_found) {
      ROS_WARN("Cannot find floor plane");
      return;
    }

    // 4) 迭代偵測階梯平面
    std::vector<pcl::ModelCoefficients> stair_planes;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining = no_floor;
    for (int i = 0; i < 10; ++i) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ModelCoefficients step_coeff;
      bool found = removePlane(remaining, next_cloud,
                               Eigen::Vector3f(0,0,1),
                               0.05f,
                               step_coeff);
      if (!found) break;
      stair_planes.push_back(step_coeff);
      remaining.swap(next_cloud);
    }

    // 5) 印出每階梯的垂直高度
    ROS_INFO("Detected %lu stair steps:", stair_planes.size());
    for (size_t i = 0; i < stair_planes.size(); ++i) {
      auto& c = stair_planes[i];
      // 平面方程 ax + by + cz + d = 0, 法線是 (a,b,c)
      float a = c.values[0], b = c.values[1], c_z = c.values[2], d = c.values[3];
      float norm = std::sqrt(a*a + b*b + c_z*c_z);
      float height = std::fabs(d) / norm;
      ROS_INFO("  Step %zu → height: %.3f m", i+1, height);
    }
  }

  // helper: RANSAC 平面偵測 + 提取以外的點
  bool removePlane(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr out,
      const Eigen::Vector3f& axis,
      float dist_thresh,
      pcl::ModelCoefficients& coeff_out)
  {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(axis);
    seg.setEpsAngle(10.0f * M_PI / 180.0f);
    seg.setDistanceThreshold(dist_thresh);
    seg.setInputCloud(in);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, coeff_out);
    if (inliers->indices.empty()) return false;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*out);
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "stair_detector");
  StairDetector sd;
  ros::spin();
  return 0;
}
