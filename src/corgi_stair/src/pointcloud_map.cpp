#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <mutex>

typedef pcl::PointXYZRGB PointT;

class PointCloudMapper {
public:
    PointCloudMapper() : nh_("~"), cloud_map_(new pcl::PointCloud<PointT>()) {
        sub_ = nh_.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, &PointCloudMapper::pointCloudCallback, this);
        ROS_INFO("Subscribed to point cloud topic.");
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 可以根據需要加上 Transform，例如用 TF 查當前位姿
        // pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);

        *cloud_map_ += *cloud;
        ROS_INFO("Point cloud received. Total points: %lu", cloud_map_->points.size());
    }

    void saveMap(const std::string& filename) {
        std::lock_guard<std::mutex> lock(mutex_);
        pcl::io::savePCDFileBinary(filename, *cloud_map_);
        ROS_INFO("Saved map to %s", filename.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    pcl::PointCloud<PointT>::Ptr cloud_map_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_mapper");
    PointCloudMapper mapper;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("Mapping... Press Ctrl+C to stop.");
    ros::waitForShutdown();

    mapper.saveMap("pointcloud_map.pcd");
    return 0;
}
