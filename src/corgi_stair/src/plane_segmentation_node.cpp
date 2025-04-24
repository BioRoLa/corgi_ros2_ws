#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_raw);

    // 降採樣以加速處理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_raw);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cm
    voxel.filter(*cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *working_cloud = *cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes(new pcl::PointCloud<pcl::PointXYZ>);

    int max_planes = 5;
    float distance_threshold = 0.01;  // 1cm
    int min_inliers = 100;

    for (int i = 0; i < max_planes; ++i)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(working_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < min_inliers)
            break;

        // 提取平面點
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(working_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane);

        *all_planes += *plane;

        // 移除平面點，繼續找下一個
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_f);
        working_cloud = cloud_f;
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
    pub = nh.advertise<sensor_msgs::PointCloud2>("multi_plane_segmented", 1);

    ros::spin();
}
