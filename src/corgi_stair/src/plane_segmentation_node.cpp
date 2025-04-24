#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // 設定 RANSAC 分割器
    int max_planes = 5;
    float distance_threshold = 0.01;  // 1cm
    int min_inliers = 100;  // 至少有這麼多點才算一個平面
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *working_cloud = *cloud;
    
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
    
        // 把這個平面加入結果
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(working_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane);
        result_cloud += *plane;
    
        // 從 working_cloud 中移除這個平面
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_f);
        working_cloud = cloud_f;
    }
    
    // 轉回 ROS 訊息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_plane, output);
    output.header = input->header;

    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_plane", 1);

    ros::spin();
}
