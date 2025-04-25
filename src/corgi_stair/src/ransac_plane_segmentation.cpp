#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

// typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

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

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cm
    voxel.filter(*cloud);

    // Set x,y,z range
    pcl::PassThrough<PointT> pass;
    pass.setKeepOrganized(true);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 3.0);
    pass.filter(*cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.0, 1.0);
    pass.filter(*cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.5, 1.0);
    pass.filter(*cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *working_cloud = *cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes(new pcl::PointCloud<pcl::PointXYZ>);

    int max_planes = 5;
    float distance_threshold = 0.02;  // 1cm
    int min_inliers = 100;

    for (int i = 0; i < max_planes; ++i)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);    
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setAxis(Eigen::Vector3f::UnitX());          // X 軸
        seg.setEpsAngle(15.0 * M_PI / 180.0);           // ±15°
        seg.setMaxIterations(200);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_rgb->width = all_planes->width;
    cloud_rgb->height = all_planes->height;
    cloud_rgb->points.resize(cloud_rgb->width * cloud_rgb->height);

    // 轉換：將 pcl::PointXYZ 轉換成 pcl::PointXYZRGB 並設定顏色為紅色
    for (size_t i = 0; i < all_planes->points.size(); ++i) {
        cloud_rgb->points[i].x = all_planes->points[i].x;
        cloud_rgb->points[i].y = all_planes->points[i].y;
        cloud_rgb->points[i].z = all_planes->points[i].z;

        // 設置顏色為紅色 (r=255, g=0, b=0)
        cloud_rgb->points[i].r = 255;  // Red
        cloud_rgb->points[i].g = 0;    // Green
        cloud_rgb->points[i].b = 0;    // Blue
    }

    // 發布結果
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_rgb, output);
    output.header = input->header;
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("plane_segmentation", 1);
    ros::spin();
    return 0;
}
