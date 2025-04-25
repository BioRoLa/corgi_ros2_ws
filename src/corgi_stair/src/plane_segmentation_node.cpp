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

    // Set x,y,z range
    pcl::PassThrough<pcl::PointXYZRGB> pass;
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

    // // Estimate normals
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    // ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    // ne.setMaxDepthChangeFactor(0.05f);
    // ne.setNormalSmoothingSize(15.0f);
    // ne.setInputCloud(cloud);
    // ne.compute(*normals);

    // // Plane segmentation
    // pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    // mps.setMinInliers(50);
    // mps.setAngularThreshold(0.017453 * 20.0); // 20 degrees in radians
    // mps.setDistanceThreshold(0.02);          // 2cm
    // mps.setInputCloud(cloud);
    // mps.setInputNormals(normals);

    // std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
    // mps.segment(regions); // 原始版本僅需 regions，無需 refine

    // pcl::PointCloud<PointT>::Ptr all_planes(new pcl::PointCloud<PointT>);

    // for (size_t i = 0; i < regions.size(); ++i)
    // {
    //     const pcl::PlanarRegion<PointT>& region = regions[i];
    //     pcl::PointCloud<PointT> contour;
    //     contour.points = region.getContour();
    //     contour.width = contour.points.size();
    //     contour.height = 1;
    //     contour.is_dense = true;

    //     *all_planes += contour; // 合併所有輪廓到一起
    // }

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
    for (auto& point : all_planes->points) {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
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
