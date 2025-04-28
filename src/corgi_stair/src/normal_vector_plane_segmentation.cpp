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
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;
ros::Publisher normal_pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);

    if (!cloud->isOrganized())
    {
        ROS_WARN("Point cloud is not organized. Skipping frame.");
        return;
    }

    // pcl::VoxelGrid<PointT> voxel;
    // voxel.setInputCloud(cloud);
    // voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cm
    // voxel.filter(*cloud);

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

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(15.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // Plane segmentation
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(50);
    mps.setAngularThreshold(0.017453 * 20.0); // 20 degrees in radians
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
    for (auto& point : all_planes->points) {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*all_planes, output);
    output.header = input->header;
    pub.publish(output);



    // 發布法線
    // 可視化 Marker
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_template;
    marker_template.header.frame_id = input->header.frame_id;
    marker_template.type = visualization_msgs::Marker::ARROW;
    marker_template.action = visualization_msgs::Marker::ADD;
    marker_template.scale.x = 0.01;
    marker_template.scale.y = 0.002;
    marker_template.scale.z = 0.002;
    marker_template.color.r = 0.0;
    marker_template.color.g = 1.0;
    marker_template.color.b = 0.0;
    marker_template.color.a = 1.0;

    // 空間分格子平均
    float grid_size = 0.2f;
    std::unordered_map<std::tuple<int, int, int>, pcl::PointNormal, boost::hash<std::tuple<int, int, int>>> grid_map;
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        const auto& nm = normals->points[i];
        if (!pcl::isFinite(pt) || !pcl::isFinite(nm))
            continue;
        int gx = static_cast<int>(std::floor(pt.x / grid_size));
        int gy = static_cast<int>(std::floor(pt.y / grid_size));
        int gz = static_cast<int>(std::floor(pt.z / grid_size));
        auto key = std::make_tuple(gx, gy, gz);
        if (grid_map.find(key) == grid_map.end()) {
            pcl::PointNormal ptn;
            ptn.x = pt.x; ptn.y = pt.y; ptn.z = pt.z;
            ptn.normal_x = nm.normal_x; ptn.normal_y = nm.normal_y; ptn.normal_z = nm.normal_z;
            grid_map[key] = ptn;
        }
    }

    int id = 0;
    for (const auto& kv : grid_map) {
        const auto& pt = kv.second;

        visualization_msgs::Marker arrow = marker_template;
        arrow.id = id++;

        geometry_msgs::Point start, end;
        start.x = pt.x;
        start.y = pt.y;
        start.z = pt.z;
        end.x = pt.x + 0.05 * pt.normal_x;
        end.y = pt.y + 0.05 * pt.normal_y;
        end.z = pt.z + 0.05 * pt.normal_z;
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        marker_array.markers.push_back(arrow);
    }

    normal_pub.publish(marker_array);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("plane_segmentation", 1);
    normal_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_normals", 1);
    ros::spin();
    return 0;
}
