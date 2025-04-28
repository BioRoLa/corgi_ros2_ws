#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <unordered_map>
#include <random>

#include <dbscan.hpp>


#define INTERGRAL_IMAGE_NORMAL_ESTIMATION 1

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT_no_color;

ros::Publisher pub;
ros::Publisher normal_pub;


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    /* Step 1: Convert the ROS PointCloud2 message to PCL point cloud */
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);
    if (!cloud->isOrganized())
    {
        ROS_WARN("Point cloud is not organized. Skipping frame.");
        return;
    }


    /* Step 2: Set ROI */
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


    /* Step 3: Find normal vector for each point */
    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    #if INTERGRAL_IMAGE_NORMAL_ESTIMATION
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    // ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    // ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(10.0f);
    ne.setNormalSmoothingSize(5.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    #else // too slow
    pcl::PointCloud<PointT_no_color>::Ptr cloud_no_color(new pcl::PointCloud<PointT_no_color>);
    pcl::fromROSMsg(*input, *cloud_no_color);
    pcl::NormalEstimation<PointT_no_color, pcl::Normal> ne;
    pcl::search::KdTree<PointT_no_color>::Ptr tree(new pcl::search::KdTree<PointT_no_color>);
    ne.setInputCloud(cloud_no_color);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 設置搜索半徑
    ne.compute(*normals);
    #endif


    /* Step 4: Implementing DBSCAN for clustering */
    vector<Point> normal_points;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!std::isnan(normals->points[i].normal_x) && !std::isnan(normals->points[i].normal_y) && !std::isnan(normals->points[i].normal_z)) {
            Point point;
            point.x = normals->points[i].normal_x;
            point.y = normals->points[i].normal_y;
            point.z = normals->points[i].normal_z;
            point.clusterID = UNCLASSIFIED;
            normal_points.push_back(point);
        } else {
            // 法線無效，可以選擇跳過該點或給予默認值
            // 這裡選擇跳過無效法線的點
            continue;
        }
    }
    DBSCAN ds(2, 0.1*0.1, normal_points); // minimum number of cluster, distance for clustering(metre^2), points
    ds.run();

    // Step 5: Visualize the clusters (use random colors)
    pcl::PointCloud<PointT>::Ptr colored_cloud(new pcl::PointCloud<PointT>(*cloud));

    int idx = 0;
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<int> dist(0, 255);

    // 現在給每個 colored_cloud 的點塗上對應顏色
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!std::isnan(normals->points[i].normal_x) && !std::isnan(normals->points[i].normal_y) && !std::isnan(normals->points[i].normal_z)) {
            int cluster_id = normal_points[idx].clusterID;
            if (cluster_id != UNCLASSIFIED) {
                uint8_t r = static_cast<uint8_t>(128 + (cluster_id * 53) % 127);
                uint8_t g = static_cast<uint8_t>(128 + (cluster_id * 97) % 127);
                uint8_t b = static_cast<uint8_t>(128 + (cluster_id * 223) % 127);
                colored_cloud->points[i].r = r;
                colored_cloud->points[i].g = g;
                colored_cloud->points[i].b = b;
            } else {
                // 不是有效分類的點，塗成灰色
                colored_cloud->points[i].r = 128;
                colored_cloud->points[i].g = 128;
                colored_cloud->points[i].b = 128;
            }
            idx ++;
        } else {
            // 沒有有效 normal 的點，塗成黑色
            colored_cloud->points[i].r = 0;
            colored_cloud->points[i].g = 0;
            colored_cloud->points[i].b = 0;
        }
    }

    // Publish the result
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = input->header;
    pub.publish(output);

    
    // // /* Step 4: Plane segmentation using normal vector */
    // // // Setting
    // // pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    // // mps.setMinInliers(10);
    // // mps.setAngularThreshold(0.017453 * 20.0); // 20 degrees in radians
    // // mps.setDistanceThreshold(0.10);          // 5cm
    // // mps.setInputCloud(cloud);
    // // mps.setInputNormals(normals);

    // // std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
    // // std::vector<pcl::ModelCoefficients> model_coefficients;
    // // std::vector<pcl::PointIndices> inlier_indices;
    // // pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    // // std::vector<pcl::PointIndices> label_indices;
    // // std::vector<pcl::PointIndices> boundary_indices;
    // // // mps.segment(regions); // 原始版本僅需 regions，無需 refine
    // // // mps.segmentAndRefine(regions); // refine
    // // mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);


    // /* Step 5: Merge similar planes based on normal and distance */
    // const float normal_threshold = std::cos(40.0 * M_PI / 180.0); // 10度以内允許合併
    // const float distance_threshold = 0.10; // 平面到原點距離誤差5cm以内允許合併

    // std::vector<bool> merged(model_coefficients.size(), false);
    // std::vector<int> plane_labels(cloud->size(), -1); // 記錄每個點屬於哪個合併後的平面
    // int plane_id = 0;
    // for (size_t i = 0; i < model_coefficients.size(); ++i)
    // {
    //     if (merged[i])
    //         continue;

    //     // 這是 base 平面
    //     const auto& coeff_base = model_coefficients[i];
    //     Eigen::Vector3f normal_base(coeff_base.values[0], coeff_base.values[1], coeff_base.values[2]);
    //     normal_base.normalize();
    //     float d_base = coeff_base.values[3] / normal_base.norm();

    //     // 對其他平面找可合併的
    //     merged[i] = true;
    //     for (size_t j = i + 1; j < model_coefficients.size(); ++j) {
    //         if (merged[j])
    //             continue;

    //         const auto& coeff_other = model_coefficients[j];
    //         Eigen::Vector3f normal_other(coeff_other.values[0], coeff_other.values[1], coeff_other.values[2]);
    //         normal_other.normalize();
    //         float d_other = coeff_other.values[3] / normal_other.norm();

    //         float dot_product = std::abs(normal_base.dot(normal_other));
    //         float distance_diff = std::abs(d_base - d_other);

    //         if (dot_product > normal_threshold && distance_diff < distance_threshold)
    //         {
    //             merged[j] = true;

    //             // 將 j 平面的 inlier 也歸類到 i 平面
    //             for (int idx : inlier_indices[j].indices)
    //                 plane_labels[idx] = plane_id;
    //         }
    //     }

    //     // 把自己也標上 plane_id
    //     for (int idx : inlier_indices[i].indices)
    //         plane_labels[idx] = plane_id;

    //     ++plane_id;
    // }

    // // 重新上色
    // pcl::PointCloud<PointT>::Ptr merged_color_cloud(new pcl::PointCloud<PointT>(*cloud));
    // for (size_t idx = 0; idx < plane_labels.size(); ++idx)
    // {
    //     if (plane_labels[idx] >= 0 && merged_color_cloud->points[idx].z > -0.20)
    //     {
    //         uint8_t r = static_cast<uint8_t>((plane_labels[idx] * 53) % 255);
    //         uint8_t g = static_cast<uint8_t>((plane_labels[idx] * 97) % 255);
    //         uint8_t b = static_cast<uint8_t>((plane_labels[idx] * 223) % 255);

    //         merged_color_cloud->points[idx].r = r;
    //         merged_color_cloud->points[idx].g = g;
    //         merged_color_cloud->points[idx].b = b;
    //     }
    // }

    // // 發布合併後結果
    // sensor_msgs::PointCloud2 merged_output;
    // pcl::toROSMsg(*merged_color_cloud, merged_output);
    // merged_output.header = input->header;
    // pub.publish(merged_output);


    // // 隨機顏色產生器
    // // bool random_color = false;
    // // std::mt19937 rng;
    // // rng.seed(std::random_device()());
    // // std::uniform_int_distribution<int> color_dist(0, 255);
    // // // 複製原始點雲（要修改顏色）
    // // pcl::PointCloud<PointT>::Ptr color_cloud(new pcl::PointCloud<PointT>(*cloud));
    // // // 把每個平面的 inliers 上色
    // // for (size_t i = 0; i < inlier_indices.size(); ++i)
    // // {
    // //     uint8_t r = color_dist(rng);
    // //     uint8_t g = color_dist(rng);
    // //     uint8_t b = color_dist(rng);

    // //     for (size_t j = 0; j < inlier_indices[i].indices.size(); ++j)
    // //     {
    // //         int idx = inlier_indices[i].indices[j];
    // //         color_cloud->points[idx].r = random_color? r: 255;
    // //         color_cloud->points[idx].g = random_color? g: 0;
    // //         color_cloud->points[idx].b = random_color? b: 0;
    // //     }
    // // }


    // // pcl::PointCloud<PointT>::Ptr all_planes(new pcl::PointCloud<PointT>);
    // // for (size_t i = 0; i < regions.size(); ++i)
    // // {
    // //     const pcl::PlanarRegion<PointT>& region = regions[i];
    // //     pcl::PointCloud<PointT> contour;
    // //     contour.points = region.getContour();
    // //     contour.width = contour.points.size();
    // //     contour.height = 1;
    // //     contour.is_dense = true;

    // //     *all_planes += contour; // 合併所有輪廓到一起
    // // }
    // // for (auto& point : all_planes->points) {
    // //     point.r = 255;
    // //     point.g = 0;
    // //     point.b = 0;
    // // }

    // // // 發布結果
    // // sensor_msgs::PointCloud2 output;
    // // pcl::toROSMsg(*color_cloud, output);
    // // output.header = input->header;
    // // pub.publish(output);


    // // 發布法線
    // // 可視化 Marker
    // visualization_msgs::MarkerArray marker_array;
    // visualization_msgs::Marker marker_template;
    // marker_template.header.frame_id = input->header.frame_id;
    // marker_template.type = visualization_msgs::Marker::ARROW;
    // marker_template.action = visualization_msgs::Marker::ADD;
    // marker_template.scale.x = 0.01;
    // marker_template.scale.y = 0.002;
    // marker_template.scale.z = 0.002;
    // marker_template.color.r = 0.0;
    // marker_template.color.g = 1.0;
    // marker_template.color.b = 0.0;
    // marker_template.color.a = 1.0;
    // marker_template.pose.orientation.x = 0.0;
    // marker_template.pose.orientation.y = 0.0;
    // marker_template.pose.orientation.z = 0.0;
    // marker_template.pose.orientation.w = 1.0;  // 這是必要的！不能為 0
    // marker_template.lifetime = ros::Duration(0.2);



    // // 空間分格子平均
    // float grid_size = 0.10f;
    // std::unordered_map<std::tuple<int, int, int>, pcl::PointNormal, boost::hash<std::tuple<int, int, int>>> grid_map;
    // for (size_t i = 0; i < cloud->size(); ++i) {
    //     const auto& pt = cloud->points[i];
    //     const auto& nm = normals->points[i];
    //     if (!pcl::isFinite(pt) || !pcl::isFinite(nm))
    //         continue;
    //     int gx = static_cast<int>(std::floor(pt.x / grid_size));
    //     int gy = static_cast<int>(std::floor(pt.y / grid_size));
    //     int gz = static_cast<int>(std::floor(pt.z / grid_size));
    //     auto key = std::make_tuple(gx, gy, gz);
    //     if (grid_map.find(key) == grid_map.end()) {
    //         pcl::PointNormal ptn;
    //         ptn.x = pt.x; ptn.y = pt.y; ptn.z = pt.z;
    //         ptn.normal_x = nm.normal_x; ptn.normal_y = nm.normal_y; ptn.normal_z = nm.normal_z;
    //         grid_map[key] = ptn;
    //     }
    // }

    // int id = 0;
    // for (const auto& kv : grid_map) {
    //     const auto& pt = kv.second;

    //     visualization_msgs::Marker arrow = marker_template;
    //     arrow.id = id++;

    //     geometry_msgs::Point start, end;
    //     start.x = pt.x;
    //     start.y = pt.y;
    //     start.z = pt.z;
    //     end.x = pt.x + 0.05 * pt.normal_x;
    //     end.y = pt.y + 0.05 * pt.normal_y;
    //     end.z = pt.z + 0.05 * pt.normal_z;
    //     arrow.points.push_back(start);
    //     arrow.points.push_back(end);

    //     marker_array.markers.push_back(arrow);
    // }

    // normal_pub.publish(marker_array);
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
