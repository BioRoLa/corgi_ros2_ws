//// Plane Segmentation in Organized Point Clouds using Flood Fill ////
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;

// ROS訊息處理
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 將ROS點雲消息轉換為PCL點雲格式
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

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

    // 法線估計
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 設置搜索半徑
    ne.compute(*normals);

    // Region Growing分割
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(100); // 最小點數
    reg.setMaxClusterSize(25000); // 最大點數
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30); // 鄰居數
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // 設定平滑度閾值
    reg.setCurvatureThreshold(1.0); // 設定曲率閾值

    // 提取平面
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // 將分割結果發佈為紅色點雲
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (const auto& index : cluster.indices) {
            pcl::PointXYZRGB point = cloud->points[index];
            point.r = 255;  // 設為紅色
            point.g = 0;
            point.b = 0;
            cloud_cluster->points.push_back(point);
        }
        *output_cloud += *cloud_cluster;
    }

    // 將PCL點雲轉換為ROS消息
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header = cloud_msg->header;  // 保持原始消息頭部信息
    // 發佈紅色點雲
    pub.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_segmentation_node");
    ros::NodeHandle nh;

    // 訂閱&發佈點雲topic
    ros::Subscriber sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("plane_segmentation", 1);

    // 開始循環
    ros::spin();

    return 0;
}
