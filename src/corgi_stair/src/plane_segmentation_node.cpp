#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 載入點雲（你可以改成從 /zedxm topic 轉存的 .pcd）
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("input.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read input.pcd\n");
        return -1;
    }

    // 計算 normal
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // 建立分割器
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(1000);
    mps.setAngularThreshold(0.017453 * 2.0); // ~2 degrees
    mps.setDistanceThreshold(0.02); // 2cm
    mps.setInputNormals(normals);
    mps.setInputCloud(cloud);

    // 執行分割
    std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>> regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> boundaries;

    mps.segment(regions);
    // refinePlanes() 只能在 PCL >= 1.11 才支援用 indices/boundaries 輸出
    // PCL 1.10 僅支援這種簡單用法

    // 顯示分割結果
    pcl::visualization::PCLVisualizer viewer("Multi-Plane Segmentation");
    viewer.addPointCloud(cloud, "cloud");

    for (size_t i = 0; i < regions.size(); ++i) {
        std::stringstream name;
        name << "plane_" << i;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        *plane = *regions[i].getContour();
        viewer.addPolygon<pcl::PointXYZRGB>(plane, 1.0, 0.0, 0.0, name.str());
    }

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}
