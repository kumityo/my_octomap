#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


int main(int argc, char** argv)
{
    const std::string pcl_file_path = "points_from_depth.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));

    pcl::io::loadPCDFile (pcl_file_path,*cloud_ptr);
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    viewer.showCloud(cloud_ptr);

    while (!viewer.wasStopped())
    {

    }

    return 0;

}