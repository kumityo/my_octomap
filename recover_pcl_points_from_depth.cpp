#include <string>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>

// Camera Parameter
// TUM dataset freiburg1_xyz
const double fx =  517.306408;
const double fy =  516.469215;
const double cx =  318.643040;
const double cy =  255.313989;

void Make3DPoints(const cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    const double depth_min = 0.02;
    const double depth_max = 4.5;
    Eigen::Vector3d point_xyz;
    cloud.width = img.cols;
    cloud.height = img.rows;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    for (int i=0; i<img.cols; i++)
    {
        for (int j=0; j<img.rows; j++)
        {
            double depth = img.at<float> ( j,i );
            if (depth > depth_min && depth < depth_max)
            {
                point_xyz[0] = (i - cx) * depth / fx;
                point_xyz[1] = (j - cy) * depth / fy;
                point_xyz[2] = depth;
                // std::cout << point_xyz[2] << std::endl;
                cloud.points[j * img.cols + i] = pcl::PointXYZ(point_xyz[0],point_xyz[1],point_xyz[2]);
            }      
        }
    }
}

void Visualize_point(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    viewer.showCloud(cloud_ptr);

    while (!viewer.wasStopped())
    {
    }
}

void dump_pcl_file(pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& out_point_path)
{
    pcl::io::savePCDFileASCII(out_point_path,cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to " << out_point_path << "." << std::endl;
}

int main(int argc, char* argv[])
{
    const std::string depth_img_path(argv[1]);
    const std::string pcd_file_path = "points_from_depth.pcd";

    std::cout << "Load depth image..." << std::endl;
    std::cout << "depth_img_path: " << depth_img_path << std::endl;
    const double depthmap_factor = 5000.0;
    cv::Mat depth_img = imread(depth_img_path, cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_32F, 1.0 / depthmap_factor);
    std::cout << "done." << std::endl;

    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    std::cout << "create point cloud from depth img... ";
    Make3DPoints(depth_img, point_cloud);
    std::cout << "done." << std::endl;

    std::cout << "save pcl file..." << std::endl;
    dump_pcl_file(point_cloud, pcd_file_path);
    std::cout << "done." << std::endl;

    std::cout << "visualize point...";
    Visualize_point(point_cloud);
    std::cout << "done." << std::endl;

    return 0;
}