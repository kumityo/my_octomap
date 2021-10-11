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

void Make3DPoints(const cv::Mat& depth_img, const cv::Mat& rgb_img, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    const double depth_min = 0.02;
    const double depth_max = 4.5;
    Eigen::Vector3d point_xyz;
    uint8_t R,G,B;
    uint32_t RGB;
    cloud.width = depth_img.cols;
    cloud.height = depth_img.rows;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    for (int i=0; i<depth_img.cols; i++)
    {
        for (int j=0; j<depth_img.rows; j++)
        {
            double depth = depth_img.at<float> ( j,i );
            cv::Vec3b bgr = rgb_img.at<cv::Vec3b>(cv::Point(j,i));
            std::uint8_t r, g, b;
            r = std::uint8_t(bgr[2]);
            g = std::uint8_t(bgr[1]);
            b = std::uint8_t(bgr[0]);
            std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            if (depth > depth_min && depth < depth_max)
            {
                point_xyz[0] = (i - cx) * depth / fx;
                point_xyz[1] = (j - cy) * depth / fy;
                point_xyz[2] = depth;
                // std::cout << point_xyz[2] << std::endl;
                // cloud.points[j * depth_img.cols + i] = pcl::PointXYZRGB(point_xyz[0],point_xyz[1],point_xyz[2]);
                // cloud.rgb[j * depth_img.cols + i] = pcl::PointXYZRGB(rgb);
                pcl::PointXYZRGB &point = cloud.points[j * depth_img.cols + i];
                point.x = point_xyz[0];
                point.y = point_xyz[1];
                point.z = point_xyz[2];
                point.r = int(bgr[2]);
                point.g = int(bgr[1]);
                point.b = int(bgr[0]);

            }      
        }
    }
}

void Visualize_point(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    viewer.showCloud(cloud_ptr);

    while (!viewer.wasStopped())
    {
    }
}

void dump_pcl_file(pcl::PointCloud<pcl::PointXYZRGB>& cloud, const std::string& out_point_path)
{
    pcl::io::savePCDFileASCII(out_point_path,cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to " << out_point_path << "." << std::endl;
}

int main(int argc, char* argv[])
{
    const std::string depth_img_path(argv[1]);
    const std::string rgb_img_path(argv[2]);
    const std::string pcd_file_path = "RGB_points_from_depth.pcd";

    std::cout << "load depth image... " << std::endl;
    std::cout << "depth_img_path: " << depth_img_path << std::endl;
    const double depthmap_factor = 5000.0;
    cv::Mat depth_img = imread(depth_img_path, cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_32F, 1.0 / depthmap_factor);
    std::cout << "done." << std::endl;

    std::cout << "load rgb image... " << std::endl;
    std::cout << "rgb_img_path: " << rgb_img_path << std::endl;
    cv::Mat rgb_img = imread(rgb_img_path, cv::IMREAD_UNCHANGED);
    std::cout << "done." << std::endl;

    // pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud_color;

    std::cout << "create RGB point cloud from depth img... ";
    Make3DPoints(depth_img, rgb_img, point_cloud_color);
    std::cout << "done." << std::endl;

    std::cout << "save pcl file..." << std::endl;
    dump_pcl_file(point_cloud_color, pcd_file_path);
    std::cout << "done." << std::endl;

    std::cout << "visualize point...";
    Visualize_point(point_cloud_color);
    std::cout << "done." << std::endl;

    return 0;
}