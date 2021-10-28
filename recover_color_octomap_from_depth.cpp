#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <vector>
#include <Eigen/Core>

// Camera Parameter
// TUM dataset freiburg1_xyz
const double fx =  517.306408;
const double fy =  516.469215;
const double cx =  318.643040;
const double cy =  255.313989;


void make_and_insert_cloud_to_octomap(const cv::Mat& depth_img, const cv::Mat& rgb_img, octomap::ColorOcTree* color_octomap_tree)
{
    const double depth_min = 0.02;
    const double depth_max = 4.5;
    Eigen::Vector3d point_xyz;
    octomap::point3d endpoint;
    for (int i=0; i<depth_img.cols; i++)
    {
        for (int j=0; j<depth_img.rows; j++)
        {
            double depth = depth_img.at<float> (j,i);
            cv::Vec3b bgr = rgb_img.at<cv::Vec3b>(cv::Point(j,i));
            if (depth > depth_min && depth < depth_max)
            {
                point_xyz[0] = (i - cx) * depth / fx;
                point_xyz[1] = (j - cy) * depth / fy;
                point_xyz[2] = depth;
                octomap::point3d endpoint ((float)point_xyz[0],(float)point_xyz[1],(float)point_xyz[2]);
                octomap::ColorOcTreeNode* n = color_octomap_tree->updateNode(endpoint, true);
                n->setColor(int(bgr[2]),int(bgr[1]),int(bgr[0]));
                // cloud.push_back(point_xyz[0], point_xyz[1], point_xyz[2]);

            }
        }
    }
}


// void insert_cloud_to_color_octomap(octomap::ColorOcTree* const color_octomap_tree, const octomap::Pointcloud& point_cloud)
// {
//     octomap::point3d sensor_origin{0,0,0};
//     color_octomap_tree->insertPointCloud(point_cloud,sensor_origin);
// }


int main(int argc, char* argv[])
{
    const std::string depth_img_path(argv[1]);
    const std::string rgb_img_path(argv[2]);
    const std::string color_octomap_file_path = "color_octomap_from_depth.ot";

    std::cout << "Load depth image ..." << std::endl;
    std::cout << "depth_img_path: " << depth_img_path << std::endl;
    const double depth_map_factor = 5000.0;
    cv::Mat depth_img = imread(depth_img_path, cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_32F, 1.0/depth_map_factor);
    std::cout << "done." << std::endl;

    std::cout << "Load RGB image ..." << std::endl;
    std::cout << "rgb_img_path: " << rgb_img_path << std::endl;
    cv::Mat rgb_img = imread(rgb_img_path, cv::IMREAD_UNCHANGED);
    // rgb_img.convertTo(rgb_img, CV_32F, 1.0);
    cv::rotate(rgb_img, rgb_img, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::flip(rgb_img, rgb_img, 0);
    std::cout << "done." << std::endl;

    // std::cout << "Create point cloud (for octomap) from depth image ..." << std::endl;
    // octomap::Pointcloud point_cloud;
    // Make3DPoints_for_octomap(depth_img, point_cloud);
    // make_and_insert_cloud_to_octomap(depth_img, color_octomap_tree);
    // std::cout << "done." << std::endl;

    std::cout << "Prepare for octomap ..." << std::endl;
    const double voxel_size = 0.01;
    const double occupancy_thres = 0.61;
    const double prob_hit = 0.6;
    const double prob_miss = 0.45;
    octomap::ColorOcTree* color_octomap_tree = new octomap::ColorOcTree (voxel_size);
    color_octomap_tree->setOccupancyThres (occupancy_thres);
    color_octomap_tree->setProbHit (prob_hit);
    color_octomap_tree->setProbMiss (prob_miss);
    std::cout << "done." << std::endl;

    std::cout << "Insert point cloud into octomap ..." << std::endl;
    make_and_insert_cloud_to_octomap(depth_img, rgb_img, color_octomap_tree);
    make_and_insert_cloud_to_octomap(depth_img, rgb_img, color_octomap_tree);
    color_octomap_tree->updateInnerOccupancy();
    std::cout << "done." << std::endl;

    std::cout << "Save octomap ..." << std::endl;
    // dump_color_octomap(color_octomap_tree, color_octomap_file_path);
    color_octomap_tree->write(color_octomap_file_path);
    std::cout << "done." << std::endl;

    std::cout << "Finish." << std::endl;

    delete color_octomap_tree;

    return 0;
}
