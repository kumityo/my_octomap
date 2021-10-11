#include <octomap/octomap.h>
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

void Make3DPoints_for_octomap(const cv::Mat& img, octomap::Pointcloud& cloud)
{
    const double depth_min = 0.02;
    const double depth_max = 4.5;
    Eigen::Vector3d point_xyz;
    for (int i=0; i<img.cols; i++)
    {
        for (int j=0; j<img.rows; j++)
        {
            double depth = img.at<float> (j,i);
            if (depth > depth_min && depth < depth_max)
            {
                point_xyz[0] = (i - cx) * depth / fx;
                point_xyz[1] = (j - cy) * depth / fy;
                point_xyz[2] = depth;

                cloud.push_back(point_xyz[0],point_xyz[1],point_xyz[2]);
            }
        }
    }
}

void insert_cloud_to_octomap(octomap::OcTree* const octomap_tree, const octomap::Pointcloud& point_cloud)
{
    octomap::point3d sensor_origin{0,0,0};
    octomap_tree->insertPointCloud(point_cloud,sensor_origin);
}

bool dump_octomap(octomap::OcTree* const octomap_tree, const std::string& out_octomap_path)
{
    bool is_success = octomap_tree->writeBinary(out_octomap_path);
    return is_success;
}

int main(int argc, char* argv[])
{
    const std::string depth_img_path(argv[1]);
    const std::string octomap_file_path = "octomap_from_depth.bt";

    std::cout << "Load depth image..." << std::endl;
    std::cout << "depth_img_path: " << depth_img_path << std::endl;
    const double depthmap_factor = 5000.0;
    cv::Mat depth_img = imread(depth_img_path, cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_32F, 1.0 / depthmap_factor);
    std::cout << "done." << std::endl;

    std::cout << "create point cloud (for octomap) from depth img... ";
    octomap::Pointcloud point_cloud;
    Make3DPoints_for_octomap(depth_img, point_cloud);
    std::cout << "done." << std::endl;

    std::cout << "prepare for octomap... ";
    const double voxel_size = 0.01;
    const double occupancy_thres = 0.61;
    const double prob_hit = 0.6;
    const double prob_miss = 0.45;
    octomap::OcTree* octomap_tree = new octomap::OcTree ( voxel_size );
    octomap_tree->setOccupancyThres ( occupancy_thres );
    octomap_tree->setProbHit ( prob_hit );
    octomap_tree->setProbMiss ( prob_miss );
    std::cout << "done." << std::endl;

    std::cout << "insert point cloud into octomap... ";
    insert_cloud_to_octomap(octomap_tree, point_cloud);
    insert_cloud_to_octomap(octomap_tree, point_cloud);
    octomap_tree->updateInnerOccupancy();
    std::cout << "done." << std::endl;

    std::cout << "save octomap... ";
    dump_octomap(octomap_tree, octomap_file_path);
    std::cout << "done." << std::endl;

    std::cout << "finish." << std::endl;

    delete octomap_tree;

    return 0;
}
