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

std::vector<int> convert_temp_color(const cv::Vec3b bgr)
{
    int r_origin = int(bgr[2]);
    std::vector<int> rgb_temp(3);
    if (r_origin <= 30)
    {
        rgb_temp = {128,0,128};
    }
    else if (r_origin > 30 && r_origin <= 60)
    {
        rgb_temp = {0,0,255};
    }
    else if (r_origin > 60 && r_origin <= 90)
    {
        rgb_temp = {0,255,255};
    }
    else if (r_origin > 90 && r_origin <= 120)
    {
        rgb_temp = {141,219,191};
    }
    else if (r_origin > 120 && r_origin <= 150)
    {
        rgb_temp = {0,128,0};
    }
    else if (r_origin > 150 && r_origin <= 180)
    {
        rgb_temp = {200,200,0};
    }
    else if (r_origin > 180 && r_origin <= 210)
    {
        rgb_temp = {255,255,0};
    }
    else if (r_origin > 210 && r_origin <= 240)
    {
        rgb_temp = {255,165,0};
    }
    else
    {
        rgb_temp = {255,0,0};
    }
    
    return rgb_temp;
}

void make_and_insert_cloud_to_octomap(const cv::Mat& depth_img, const cv::Mat& rgb_img, octomap::ColorOcTree* color_octomap_tree)
{
    const double depth_min = 0.02;
    const double depth_max = 4.5;
    std::vector<int> rgb_temp;
    Eigen::Vector3d point_xyz;
    octomap::point3d endpoint;
    for (int i=0; i<depth_img.cols; i++)
    {
        for (int j=0; j<depth_img.rows; j++)
        {
            double depth = depth_img.at<float> (j,i);
            cv::Vec3b bgr = rgb_img.at<cv::Vec3b>(cv::Point(j,i));
            rgb_temp = convert_temp_color(bgr);
            if (depth > depth_min && depth < depth_max)
            {
                point_xyz[0] = (i - cx) * depth / fx;
                point_xyz[1] = (j - cy) * depth / fy;
                point_xyz[2] = depth;
                // octomap::Pointcloud point_cloud;
                // point_cloud.push_back((float)point_xyz[0],(float)point_xyz[1],(float)point_xyz[2]);
                // octomap::point3d sensor_origin{0,0,0};
                // color_octomap_tree->insertPointCloud(point_cloud, sensor_origin);
                octomap::point3d endpoint ((float)point_xyz[0],(float)point_xyz[1],(float)point_xyz[2]);
                // octomap::ColorOcTreeNode* n = color_octomap_tree->insertPointCloud(point_cloud,sensor_origin);
                octomap::ColorOcTreeNode* n = color_octomap_tree->updateNode(endpoint, true);
                n->setColor(rgb_temp[0],rgb_temp[1],rgb_temp[2]);
                // color_octomap_tree->setColor(rgb_temp[0],rgb_temp[1],rgb_temp[2])
            }
        }
    }
}

cv::Mat load_depth_img(const std::string depth_img_path)
{
    std::cout << "depth_img_path: " << depth_img_path << std::endl;
    const double depth_map_factor = 5000.0;
    cv::Mat depth_img = imread(depth_img_path, cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_32F, 1.0/depth_map_factor);
    return depth_img;
}

cv::Mat load_rgb_img(const std::string rgb_img_path)
{
    std::cout << "rgb_img_path: " << rgb_img_path << std::endl;
    cv::Mat rgb_img = imread(rgb_img_path, cv::IMREAD_UNCHANGED);
    cv::rotate(rgb_img, rgb_img, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::flip(rgb_img, rgb_img, 0);
    return rgb_img;
}


int main(int argc, char* argv[])
{
    // const std::string depth_img_path(argv[1]);
    // const std::string rgb_img_path(argv[2]);
    const std::string depth_img_path_1 = "/home/takafumi/octomap_simple_example/datasets/depth/1305031104.863256.png";
    const std::string depth_img_path_2 = "/home/takafumi/octomap_simple_example/datasets/depth/1305031105.298501.png";
    // const std::string depth_img_path_3 = "/home/takafumi/octomap_simple_example/datasets/depth/1305031105.298501.png";
    const std::string rgb_img_path_1 = "/home/takafumi/octomap_simple_example/datasets/rgb/1305031104.875350.png";
    const std::string rgb_img_path_2 = "/home/takafumi/octomap_simple_example/datasets/rgb/1305031105.311290.png";
    // const std::string rgb_img_path_3 = "/home/takafumi/octomap_simple_example/datasets/rgb/1305031105.311290.png";
    
    const std::string color_octomap_file_path = "temp_octomap_from_multiple_depth.ot";

    std::cout << "Load depth image ..." << std::endl;
    cv::Mat depth_img_1 = load_depth_img(depth_img_path_1);
    cv::Mat depth_img_2 = load_depth_img(depth_img_path_2);
    // cv::Mat depth_img_3 = load_depth_img(depth_img_path_3);
    std::cout << "done." << std::endl;

    std::cout << "Load RGB image ..." << std::endl;
    cv::Mat rgb_img_1 = load_rgb_img(rgb_img_path_1);
    cv::Mat rgb_img_2 = load_rgb_img(rgb_img_path_2);
    // cv::Mat rgb_img_3 = load_rgb_img(rgb_img_path_3);
    std::cout << "done." << std::endl;

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
    make_and_insert_cloud_to_octomap(depth_img_1, rgb_img_1, color_octomap_tree);
    make_and_insert_cloud_to_octomap(depth_img_1, rgb_img_1, color_octomap_tree);
    color_octomap_tree->updateInnerOccupancy();
    make_and_insert_cloud_to_octomap(depth_img_2, rgb_img_2, color_octomap_tree);
    make_and_insert_cloud_to_octomap(depth_img_2, rgb_img_2, color_octomap_tree);
    color_octomap_tree->updateInnerOccupancy();
    // make_and_insert_cloud_to_octomap(depth_img_3, rgb_img_3, color_octomap_tree);
    // make_and_insert_cloud_to_octomap(depth_img_3, rgb_img_3, color_octomap_tree);
    // color_octomap_tree->updateInnerOccupancy();
    std::cout << "done." << std::endl;

    std::cout << "Save octomap ..." << std::endl;
    std::cout << "Octomap_path: " << color_octomap_file_path << std::endl;
    // dump_color_octomap(color_octomap_tree, color_octomap_file_path);
    color_octomap_tree->write(color_octomap_file_path);
    std::cout << "done." << std::endl;

    std::cout << "Finish." << std::endl;

    delete color_octomap_tree;

    return 0;
}
