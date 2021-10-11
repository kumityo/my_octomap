#include "recover_points_from_depth.hpp"

#include <fstream>

// using namespace cv;
// using namespace std;

// camera parameter
// Camera.cols: 960
// Camera.rows: 540

// const double fx =  530.2126;
// const double fy =  530.1829;
// const double cx =  475.1043;
// const double cy =  265.1346;

//TUM dataset freiburg1_xyz
const double fx =  517.306408;
const double fy =  516.469215;
const double cx =  318.643040;
const double cy =  255.313989;

// Camera.k1: 0.060082494282151171
// Camera.k2: -0.072665526039814521
// Camera.p1: 0.00096326179384144969
// Camera.p2: -0.0014841003265764945
// Camera.k3: 0.026152189230298935

Eigen::Vector4d recover_points(const double depth, const double ux, const double uy )
{
    Eigen::Vector4d xyz;
    xyz[0] = ( ux - cx ) * depth/fx;
    xyz[1] = ( uy - cy ) * depth/fy;
    xyz[2] = depth;
    xyz[3] = 1.0;
    return xyz;
}

void read_cloud_from_depth(const cv::Mat& depth_img, octomap::Pointcloud& cloud)
{
    //cout << "cols: " << depth_img.cols << " rows: " << depth_img.rows << endl;

    const double cam_dmin = 0.02;
    const double cam_dmax = 4.5;

    for(int i=0; i<depth_img.cols; i++)
    {
        for(int j=0; j<depth_img.rows; j++)
        {
            double depth = depth_img.at<float> ( j,i );

            // 有効なdepthが存在するときのみ有効な点群として扱う
            if ( depth > cam_dmin && depth < cam_dmax ) {
                const Eigen::Vector4d cp = recover_points(depth, i, j);  

                cloud.push_back(cp[0], cp[1], cp[2]);
            }
        }
    }

}


void insert_cloud_to_octomap(octomap::OcTree* const octomap_tree, const octomap::Pointcloud& point_cloud)
{
    // ここでは，原点から点群を得たものとしてoctomapに挿入する
    octomap::point3d sensor_origin{0,0,0};
    octomap_tree->insertPointCloud(point_cloud, sensor_origin);

}

bool dump_points(const octomap::Pointcloud& point_cloud, const std::string& out_point_path)
{
    std::ofstream f(out_point_path.c_str());

    if(!f)
    {
        std::cerr << "[dump_points()]: failed to open file\n";
        return false;
    }

    // 点群を取得し，テキストファイルに書き込んでいく
    for(int i=0; i<point_cloud.size(); i++)
    {
        const double x = point_cloud[i].x();
        const double y = point_cloud[i].y();
        const double z = point_cloud[i].z();

        f << x << "," << y << "," << z << "\n";
    }

    f.close();

    return true;
}

bool dump_points_pcl(const octomap::Pointcloud& point_cloud, const std::string& out_point_pcl_path)
{
    //std::ofstream f(out_point_pcl_path.c_str());
    std::ofstream writing_file;
    writing_file.open(out_point_pcl_path, std::ios::out);

    // if(!f)
    // {
    //     std::cerr << "[dump_points()]: failed to open file\n";
    //     return false;
    // }
    
    std::string writing_text1 = "# .PCD v0.7 - Point Cloud Data file format";
    std::string writing_text2 = "VERSION 0.7";
    std::string writing_text3 = "FIELDS x y z";
    std::string writing_text4 = "SIZE 4 4 4";
    std::string writing_text5 = "TYPE F F F";
    std::string writing_text6 = "COUNT 1 1 1";
    std::string writing_text7 = "WIDTH ";
    std::string writing_text8 = "HEIGHT 1";
    std::string writing_text9 = "VIEWPOINT 0 0 0 1 0 0 0";
    std::string writing_text10 = "POINTS ";
    std::string writing_text11 = "DATA ascii";

    writing_file << writing_text1 << std::endl;
    writing_file << writing_text2 << std::endl;
    writing_file << writing_text3 << std::endl;
    writing_file << writing_text4 << std::endl;
    writing_file << writing_text5 << std::endl;
    writing_file << writing_text6 << std::endl;
    writing_file << writing_text7 << point_cloud.size() << std::endl;
    writing_file << writing_text8 << std::endl;
    writing_file << writing_text9 << std::endl;
    writing_file << writing_text10 << point_cloud.size() << std::endl;
    writing_file << writing_text11 << std::endl;



    // 点群を取得し，テキストファイルに書き込んでいく
    for(int i=0; i<point_cloud.size(); i++)
    {
        const double x = point_cloud[i].x();
        const double y = point_cloud[i].y();
        const double z = point_cloud[i].z();

        writing_file << x << " " << y << " " << z << "\n";
    }

    writing_file.close();

    return true;
}

bool dump_octomap(octomap::OcTree* const octomap_tree, const std::string& out_octomap_path)
{
    // バイナリファイル形式で保存（開いても人間には読めない）
    bool is_success = octomap_tree->writeBinary(out_octomap_path);

    return is_success;
}
