#include "recover_points_from_depth.hpp"

int main(int argc, char* argv[])
{
    const std::string depth_img_path(argv[1]);
    const std::string out_octomap_path = "octomap.bt";
    const std::string out_point_path = "points.txt";
    const std::string out_point_pcl_path = "points.pcd";


    ////////////////////////////////////////////////////////////////////////////////////
    // depth画像を読み，変換する

    std::cout << "depth_img_path: " << depth_img_path << std::endl;

    // 北島取得のデータセットは，実際のdepth[m]を5000倍した値が16bit整数で表現されているので，
    // 浮動小数点型で実際のdepthに変換する
    // TUM datasetも5000倍されているのでTUMdatasetの場合はこのままでいい
    const double depthmap_factor = 5000.0;
    cv::Mat depth_img = imread(depth_img_path, cv::IMREAD_UNCHANGED);
    depth_img.convertTo(depth_img, CV_32F, 1.0 / depthmap_factor);


    ////////////////////////////////////////////////////////////////////////////////////
    // 画像から3次元点を作成する

    std::cout << "create point cloud from depth img... ";

    octomap::Pointcloud point_cloud;
    read_cloud_from_depth(depth_img, point_cloud);
    std::cout << "done.\n" << std::endl;



    ////////////////////////////////////////////////////////////////////////////////////
    // Octomapの準備

    std::cout << "prepare for octomap... ";

    // しきい値はdre_slamのものによる
    // from dre_slam source code: https://github.com/ydsf16/dre_slam
    // from octomap documentation: https://octomap.github.io/octomap/doc/classoctomap_1_1OcTree.html#a03a4b455b5185ad715cc1a50aae42ed8
    const double voxel_size = 0.01;
    const double occupancy_thres = 0.61;
    const double prob_hit = 0.6;
    const double prob_miss = 0.45;
    octomap::OcTree* octomap_tree = new octomap::OcTree ( voxel_size );
    octomap_tree->setOccupancyThres ( occupancy_thres );      // 占有状態とみなす確率のしきい値
    octomap_tree->setProbHit ( prob_hit );                    // 点が観測されたときの占有確率
    octomap_tree->setProbMiss ( prob_miss);                   // 点が観測されなかったときの自由確率

    std::cout << "done.\n" << std::endl;


    ////////////////////////////////////////////////////////////////////////////////////
    // octomapへの点群の挿入

    std::cout << "insert point cloud into octomap... ";

    // 1回目：点群をoctomapに挿入する
    insert_cloud_to_octomap(octomap_tree, point_cloud);

    // 2回目：点群をoctomapに挿入する
    // 上記のしきい値の設定例では、1度観測されるとそのボクセルの占有確率は0.6となり、
    // 占有状態のしきい値0.61を超えない。
    // したがって、複数回にわたり観測されないと占有状態にはならない
    insert_cloud_to_octomap(octomap_tree, point_cloud);

    // octomapの占有確率の更新
    octomap_tree->updateInnerOccupancy();

    std::cout << "done.\n" << std::endl;


    ////////////////////////////////////////////////////////////////////////////////////
    // 点群とoctomapをファイルに出力する

    std::cout << "save octomap and point into the file... ";

    dump_points(point_cloud, out_point_path);
    dump_octomap(octomap_tree, out_octomap_path);
    dump_points_pcl(point_cloud, out_point_pcl_path);

    std::cout << "done.\n" << std::endl;

    /////////////////////////////////
    // 点群をpcdファイルに保存する
    // std::cout << "save point cloud into the pcd file... ";
    // pcl::io::savePCDFileASCII("point_cloud_pcd.pcd",point_cloud_pcl);
    // std::cout << "done.\n" << std::endl;
    
    ////////////////////////////////////////////////////////////////////////////////////

    std::cout << "finish" << std::endl;
    std::cout << "points: " << out_point_path << std::endl;
    std::cout << "points(pcl): " << out_point_pcl_path << std::endl;
    std::cout << "octomap: " << out_octomap_path << std::endl;

    // newしたポインタは必ずdeleteする
    delete octomap_tree;

    return 0;
}