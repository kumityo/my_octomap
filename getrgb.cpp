// #include <stdafx.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    const std::string rgb_img_path(argv[1]);
    cv::Mat testImage = cv::imread(rgb_img_path, 1);
    cv::Vec3b bgr = testImage.at<cv::Vec3b>(cv::Point(20,20));
    std::cout << int(bgr[0]) << "," << int(bgr[1]) << "," << int(bgr[2]) << std::endl;

    return 0;
}