/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/23/2016 10:38:42 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tad Yin (), yinkejie1992@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */


#include "CameraKinect.h"
#include <unistd.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

int user_data = 0;

int main(int argc, char *argv[])
{

    //std::shared_ptr<Camera> camera = Camera::makeCamera<CameraBumblebee>();
    //camera->init();
    //cv::Mat ma;
    //pcl::PointCloud<pcl::PointXYZRGB> p;
    //while(1){
    //camera->capture(p, ma);
    //cv::namedWindow("test");
    //cv::imshow("test", ma);
    ////pcl::io::savePCDFileASCII("test.pcd", p);

    //cv::waitKey(0);
    //sleep(2);
    //}
    std::shared_ptr<Camera> camera = Camera::makeCamera<CameraKinect>();
    camera->init();
    cv::Mat ma;
    //pcl::visualization::CloudViewer viewer("test");
    sleep(1);
    camera->capture(ma);
    cv::namedWindow("capture");
    cv::imshow("capture", ma);
    cv::waitKey(0);
    //delete(camera);
    return 0;
}
