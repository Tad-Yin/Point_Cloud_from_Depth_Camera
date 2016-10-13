/*
 * =====================================================================================
 *
 *       Filename:  CameraBumblebee.h
 *
 *    Description:  interface Camera for Bumblebee
 *
 *        Version:  1.0
 *        Created:  2016年08月22日 14时04分44秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  yinkejie1992@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef CAMERABUMBLEBEE_H_JSYOY40M
#define CAMERABUMBLEBEE_H_JSYOY40M

#include "Camera.h"
#include <triclops.h>
#include <fc2triclops.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <opencv2/imgproc/imgproc.hpp>

class CameraBumblebee : public Camera{
public:
    //static bool available();
    //CameraBumblebee(float imageRate = 0.0f,
    //        const Transform& localTransform = Transform::getIdentity());

    CameraBumblebee();
    virtual ~CameraBumblebee();
    virtual bool init();
    virtual bool isCalibrated() const;
    virtual int getSerial() const;



    virtual bool capture(cv::Mat&);
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&);
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&, cv::Mat&);

private:
    struct ImageContainer
    {
        FlyCapture2::Image mono[2];
        FlyCapture2::Image unprocessed[2];
        FlyCapture2::Image bgru[2];
    };

    bool grabImage(cv::Mat&);
    void generate3DPoints(
            pcl::PointCloud<pcl::PointXYZRGB>&,
            cv::Mat&);

    FlyCapture2::Camera* _camera;
    void* _triclopsCtx; //triclopsContext;
    void UERROR(const std::string& error){
        std::cerr << error << "\n";
    }

    int WIDTH = 640;
    int HEIGHT = 480;
    static const int RIGHT = 0;
    static const int LEFT = 1;
};

#endif /* end of include guard: CAMERABUMBLEBEE_H_JSYOY40M */
