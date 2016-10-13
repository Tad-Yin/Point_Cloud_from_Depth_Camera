/*
 * =====================================================================================
 *
 *       Filename:  CameraKinect.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年10月12日 08时36分04秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tad Yin (), yinkejie1992@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "CameraKinect.h"
#include <opencv2/imgproc/imgproc.hpp>
CameraKinect::CameraKinect() :
    freenect2_(0),
    dev_(0),
    listener_(0),
    reg_(0){
    freenect2_ = new libfreenect2::Freenect2();
    listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color  | libfreenect2::Frame::Depth);
}
CameraKinect::~CameraKinect(){
    if(dev_){
        dev_->stop();
        dev_->close();
    }
    if(listener_){
        delete listener_;
    }
    if(reg_){
        delete reg_;
        reg_ = 0;
    }
    if(freenect2_){
        delete freenect2_;
    }
}
bool CameraKinect::init(){
    if(dev_){
        dev_->stop();
        dev_->close();
        dev_ = 0;
    }
    if(reg_){
        delete reg_;
        reg_ = 0;
    }
    libfreenect2::PacketPipeline* pipeline;
    //use cpu pipeline
    pipeline = new libfreenect2::CpuPacketPipeline();

    dev_ = freenect2_->openDefaultDevice(pipeline);
    pipeline = 0;
    if(!dev_) {
        std::cerr << "no kinect found!\n";
        return false;
    }
    libfreenect2::Freenect2Device::Config config;
    //config for kinect2
    config.EnableBilateralFilter = true;
    config.EnableEdgeAwareFilter = true;
    config.MinDepth = 0.3f;
    config.MaxDepth = 12.0f;
    dev_->setConfiguration(config);

    dev_->setColorFrameListener(listener_);
    dev_->setIrAndDepthFrameListener(listener_);

    dev_->start();

    libfreenect2::Freenect2Device::IrCameraParams depthParams =
        dev_->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams colorParams =
        dev_->getColorCameraParams();
    reg_ = new libfreenect2::Registration(depthParams, colorParams);
    return true;
}

bool CameraKinect::capture(cv::Mat& _mat){
    if(!dev_ || !listener_) {
        std::cerr << "kinect not open!\n";
        return false;
    }
    libfreenect2::FrameMap frames;
    listener_->waitForNewFrame(frames);
    libfreenect2::Frame *rgbFrame = 0;
    //libfreenect2::Frame *depthFrame = 0;
    rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
    //depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
    cv::Mat rgbMatC4((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
    cv::namedWindow("original");
    cv::imshow("original", rgbMatC4);
    cv::cvtColor(rgbMatC4, _mat, CV_BGRA2BGR);
    return true;
}
