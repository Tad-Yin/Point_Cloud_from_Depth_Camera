/*
 * =====================================================================================
 *
 *       Filename:  CameraOpenNI.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年09月07日 10时22分27秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:   (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "CameraOpenNI.h"
#include <unistd.h>
CameraOpenNI::CameraOpenNI() :
    _interface(0),
    _depthConstant(0.0f){
    }

CameraOpenNI::~CameraOpenNI(){
    if(_connection.connected()) _connection.disconnect();
    if(_interface){
        _interface->stop();
        usleep(1000);
        delete _interface;
        _interface  = 0;
    }
}

void CameraOpenNI::image_cb(
        const boost::shared_ptr<openni_wrapper::Image>& rgb)
{
    UScopeMutex s(_dataMutex);
    bool notify = _rgb.empty();

    cv::Mat rgbFrame(rgb->getHeight(), rgb->getWidth(), CV_8UC3);
    rgb->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbFrame.data);
    cvtColor(rgbFrame, _rgb, CV_RGB2BGR);

    if(notify) _matReady.release();
}
void CameraOpenNI::cloud_cb(
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
    UScopeMutex s(_dataMutex);
    bool notify = (_point.size() == 0);

    pcl::copyPointCloud(*cloud, _point);

    if(notify) _cloudReady.release();
    //_point = *cloud;
}
bool CameraOpenNI::init(){
    if(_interface){
        _interface->stop();
        usleep(1000);
        delete _interface;
        _interface  = 0;
    }
    try{
        _interface = new pcl::OpenNIGrabber();
        boost::function<void(
                const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&
                )> f = boost::bind(&CameraOpenNI::cloud_cb, this, _1);
        boost::function<void(
                const boost::shared_ptr<openni_wrapper::Image>&
                )> g = boost::bind(&CameraOpenNI::image_cb, this, _1);
        _connection = _interface->registerCallback(f);
        _connection = _interface->registerCallback(g);
        _interface->start();
    }catch(const pcl::IOException& ex){
        fprintf(stderr, "OpenNI exceptrion %s\n", ex.what());
        if(_interface){
            delete _interface;
            _interface = 0;
        }
        return false;
    }
    return true;
}

bool CameraOpenNI::capture(pcl::PointCloud<pcl::PointXYZRGB>& point){
    if(_interface && _interface->isRunning()){
        if(!_matReady.acquire(1, 2000) || !_cloudReady.acquire(1, 2000)){
            fprintf(stderr, "wait 2 seconds with no data required");
        }else{
            UScopeMutex s(_dataMutex);
            if(_point.size() != 0){
                point = std::move(_point);
            }
            _point.resize(0);
            _rgb = std::move(cv::Mat());
            return true;
        }
    }
    return false;
}

bool CameraOpenNI::capture(cv::Mat& mat){
    if(_interface && _interface->isRunning()){
        if(!_matReady.acquire(1, 2000) || !_cloudReady.acquire(1, 2000)){
            fprintf(stderr, "wait 2 seconds with no data required");
        }else{
            UScopeMutex s(_dataMutex);
            if(!_rgb.empty()){
                mat = std::move(_rgb.clone());
            }
            _rgb = std::move(cv::Mat());
            _point.resize(0);
            return true;
        }
    }
    return false;
}

bool CameraOpenNI::capture(pcl::PointCloud<pcl::PointXYZRGB>& point,
        cv::Mat& mat){
    if(_interface && _interface->isRunning()){
        if(!_matReady.acquire(1, 2000) || !_cloudReady.acquire(1, 2000)){
            fprintf(stderr, "wait 2 seconds with no data required");
        }else{
            UScopeMutex s(_dataMutex);
            if(!_rgb.empty() && _point.size() != 0){
                mat = std::move(_rgb.clone());
                point = std::move(_point);
            }
            _rgb = std::move(cv::Mat());
            _point.resize(0);
            return true;
        }
    }
    return false;
}
