/*
 * =====================================================================================
 *
 *       Filename:  CameraOpenNI.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年09月07日 09时47分40秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:   (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef CAMERAOPENNI_H_JPPYVZHU
#define CAMERAOPENNI_H_JPPYVZHU



#include "Camera.h"
#include "UMutex.h"
#include "USemaphore.h"
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_grabber.h>

#include <boost/signals2/connection.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
class CameraOpenNI : public Camera{
public:
    CameraOpenNI();
    virtual ~CameraOpenNI();

    virtual bool init();
    virtual bool isCalibrated() const{
        return true;
    }
    virtual int getSerial() const{
        return -1;
    };

    virtual bool capture(cv::Mat&);
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&);
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&, cv::Mat&);

    void cloud_cb(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);
    void image_cb(const boost::shared_ptr<openni_wrapper::Image>&);
private:
    pcl::Grabber* _interface;
    boost::signals2::connection _connection;
    //cv::Mat _depth;
    cv::Mat _rgb;
    pcl::PointCloud<pcl::PointXYZRGB> _point;
    float _depthConstant;

    UMutex _dataMutex;
    USemaphore _matReady;
    USemaphore _cloudReady;
};


#endif /* end of include guard: CAMERAOPENNI_H_JPPYVZHU */
