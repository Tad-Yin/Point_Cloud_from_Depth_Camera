/*
 * =====================================================================================
 *
 *       Filename:  CameraKinect.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年10月11日 14时04分58秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tad Yin (), yinkejie1992@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef CAMERAKINECT_H_AMJGYZN0
#define CAMERAKINECT_H_AMJGYZN0
#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include "Camera.h"
class CameraKinect : public Camera
{
public:
    CameraKinect();
    virtual ~CameraKinect();
    virtual bool init();
    virtual bool isCalibrated() const{
        return true;
    }
    virtual int getSerial() const{
        return -1;
    };
    virtual bool capture(cv::Mat&);
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&){return false;};
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&, cv::Mat&){return false;};
private:
    libfreenect2::Freenect2 * freenect2_;
    libfreenect2::Freenect2Device *dev_;
    libfreenect2::SyncMultiFrameListener * listener_;
    libfreenect2::Registration * reg_;

    template<class K, class V>
    inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
    {
        V v = defaultValue;
        typename std::map<K, V>::const_iterator i = m.find(key);
        if(i != m.end())
        {
            v = i->second;
        }
        return v;
    }
};

#endif /* end of include guard: CAMERAKINECT_H_AMJGYZN0 */
