/*
 * =====================================================================================
 *
 *       Filename:  Camera.h
 *
 *    Description:  Interface for Camera
 *
 *        Version:  1.0
 *        Created:  2016年08月22日 12时23分51秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  yinkejie1992@gmail.com 
 *   Organization:
 *
 * =====================================================================================
 */
#ifndef CAMERA_H_JULLETXD
#define CAMERA_H_JULLETXD

#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
class Camera
{
public:
	virtual ~Camera(){};

    template<typename T, typename... Ts>
    static auto makeCamera(Ts&&... params){
        std::unique_ptr<Camera> cInv(nullptr);
        cInv.reset(new T(std::forward<Ts>(params)...));
        return cInv;
    }
	virtual bool init() = 0;
	virtual bool isCalibrated() const = 0;
	virtual int getSerial() const = 0;

	//getters
	//const Transform& getLocalTransform() const {return _localTransform;}

	//setters
	//void setLocalTransform(const Transform& localTransform) {_localTransform = localTransform;}
	virtual bool capture(cv::Mat&) = 0;
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&) = 0;
    virtual bool capture(pcl::PointCloud<pcl::PointXYZRGB>&, cv::Mat&) = 0;

protected:
	//Camera(float imageRate = 0, const Transform& localTransform = Transform::getIdentity());
    Camera():_seq(0){}
	int getNextSeqID() {return ++_seq;}

private:
	//Transform _localTransform;
    int _seq;
};

#endif /* end of include guard: CAMERA_H_JULLETXD */
