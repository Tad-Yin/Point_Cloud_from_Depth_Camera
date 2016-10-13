/*
 * =====================================================================================
 *
 *       Filename:  CameraBumblebee.cpp
 *
 *    Description:  function for CameraBumblebee interface
 *
 *        Version:  1.0
 *        Created:  08/23/2016 09:23:25 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Tad Yin (), yinkejie1992@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */
#include "CameraBumblebee.h"


CameraBumblebee::CameraBumblebee():
    _camera(0),
    _triclopsCtx(0){
        _camera = new FlyCapture2::Camera();
    }
CameraBumblebee::~CameraBumblebee(){
    // Close the camera
    _camera->StopCapture();
    _camera->Disconnect();
    // Destroy the Triclops context
    triclopsDestroyContext(_triclopsCtx);
    delete _camera;
}
//CameraBumblebee& CameraBumblebee::getInstance(){
//static CameraBumblebee _instance;
//return _instance;
//}
bool CameraBumblebee::init(){
    if(_camera)
    {
        // Close the camera
        _camera->StopCapture();
        _camera->Disconnect();
    }
    if(_triclopsCtx)
    {
        triclopsDestroyContext(_triclopsCtx);
        _triclopsCtx = 0;
    }

    // connect camera
    FlyCapture2::Error fc2Error = _camera->Connect();
    if(fc2Error != FlyCapture2::PGRERROR_OK)
    {
        UERROR("Failed to connect the camera.");
        return false;
    }

    // configure camera
    Fc2Triclops::StereoCameraMode mode = Fc2Triclops::TWO_CAMERA;
    if(Fc2Triclops::setStereoMode(*_camera, mode))
    {
        UERROR("Failed to set stereo mode.");
        return false;
    }

    // generate the Triclops context
    FlyCapture2::CameraInfo camInfo;
    if(_camera->GetCameraInfo(&camInfo) != FlyCapture2::PGRERROR_OK)
    {
        UERROR("Failed to get camera info.");
        return false;
    }

    // Get calibration from th camera
    if(Fc2Triclops::getContextFromCamera(camInfo.serialNumber, &_triclopsCtx))
    {
        UERROR("Failed to get calibration from the camera.");
        return false;
    }

    triclopsSetCameraConfiguration(_triclopsCtx, TriCfg_2CAM_HORIZONTAL);
    triclopsSetResolutionAndPrepare(_triclopsCtx, HEIGHT, WIDTH, HEIGHT, WIDTH);
    if(_camera->StartCapture() != FlyCapture2::PGRERROR_OK)
    {
        UERROR("Failed to start capture.");
        return false;
    }

    return true;
}

bool CameraBumblebee::isCalibrated() const{
    if(_triclopsCtx)
    {
        float fx, cx, cy, baseline;
        triclopsGetFocalLength(_triclopsCtx, &fx);
        triclopsGetImageCenter(_triclopsCtx, &cy, &cx);
        triclopsGetBaseline(_triclopsCtx, &baseline);
        return fx > 0.0f && cx > 0.0f && cy > 0.0f && baseline > 0.0f;
    }
    return false;
}

int CameraBumblebee::getSerial() const{
    if(_camera && _camera->IsConnected())
    {
        FlyCapture2::CameraInfo camInfo;
        if(_camera->GetCameraInfo(&camInfo) == FlyCapture2::PGRERROR_OK)
        {
            return camInfo.serialNumber;
        }
    }
    return -1;
}

bool CameraBumblebee::grabImage(
        cv::Mat& _mat){

    FlyCapture2::Image grabbedImage;
    ImageContainer imgCont;
    if(!_camera || !_triclopsCtx || !_camera->IsConnected()) {
        UERROR("camera is not opened!");
        return false;
    }
    if(_camera->RetrieveBuffer(&grabbedImage) != FlyCapture2::PGRERROR_OK){
        UERROR("can't get picture");
        return false;
    }
    FlyCapture2::Image* unprocessedImg = imgCont.unprocessed;
    FlyCapture2::Image* bgruImg = imgCont.bgru;
    FlyCapture2::Image* monoImg = imgCont.mono;
    if(Fc2Triclops::unpackUnprocessedRawOrMono16Image(
                grabbedImage,
                true /*assume little endian*/,
                unprocessedImg[RIGHT] /* right */,
                unprocessedImg[LEFT] /* left */) != Fc2Triclops::ERRORTYPE_OK){
        UERROR("Can not unpack image");
        return false;
    }

    FlyCapture2::Error fc2err;
    Fc2Triclops::ErrorType fc2terr;
    for(int i = 0; i < 2; i++){
        fc2err = unprocessedImg[i].SetColorProcessing(FlyCapture2::HQ_LINEAR);
        if(fc2err != FlyCapture2::PGRERROR_OK){
            UERROR("set color process error");
            return false;
        }
        fc2err = unprocessedImg[i].Convert(FlyCapture2::PIXEL_FORMAT_BGRU, &bgruImg[i]);
        if(fc2err != FlyCapture2::PGRERROR_OK){
            UERROR("Convert to mono fail");
            return false;
        }
        fc2err = bgruImg[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImg[i]);
        if(fc2err != FlyCapture2::PGRERROR_OK)
            return false;
    }
    FlyCapture2::Image packedColorImage;
    fc2terr = Fc2Triclops::packTwoSideBySideRgbImage(
            bgruImg[RIGHT],
            bgruImg[LEFT],
            packedColorImage);
    if(fc2terr != Fc2Triclops::ERRORTYPE_OK){
        return false;
    }
    TriclopsInput triclopsColorInput, triclopsMonoInput;
    triclopsBuildPackedTriclopsInput(
            grabbedImage.GetCols(),
            grabbedImage.GetRows(),
            packedColorImage.GetStride(),
            (unsigned long)grabbedImage.GetTimeStamp().seconds,
            (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
            packedColorImage.GetData(),
            &triclopsColorInput);

    triclopsBuildRGBTriclopsInput(
            grabbedImage.GetCols(),
            grabbedImage.GetRows(),
            grabbedImage.GetCols(),
            (unsigned long)grabbedImage.GetTimeStamp().seconds,
            (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
            monoImg[RIGHT].GetData(),
            monoImg[LEFT].GetData(),
            monoImg[LEFT].GetData(),
            &triclopsMonoInput);
    triclopsSetSubpixelInterpolation(_triclopsCtx, 1);
    // Rectify the images
    triclopsRectify(_triclopsCtx, const_cast<TriclopsInput *>(&triclopsMonoInput));
    // Do stereo processing
    triclopsStereo(_triclopsCtx);

    cv::Mat pixelsLeftBuffer( HEIGHT, WIDTH, CV_8UC4);
    //cv::Mat pixelsLeftBuffer(grabbedImage.GetRows(), grabbedImage.GetCols(), CV_8UC4);
    TriclopsPackedColorImage _colorImage;
    triclopsSetPackedColorImageBuffer(
            _triclopsCtx,
            TriCam_REFERENCE,
            (TriclopsPackedColorPixel*)pixelsLeftBuffer.data );

    triclopsRectifyPackedColorImage(
            _triclopsCtx,
            TriCam_REFERENCE,
            &triclopsColorInput,
            &_colorImage );
    cv::cvtColor(pixelsLeftBuffer, _mat, CV_RGBA2RGB);

    //triclopsRectify(_triclopsCtx, const_cast<TriclopsInput *>(&triclopsMonoInput));
    //triclopsStereo(_triclopsCtx);
    return true;
}

void CameraBumblebee::generate3DPoints(pcl::PointCloud<pcl::PointXYZRGB>& _cloud,
        cv::Mat& colorImage){
    TriclopsImage16 disparityImage16;

    // Retrieve the interpolated depth image from the context
    triclopsGetImage16(_triclopsCtx,
            TriImg16_DISPARITY,
            TriCam_REFERENCE,
            &disparityImage16);
    //triclopsSaveImage16(&disparityImage16, "test.pgm");

    int pixelinc = disparityImage16.rowinc/2;
    float x, y, z;
    _cloud.width = disparityImage16.ncols;
    _cloud.height = disparityImage16.nrows;
    _cloud.points.resize(disparityImage16.nrows * disparityImage16.ncols);
    _cloud.is_dense = false;
    for (int i = 0, k  = 0; i < disparityImage16.nrows; i++)
    {
        unsigned short* row = disparityImage16.data + i * pixelinc;
        for (int j = 0; j < disparityImage16.ncols; j++, k++)
        {
            unsigned short disparity = row[j];
            //pcl::PointXYZRGB pr;

            triclopsRCD16ToXYZ(_triclopsCtx, i, j, disparity, &x, &y, &z);
            // do not save invalid points
            _cloud.points[k].b = (int)colorImage.data[3 * k];
            _cloud.points[k].g = (int)colorImage.data[3 * k + 1];
            _cloud.points[k].r = (int)colorImage.data[3 * k + 2];

            if (disparity < 0xFF00 && z < 2.0 && z > 0.4)
            {
                // convert the 16 bit disparity value to floating point x,y,z
                // look at points within a range
                _cloud.points[k].x = x;
                _cloud.points[k].y = y;
                _cloud.points[k].z = z;
            }else{
                _cloud.points[k].x = std::numeric_limits<float>::quiet_NaN();
                _cloud.points[k].y = std::numeric_limits<float>::quiet_NaN();
                _cloud.points[k].z = std::numeric_limits<float>::quiet_NaN();
            }
            //_cloud.push_back(pr);
        }
    }
}

bool CameraBumblebee::capture(cv::Mat& _mat){
    if(!grabImage(_mat)) return false;
    return true;
}

bool CameraBumblebee::capture(pcl::PointCloud<pcl::PointXYZRGB>& _cloud){
    cv::Mat _mat;
    if(!grabImage(_mat)) return false;

    generate3DPoints(_cloud, _mat);
    return true;
}

bool CameraBumblebee::capture(
        pcl::PointCloud<pcl::PointXYZRGB>& _cloud,
        cv::Mat& _mat){
    if(!grabImage(_mat)) return false;
    generate3DPoints(_cloud, _mat);
    return true;
}


