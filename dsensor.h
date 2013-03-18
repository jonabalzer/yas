#ifndef DSENSOR_H
#define DSENSOR_H


#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include "cam.h"
#include <map>

class CDepthColorSensor {



public:

    //! Show which PrimeSense modules are available.
    static std::map<int,std::string> ShowAvailableSensors();

    //! Constructor.
    CDepthColorSensor();

    //! Destructor.
    virtual ~CDepthColorSensor() { CloseDevice(); };

    //! Parametrized constructor.
    CDepthColorSensor(CCam rgb, CDepthCam depth);

    //! Open a PrimeSense device by number.
    bool OpenDevice(int i);

    //! Closes the active device and all streams associated with it.
    bool CloseDevice();

    //! Get RGB image.
    cv::Mat GetRGB();

    //! Get a depth image.
    cv::Mat GetDepth();

    //! Get RGB/depth image (possibly time-synched)
    bool Get(cv::Mat& rgb, cv::Mat& depth);

    //! Check whether everything is still running smoothly.
    bool IsSane();

    //! Sets parameters of the RGB sensor.
    void ConfigureRGB(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F);

    //! Sets parameters of the depth sensor.
    void ConfigureDepth(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a);

private:

    openni::Device m_device;                                   //! OpenNI device
    CCam m_rgb_cam;
    openni::VideoStream m_rgb_stream;                          //! OpenNI image stream
    CDepthCam m_depth_cam;
    openni::VideoStream m_depth_stream;                        //! OpenNI depth stream

    // protect from copying
    CDepthColorSensor(const CDepthColorSensor& sensor);
    CDepthColorSensor operator=(const CDepthColorSensor& sensor);

};


#endif // DSENSOR_H
