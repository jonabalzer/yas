#ifndef PSENSORS_H
#define PSENSORS_H

#include <map>
#include <OpenNI.h>
#include "opencv2/opencv.hpp"


class CPrimeSensors {

public:

    CPrimeSensors();

    virtual ~CPrimeSensors();

    //! Show which PrimeSense modules are available.
    static void ShowAvailablePrimeSenseModules();

    //! Show if there is a Kinect connected to the computer.
    static void ShowAvailableKinectModules();

    //! Open a PrimeSense device by number.
    bool OpenPrimeSenseModule();

    //! Close the PrimeSense device which is currently open.
    bool ClosePrimeSenseModule();

    //! Open a Kinect accessed via Avins driver hack.
    bool OpenKinectModule();

    //! Close Kinect.
    bool CloseKinectModule();

    //! Get RGB/depth image (possibly time-synched)
    bool Get(cv::Mat& rgb, cv::Mat& depth);

    //! Checks which of the two sensor types is available.
    int WhichSource();

    //! Check whether everything is still running smoothly.
    bool IsSane();

    //! Returns focal lengths.
    std::vector<float> GetFocalLengths();

    //! Returns coordinates of the principal point.
    std::vector<float> GetPrincipalPoint();

private:

    cv::VideoCapture m_kinect_cap;                              //! video stream from open cv to access ms modules with avin's ms hack
    openni::Device m_ps_device;                                 //! OpenNI device
    openni::VideoStream m_ps_rgb;                               //! OpenNI image stream
    openni::VideoStream m_ps_depth;                             //! OpenNI depth stream
    cv::Size m_size;                                            //! size, fixed to 640x480 for now
    int m_source;                                               //! the current source

    //! Returns the URI to the device with a given number.
    const char* GetURI(size_t no);

    //! Get RGB image.
    cv::Mat GetRGB();

    //! Get a depth image.
    cv::Mat GetDepth();

    //! Records an RGB image with PrimeSense module.
    bool GetPrimeSenseRGB(cv::Mat& rgb);

    //! Records a depth image with PrimeSense module.
    bool GetPrimeSenseDepth(cv::Mat& depth);

    //! Records an RGB image with Kinect.
    bool GetKinectRGB(cv::Mat& rgb);

    //! Records a depth image with Kinect.
    bool GetKinectDepth(cv::Mat& depth);

    // protect copy constructor/assignment operators
    CPrimeSensors(const CPrimeSensors& ps);
    CPrimeSensors operator=(const CPrimeSensors& ps);

};

#endif // PSENSORS_H
