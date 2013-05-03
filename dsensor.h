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
    virtual ~CDepthColorSensor() { CloseDevice(); }

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
    void ConfigureDepth(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& range, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a);

    //! Converts disparity into 3d point in depth cam coordinates.
    cv::Point3f GetPoint(size_t i, size_t j, const cv::Mat& disp);

    //! Converts disparity into 3d point in depth cam coordinates.
    cv::Vec3f GetNormal(size_t i, size_t j, const cv::Mat& disp);

    //! Gets the color at a point in world (depth) coordinates.
    cv::Vec3b GetColor(cv::Point3f x, const cv::Mat& rgb);

    //! Warps an RGB image to the image plane of the depth sensor.
    cv::Mat WarpRGBToDepth(const cv::Mat& disp, const cv::Mat& rgb);

    /*! \brief Warps a depth image to the image plane of the RGB sensor.
     *
     * \details This is a HACK. Needs to be done with spline interpolation.
     */
    cv::Mat WarpDepthToRGB(const cv::Mat& disp, const cv::Mat& rgb);

    //! Access to maximum disparity.
    float DisparityToDepth(int d);

    //! Gets disparity range.
    void GetDisparityRange(size_t& min, size_t& max);

    //! Start dumping streams into a ONI file.
    bool StartRecording(const char* filename);

    //! Stop file dump.
    bool StopRecording();

    //! Access to camera.
    CCam& GetRGBCam() { return m_rgb_cam; }

    //! Access to depth camera.
    CDepthCam& GetDepthCam() { return m_depth_cam; }

    //! Write sensor configuration to a stream.
    friend std::ostream& operator << (std::ostream& os, const CDepthColorSensor& x);

    //! Reads sensor configuration from a stream.
    friend std::istream& operator >> (std::istream& is, CDepthColorSensor& x);

private:

    openni::Device m_device;                                   //! OpenNI device
    CCam m_rgb_cam;
    openni::VideoStream m_rgb_stream;                          //! OpenNI image stream
    CDepthCam m_depth_cam;
    openni::VideoStream m_depth_stream;                        //! OpenNI depth stream
    openni::Recorder m_recorder;                               //! a recorder


    // protect from copying
    CDepthColorSensor(const CDepthColorSensor& sensor);
    CDepthColorSensor operator=(const CDepthColorSensor& sensor);

};


#endif // DSENSOR_H
