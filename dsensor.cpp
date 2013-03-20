#include "dsensor.h"

using namespace openni;
using namespace std;
using namespace cv;

map<int,string> CDepthColorSensor::ShowAvailableSensors() {


    map<int,string> result;

    OpenNI::initialize();

    Array<DeviceInfo> devicelist;
    OpenNI::enumerateDevices(&devicelist);

    for(int i=0; i<devicelist.getSize(); i++) {

        result.insert(pair<int,string>(i,devicelist[i].getName()));

        cout << "Device no: " << i << endl;
        cout << "Name: " << devicelist[i].getName() << endl;
        cout << "URI: " << devicelist[i].getUri() << endl;
        cout << "Vendor: " << devicelist[i].getVendor() << endl;
        cout << "USB Product ID: " << devicelist[i].getUsbProductId() << endl;
        cout << "USB Vendor ID: " << devicelist[i].getUsbVendorId() << endl;

    }

    OpenNI::shutdown();

    return result;

}

CDepthColorSensor::CDepthColorSensor():
    m_device(),
    m_rgb_cam(),
    m_rgb_stream(),
    m_depth_cam(),
    m_depth_stream() {

    OpenNI::initialize();

}

CDepthColorSensor::CDepthColorSensor(CCam rgb, CDepthCam depth):
    m_device(),
    m_rgb_cam(rgb),
    m_rgb_stream(),
    m_depth_cam(depth),
    m_depth_stream() {

    OpenNI::initialize();

}

bool CDepthColorSensor::OpenDevice(int i) {

    Array<DeviceInfo> devicelist;
    OpenNI::enumerateDevices(&devicelist);

    if(i>=devicelist.getSize()) {
        cout << "Device no. " << i << " not found." << endl;
        return false;

    }

   // open device
   if(m_device.open(devicelist[i].getUri())!=STATUS_OK) {

       cout << "Couldn't open device: " << OpenNI::getExtendedError() << endl;
       return 1;

   }

   // set registration property, we will do this manually
   m_device.setImageRegistrationMode(IMAGE_REGISTRATION_OFF);

   // open streams
   if (m_device.getSensorInfo(SENSOR_COLOR) != NULL) {

       if(m_rgb_stream.create(m_device, SENSOR_COLOR)!=STATUS_OK)
           return 1;

   }
   else
       return 1;

   if (m_device.getSensorInfo(SENSOR_DEPTH) != NULL) {

       if(m_depth_stream.create(m_device, SENSOR_DEPTH)!=STATUS_OK)
           return 1;

   }
   else
       return 1;

   // standard settings
   VideoMode mode(m_rgb_stream.getVideoMode());
   mode.setResolution(m_rgb_cam.m_size[0],m_rgb_cam.m_size[1]);
   m_rgb_stream.setVideoMode(mode);
   m_rgb_stream.setMirroringEnabled(false);

   CameraSettings* cam = m_rgb_stream.getCameraSettings();
   cam->setAutoExposureEnabled(false);
   cam->setAutoWhiteBalanceEnabled(false);

   VideoMode dmode(m_depth_stream.getVideoMode());
   dmode.setResolution(m_depth_cam.m_size[0],m_depth_cam.m_size[1]);
   dmode.setPixelFormat(PIXEL_FORMAT_SHIFT_9_2);
   m_depth_stream.setVideoMode(dmode);
   m_depth_stream.setMirroringEnabled(false);

   if(m_rgb_stream.start()!= STATUS_OK) {

      cout << "Couldn't start RGB stream: " << OpenNI::getExtendedError() << endl;
      return 1;

   }

  if(m_depth_stream.start()!= STATUS_OK) {

      cout << "Couldn't start depth stream: " << OpenNI::getExtendedError() << endl;
      return 1;

  }

  return 0;

}

bool CDepthColorSensor::CloseDevice() {

    if(m_depth_stream.isValid()) {

        m_depth_stream.stop();
        m_depth_stream.destroy();

    }

    if(m_rgb_stream.isValid()) {

        m_rgb_stream.stop();
        m_rgb_stream.destroy();

    }

    if(m_device.isValid())
        m_device.close();

    return 0;

}

Mat CDepthColorSensor::GetRGB() {

    VideoFrameRef frame;

    if(m_rgb_stream.readFrame(&frame)!=STATUS_OK)
        return Mat::zeros(m_rgb_cam.m_size[1],m_rgb_cam.m_size[0],CV_8UC3);

    return Mat(frame.getHeight(),frame.getWidth(),CV_8UC3,(unsigned char*)frame.getData());

}

Mat CDepthColorSensor::GetDepth() {

    VideoFrameRef frame;

    if(m_depth_stream.readFrame(&frame)!=STATUS_OK)
        return Mat::zeros(m_depth_cam.m_size[1],m_depth_cam.m_size[0],CV_16UC1);

    return Mat(frame.getHeight(),frame.getWidth(),CV_16UC1,(unsigned short*)frame.getData());

}

bool CDepthColorSensor::Get(cv::Mat& rgb, cv::Mat& depth) {

    rgb = GetRGB();
    depth = GetDepth();

    return 0;

}

bool CDepthColorSensor::IsSane() {

    return m_depth_stream.isValid() && m_rgb_stream.isValid() && m_device.isValid();
}

void CDepthColorSensor::ConfigureRGB(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F) {

    m_rgb_cam.m_size[0] = size[0];
    m_rgb_cam.m_size[1] = size[1];
    m_rgb_cam.m_f[0] = f[0];
    m_rgb_cam.m_f[1] = f[1];
    m_rgb_cam.m_c[0] = c[0];
    m_rgb_cam.m_c[1] = c[1];
    m_rgb_cam.m_alpha = alpha;

    for(size_t i=0;i<5;i++)
        m_rgb_cam.m_k[i]=k[i];

    m_rgb_cam.m_F = F;
    m_rgb_cam.m_Finv = F.inv();

}

void CDepthColorSensor::ConfigureDepth(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a) {

    m_depth_cam.m_size[0] = size[0];
    m_depth_cam.m_size[1] = size[1];
    m_depth_cam.m_f[0] = f[0];
    m_depth_cam.m_f[1] = f[1];
    m_depth_cam.m_c[0] = c[0];
    m_depth_cam.m_c[1] = c[1];
    m_depth_cam.m_alpha = alpha;

    for(size_t i=0;i<5;i++)
        m_depth_cam.m_k[i]=k[i];

    m_depth_cam.m_F = F;
    m_depth_cam.m_Finv = F.inv();
    m_depth_cam.m_d[0] = d[0];
    m_depth_cam.m_d[1] = d[1];
    m_depth_cam.m_D = D;
    m_depth_cam.m_a[0] = a[0];
    m_depth_cam.m_a[1] = a[1];

    //m_depth_cam.SetMaxDisparity();

}

cv::Point3f CDepthColorSensor::GetPoint(size_t i, size_t j, const cv::Mat& disp) {

    float d = (float)disp.at<unsigned short>(i,j);

    float z = m_depth_cam.DisparityToDepth(i,j,d);

    Point2i u(j,i);

    Vec3f xc = m_depth_cam.UnProjectLocal(u);

    Point3f x = xc*z;

    return x;

}

Vec3b CDepthColorSensor::GetColor(cv::Point3f x, const cv::Mat& rgb) {

     // project it to rgb image plane
    Vec2f uc = m_rgb_cam.Project(x);

    // round
    int irgb = (int)floor(uc[1]);
    int jrgb = (int)floor(uc[0]);

    Vec3b result;
    if(irgb<0 || irgb>=rgb.cols || jrgb<0 || jrgb>=rgb.cols) {


        result *= 0;
        return result;

    }

    // interpolate
//    float vd = uc[1] - irgb;
//    float ud = uc[0] - jrgb;

//    Vec3f I00, I01, I10, I11, I0, I1;
//    I00 = (Vec3f)rgb.at<Vec3b>(irgb,jrgb);
//    I01 = (Vec3f)rgb.at<Vec3b>(irgb,jrgb+1);
//    I10 = (Vec3f)rgb.at<Vec3b>(irgb+1,jrgb);
//    I11 = (Vec3f)rgb.at<Vec3b>(irgb+1,jrgb+1);
//    I0 = I00*(1-ud) + I01*ud;
//    I1 = I10*(1-ud) + I11*ud;

    result = rgb.at<Vec3b>(irgb,jrgb);
    //result = (Vec3b)(I0*(1-vd) + I1*vd);

    return result;

}

Mat CDepthColorSensor::WarpRGBToDepth(const cv::Mat& disp, const cv::Mat& rgb) {

    Mat result = Mat(disp.rows,disp.cols,CV_8UC3);

    for(size_t i=0; i<disp.rows; i++) {

        for(size_t j=0; j<disp.cols; j++) {

            float d = disp.at<unsigned short>(i,j);
            float z = m_depth_cam.DisparityToDepth(i,j,d);

            Point2i u(j,i);

            Vec3f xc = m_depth_cam.UnProjectLocal(u);

            Point3f x = xc*z;

            result.at<Vec3b>(i,j) = GetColor(x,rgb);

        }

    }

    return result;

}

float CDepthColorSensor::DisparityToDepth(int d) {

     return 1.0/(m_depth_cam.m_d[0]+d*m_depth_cam.m_d[1]);

}
