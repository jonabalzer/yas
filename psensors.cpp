#include "psensors.h"

using namespace cv;
using namespace std;
using namespace openni;

CPrimeSensors::CPrimeSensors():
    m_kinect_cap(),
    m_ps_device(),
    m_ps_rgb(),
    m_ps_depth(),
    m_size(640,480),
    m_source(-1) {

    OpenNI::initialize();


}

CPrimeSensors::~CPrimeSensors() {

    CloseKinectModule();
    ClosePrimeSenseModule();

    OpenNI::shutdown();

}

const char* CPrimeSensors::GetURI(size_t no) {

     Array<DeviceInfo> devicelist;
     OpenNI::enumerateDevices(&devicelist);

     if(no>=devicelist.getSize()) {
         cout << "Device no. " << no << " not found." << endl;
         return NULL;

     }

     return devicelist[no].getUri();

};


void CPrimeSensors::ShowAvailablePrimeSenseModules() {

    OpenNI::initialize();

    Array<DeviceInfo> devicelist;
    OpenNI::enumerateDevices(&devicelist);

    for(int i=0; i<devicelist.getSize(); i++) {

        cout << "Device no: " << i << endl;
        cout << "Name: " << devicelist[i].getName() << endl;
        cout << "URI: " << devicelist[i].getUri() << endl;
        cout << "Vendor: " << devicelist[i].getVendor() << endl;
        cout << "USB Product ID: " << devicelist[i].getUsbProductId() << endl;
        cout << "USB Vendor ID: " << devicelist[i].getUsbVendorId() << endl;

        // fixme: query more information about the devices

    }

    OpenNI::shutdown();

};

void CPrimeSensors::ShowAvailableKinectModules() {

    // only supports a single device for now
    VideoCapture mscap(CV_CAP_OPENNI);

    if(mscap.isOpened()) {

        cout << "RGB FPS: " << mscap.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS) << endl;

    }

}


bool CPrimeSensors::OpenPrimeSenseModule() {

    // make sure the kinect sensor is not running
    if(m_kinect_cap.isOpened())
        return 1;

    // open device
    if(m_ps_device.open(ANY_DEVICE)!=STATUS_OK) {

        cout << "Couldn't open device: " << OpenNI::getExtendedError() << endl;
        return 1;

    }

    // open streams
    if (m_ps_device.getSensorInfo(SENSOR_COLOR) != NULL) {

        if(m_ps_rgb.create(m_ps_device, SENSOR_COLOR)!=STATUS_OK)
            return 1;

    }
    else
        return 1;

    if (m_ps_device.getSensorInfo(SENSOR_DEPTH) != NULL) {

        if(m_ps_depth.create(m_ps_device, SENSOR_DEPTH)!=STATUS_OK)
            return 1;

    }
    else
        return 1;

    // standard settings
    VideoMode mode = m_ps_rgb.getVideoMode();
    mode.setResolution(m_size.width,m_size.height);
    m_ps_rgb.setVideoMode(mode);
    m_ps_rgb.setMirroringEnabled(false);

    CameraSettings* cam = m_ps_rgb.getCameraSettings();
    cam->setAutoExposureEnabled(false);
    cam->setAutoWhiteBalanceEnabled(false);

    mode = m_ps_depth.getVideoMode();
    mode.setResolution(m_size.width,m_size.height);
    m_ps_depth.setVideoMode(mode);
    m_ps_depth.setMirroringEnabled(false);


   if(m_ps_rgb.start()!= STATUS_OK) {

       cout << "Couldn't start RGB stream: " << OpenNI::getExtendedError() << endl;
       return 1;

   }

   if(m_ps_depth.start()!= STATUS_OK) {

       cout << "Couldn't start depth stream: " << OpenNI::getExtendedError() << endl;
       return 1;

   }

   // set source flag
   m_source = 0;

   return 0;

}


bool CPrimeSensors::ClosePrimeSenseModule() {

    if(m_ps_depth.isValid()) {

        m_ps_depth.stop();
        m_ps_depth.destroy();

    }

    if(m_ps_rgb.isValid()) {

        m_ps_rgb.stop();
        m_ps_rgb.destroy();

    }

    if(m_ps_device.isValid())
        m_ps_device.close();

    m_source = -1;

    return 0;

}

bool CPrimeSensors::OpenKinectModule() {

    // make sure no other primesense module is open
    if(m_ps_device.isValid() || m_ps_rgb.isValid() || m_ps_depth.isValid())
        ClosePrimeSenseModule();

    m_kinect_cap = VideoCapture(CV_CAP_OPENNI);

    if(!m_kinect_cap.isOpened())
        return 1;

    // set source flag
    m_source = 1;

    return 0;

}

bool CPrimeSensors::CloseKinectModule() {

    if(m_kinect_cap.isOpened())
        m_kinect_cap.release();

    m_source = -1;

    return 0;

}

int CPrimeSensors::WhichSource() {

     return m_source;

}


bool CPrimeSensors::IsSane() {

    switch(m_source) {

    case 0:
        return m_ps_depth.isValid() && m_ps_rgb.isValid() && m_ps_depth.isValid();
        break;
    case 1:

        return m_kinect_cap.isOpened();
        break;
    default:
        return 0;
        break;

    }

    return 0;

}


bool CPrimeSensors::Get(Mat& rgb, Mat& depth) {

    if(m_source==1) {

        if(!m_kinect_cap.grab())
            return 1;

    }

    rgb = GetRGB();
    depth = GetDepth();

}


Mat CPrimeSensors::GetRGB() {

    Mat rgb;
    bool error;

    switch(m_source) {

    case 0:

        error = GetPrimeSenseRGB(rgb);

        break;

    case 1:

        error = GetKinectRGB(rgb);

        break;
    }

    if(error)
        rgb = Mat::zeros(m_size.height,m_size.width,CV_8UC3);

    return rgb;

}

Mat CPrimeSensors::GetDepth() {

    Mat depth;
    bool error;

    switch(m_source) {

    case 0:

        error = GetPrimeSenseDepth(depth);

        break;

    case 1:

        error = GetKinectDepth(depth);

        break;

    }

    if(error)
        depth = Mat::zeros(m_size.height,m_size.width,CV_16UC1);

    return depth;

}


bool CPrimeSensors::GetPrimeSenseRGB(Mat& rgb) {

    VideoFrameRef frame;

    if(m_ps_rgb.readFrame(&frame)!=STATUS_OK)
        return 1;

    rgb = Mat(frame.getHeight(),frame.getWidth(),CV_8UC3,(unsigned char*)frame.getData());

    return 0;

}

bool CPrimeSensors::GetPrimeSenseDepth(Mat& depth) {

    VideoFrameRef frame;

    if(m_ps_depth.readFrame(&frame)!=STATUS_OK)
        return 1;

    depth = Mat(frame.getHeight(),frame.getWidth(),CV_16UC1,(unsigned short*)frame.getData());

    return 0;

}

bool CPrimeSensors::GetKinectRGB(Mat& rgb) {

    bool error = m_kinect_cap.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE);
    cvtColor(rgb,rgb,CV_BGR2RGB);

    return error;


}

bool CPrimeSensors::GetKinectDepth(Mat& depth) {

    return m_kinect_cap.retrieve(depth,CV_CAP_OPENNI_DEPTH_MAP);

}

vector<float> CPrimeSensors::GetFocalLengths() {

    vector<float> result;

    switch(m_source) {

    case 0:
    {

        result.push_back(320.0/tan(0.5*m_ps_rgb.getHorizontalFieldOfView()));
        result.push_back(240.0/tan(0.5*m_ps_rgb.getVerticalFieldOfView()));

        break;

    }
    default:
    {

        result.push_back(525.0);
        result.push_back(525.0);

    }

    }

    return result;

}

std::vector<float> CPrimeSensors::GetPrincipalPoint() {

    vector<float> result;
    result.push_back((float)m_size.width*0.5);
    result.push_back((float)m_size.height*0.5);

    return result;

}




