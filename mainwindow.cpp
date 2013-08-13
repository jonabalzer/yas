#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calignransac.h"
#include "cplaneransac.h"
#include "icp.h"

#include <sstream>
#include <fstream>
#include <algorithm>
#include <set>

#include <QFileDialog>
#include <QProgressDialog>
#include <QMessageBox>

#include <opencv2/nonfree/nonfree.hpp>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_sensor(),
    m_timer(this),
    m_rgb(),
    m_depth_buffer(3),
    m_rgb_storage(),
    m_depth_storage(),
    m_trafo_storage(),
    m_alignment(new AlignWindow),
    m_glview(new QGLViewerWidget()),
    m_params(new Params())
{

    ui->setupUi(this);
    ui->labelDepth->setScaledContents(true);
    ui->labelRGB->setScaledContents(true);
    this->setFixedSize(this->width(),this->height());
    ui->spinBoxStorage->setFocusPolicy(Qt::NoFocus);

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_stepButton_clicked()));
    connect(this,SIGNAL(current_image_changed(Mat&,Mat&)),this,SLOT(update_static_view(Mat&,Mat&)));
    connect(this,SIGNAL(current_pcl_changed(std::vector<cv::Point3f>,std::vector<cv::Vec3b>)),m_glview,SLOT(set_pcl(std::vector<cv::Point3f>,std::vector<cv::Vec3b>)));
    connect(m_params,SIGNAL(cam_params_changed(CCam,CDepthCam)),this,SLOT(configure_sensor(CCam,CDepthCam)));
    connect(m_params,SIGNAL(save_params_clicked()),this,SLOT(on_saveParams_clicked()));

    // load sensor data
    m_params->on_applyButton_clicked();

    // get min/max disparity (!) to set up slider
    size_t dmin, dmax;
    m_sensor.GetDisparityRange(dmin,dmax);
    ui->depthclipSlider->setMinimum(dmax);      // the slider refers to depth!
    ui->depthclipSlider->setMaximum(dmin);
    ui->depthclipSlider->setSliderPosition(dmin);

    float zmax = m_sensor.DisparityToDepth(dmin);
    QString label;
    label.setNum(zmax,'f',2);
    ui->maxDepth->setText(label);

    if(m_sensor.OpenDevice(0))
        QMessageBox::critical(this,"Error","Could not open source. Make sure the Kinect sensor is connected to your computer and drivers are working properly.");
    else
        CDepthColorSensor::ShowAvailableSensors();

}

MainWindow::~MainWindow() {

    delete m_params;
    delete m_glview;
    delete m_alignment;
    delete ui;

}

void MainWindow::configure_sensor(const CCam& rgb, const CDepthCam& depth) {

    CCam& rgbold = m_sensor.GetRGBCam();
    rgbold = rgb;

    CDepthCam& depthold = m_sensor.GetDepthCam();
    depthold = depth;

}

bool MainWindow::on_stepButton_clicked()
{

    Mat depth;

    // get new data (this return zero matrices if something is wrong with the sensor
    m_sensor.Get(m_rgb,depth);

    // push depth into ring buffer
    m_depth_buffer.push_back(depth.clone());

    // update live view
    update_live_view();

    return 0;

}

void MainWindow::on_runButton_clicked()
{
    if(m_sensor.IsSane())
        m_timer.start(1);
    else
        QMessageBox::critical(this,"Error","Could not open source. Make sure the Kinect sensor is connected to your computer and drivers are working properly.");

}

void MainWindow::on_pauseButton_clicked()
{
    m_timer.stop();

}

bool MainWindow::save_as_png(size_t index, QString fn) {

    Mat rgb;
    cvtColor(m_rgb_storage[index],rgb,CV_BGR2RGB);

    imwrite(fn.toStdString().c_str(),rgb);

    return 0;

}

bool MainWindow::save_as_exr(size_t index, QString fn) {

    bool warp = m_params->warp_to_rgb();

    Mat wdepth;

    if(warp)
        wdepth = m_sensor.WarpDepthToRGB(m_depth_storage[index],m_rgb_storage[index]);

    Array2D<Rgba> out(m_rgb_storage[index].rows,m_rgb_storage[index].cols);

    for(size_t i=0; i<(size_t)m_rgb_storage[index].rows; i++) {

        for(size_t j=0; j<(size_t)m_rgb_storage[index].cols; j++) {

            Rgba val;
            val.r = half(m_rgb_storage[index].at<Vec3b>(i,j)[0]);
            val.g = half(m_rgb_storage[index].at<Vec3b>(i,j)[1]);
            val.b = half(m_rgb_storage[index].at<Vec3b>(i,j)[2]);

            if(!warp) {

                unsigned short d = m_depth_storage[index].at<unsigned short>(i,j);
                val.a = half(d);

            }
            else
                val.a = half(wdepth.at<float>(i,j));

            out[i][j] = val;

        }

    }

    //Header header(m_rgb.cols,m_rgb.rows);
    //header.insert ("comments", StringAttribute ("written by ucla vision lab kinect scan"));
    //header.insert ("cameraTransform", M44fAttribute (cameraTransform));

    RgbaOutputFile file(fn.toStdString().c_str(),m_rgb_storage[index].cols,m_rgb_storage[index].rows, WRITE_RGBA);
    file.setFrameBuffer (&out[0][0],1,m_rgb_storage[index].cols);
    file.writePixels (m_rgb_storage[index].rows);

    return 0;

}

bool MainWindow::save_trafo(size_t index, QString fn) {

    ofstream out(fn.toStdString().c_str());

    if(!out)
        return 1;

    CCam cam = m_sensor.GetRGBCam();
    Mat& F = cam.GetExtrinsics();

    F = transform_to_first_image(index);

    out << cam << endl;

    out.close();

    return 0;

}

bool MainWindow::save_pcl_as_ply(size_t index, QString fn) {

    // get transformation to first frame, if it exists
    Mat F = transform_to_first_image(index);
    vector<Point3f> points;
    vector<Point3f> normals;
    vector<Vec3b> colors;

    float zmax = m_sensor.DisparityToDepth(ui->depthclipSlider->sliderPosition());

    get_oriented_pcl(index,points,normals,colors,zmax,F);

    ofstream out(fn.toStdString().c_str());

    if(!out)
        return 1;

    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "comment written by ucla vision lab kinect scan" << endl;
    out << "element vertex " << points.size() << endl;
    out << "property float32 x" << endl;
    out << "property float32 y" << endl;
    out << "property float32 z" << endl;
    out << "property float32 nx" << endl;
    out << "property float32 ny" << endl;
    out << "property float32 nz" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "end_header" << endl;

    for(size_t i=0; i<points.size(); i++)
        out << points[i].x << " " << points[i].y << " " << points[i].z << " " << normals[i].x << " " << normals[i].y << " " << normals[i].z << " " << (unsigned int)colors[i][0] << " " << (unsigned int)colors[i][1] << " " << (unsigned int)colors[i][2] << endl;

    out.close();

    return 0;

}


bool MainWindow::save_mesh_as_ply(size_t index, QString fn) {

    // get transformation to first frame, if it exists
    Mat F = transform_to_first_image(index);
    vector<Point3f> vertices;
    vector<Vec3b> colors;
    vector<Vec4i> faces;

    float zmax = m_sensor.DisparityToDepth(ui->depthclipSlider->sliderPosition());

    get_mesh(index,vertices,colors,faces,zmax,F);

    ofstream out(fn.toStdString().c_str());

    if(!out)
        return 1;

    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "comment written by ucla vision lab kinect scan" << endl;
    out << "element vertex " << vertices.size() << endl;
    out << "property float32 x" << endl;
    out << "property float32 y" << endl;
    out << "property float32 z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "element face " << faces.size() << endl;
    out << "property list uchar int vertex_index" << endl;
    out << "end_header" << endl;

    // write vertices
    for(size_t i=0; i<vertices.size(); i++)
        out << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << " " << (unsigned int)colors[i][0] << " " << (unsigned int)colors[i][1] << " " << (unsigned int)colors[i][2] << endl;

    // write faces
    for(size_t i=0; i<faces.size(); i++)
        out << "4 " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << " " << faces[i][3] << endl;

    out.close();

    return 0;

}

bool MainWindow::save_as_pgm(size_t index, QString fn) {

    vector<int> params;
    params.push_back(CV_IMWRITE_PXM_BINARY);
    params.push_back(1);

    return !imwrite(fn.toStdString().c_str(),m_depth_storage[index],params);

}

void MainWindow::on_actionUpdateClipDepth_triggered()
{

    int d = ui->depthclipSlider->sliderPosition();
    float zmax = m_sensor.DisparityToDepth(d);

    QString label;
    label.setNum(zmax,'f',2);
    ui->maxDepth->setText(label);

    int index = ui->spinBoxStorage->value();

    if(index>0)
        emit current_image_changed(m_rgb_storage[index-1],m_depth_storage[index-1]);

}

unsigned short MainWindow::get_smoothed_depth(size_t i, size_t j) {

    size_t s = m_depth_buffer.size();

    unsigned short* vals = new unsigned short[s];

    for(size_t k=0; k<s; k++)
        vals[k] = m_depth_buffer[k].at<unsigned short>(i,j);

    sort(vals,vals+s);

    unsigned short result = 0;

    if(s%2==1)
        result = vals[(size_t)((s+1)/2)-1];
    else
        result = 0.5*(vals[s/2-1]+vals[s/2]);

    delete [] vals;

    return result;

}

Mat MainWindow::get_depth_from_buffer() {

    Mat result(Size(m_depth_buffer.back().cols,m_depth_buffer.back().rows),CV_16UC1);

    for(size_t i=0; i<(size_t)result.rows; i++) {

        for(size_t j=0; j<(size_t)result.cols; j++)
            result.at<unsigned short>(i,j) = get_smoothed_depth(i,j);

    }

    return result;

}

void MainWindow::on_action3D_View_triggered()
{
   //m_viewer->show();
   //m_viewer->raise();
   //m_viewer->activateWindow();

    m_glview->show();

}

void MainWindow::on_storeButton_clicked()
{

    // stop timer to make sure that the buffer does not get flushed
    m_timer.stop();

    if(!on_stepButton_clicked()) {

        // send copy of current rgb/depth to storage
        m_rgb_storage.push_back(m_rgb.clone());
        m_depth_storage.push_back(get_depth_from_buffer());

        // store a idendity matrix as transformation
        Mat trafo = Mat::eye(4,4,CV_32FC1);
        m_trafo_storage.push_back(trafo);

        // increment display and max of spin box, do this 1-based
        ui->spinBoxStorage->setMaximum(m_rgb_storage.size());

        if(m_rgb_storage.size()==1)
            ui->spinBoxStorage->setMinimum(1);

        ui->spinBoxStorage->setValue(m_rgb_storage.size());

        // show images
        emit current_image_changed(m_rgb_storage.back(),m_depth_storage.back());

        // restart timer only if capture was successful
        m_timer.start(1);

    }

}

void MainWindow::on_spinBoxStorage_valueChanged(int arg1)
{

    ui->statusBar->clearMessage();

    if(arg1>0 && arg1<=(int)m_rgb_storage.size()) {

        ui->spinBoxStorage->setValue(arg1);

        // show images
        emit current_image_changed(m_rgb_storage[arg1-1],m_depth_storage[arg1-1]);

        if(!m_glview->isHidden()) {

            Mat F = transform_to_first_image(arg1-1);
            vector<Point3f> points;
            vector<Vec3b> colors;

            float zmax = m_sensor.DisparityToDepth(ui->depthclipSlider->sliderPosition());

            get_pcl(arg1-1,points,colors,zmax,F);

            emit current_pcl_changed(points,colors);

        }

    }

}

void MainWindow::on_alignButton_clicked()
{

    // get image index
    size_t index = (size_t)ui->spinBoxStorage->value();

    // check whether there is something to do
    if(index==m_rgb_storage.size() || m_rgb_storage.size()<2)
        return;

    // stop acqusition to have computational resources
    m_timer.stop();

    // show window
    m_alignment->show();
    m_alignment->activateWindow();
    m_alignment->raise();

    // get params
    size_t nfeat, noctaves, nsamples;
    double pthresh, ethresh, ratio, athresh;
    m_params->get_alignment_parameters(nfeat,noctaves,pthresh,ethresh,ratio,nsamples,athresh);

    // init non-free module
    initModule_nonfree();

    // create SIFT object
    SIFT detector(nfeat,noctaves,pthresh,ethresh,0.5);

    // warp to depth image plane
    Mat rgb0 = m_sensor.WarpRGBToDepth(m_depth_storage[index-1],m_rgb_storage[index-1]);
    Mat rgb1 = m_sensor.WarpRGBToDepth(m_depth_storage[index],m_rgb_storage[index]);

    // detect and compute descriptors
    vector<KeyPoint> kp0, kp1;
    Mat desc0, desc1;
    detector(rgb0,Mat(),kp0,desc0);
    detector(rgb1,Mat(),kp1,desc1);

    // matching
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    vector<vector<DMatch> > matches;
    matcher->knnMatch(desc0,desc1,matches,2);

    // compute 3d points
    vector<DMatch> good_matches;
    vector<Vec3f> x0, x1;

    int maxd = ui->depthclipSlider->maximum();
    int mind = ui->depthclipSlider->minimum();

    for (size_t i = 0; i<matches.size(); i++) {

        // check if match is good
        if(matches[i][0].distance/matches[i][1].distance<ratio) {

            // get both locations
            Point2f u0, u1;
            u0 = kp0[matches[i][0].queryIdx].pt;
            u1 = kp1[matches[i][0].trainIdx].pt;

            // if both points have depth, compute their 3d location
            if(m_depth_storage[index-1].at<unsigned short>((size_t)u0.y,(size_t)u0.x)<maxd
               && m_depth_storage[index].at<unsigned short>((size_t)u1.y,(size_t)u1.x)<maxd
               && m_depth_storage[index-1].at<unsigned short>((size_t)u0.y,(size_t)u0.x)>mind
               && m_depth_storage[index].at<unsigned short>((size_t)u1.y,(size_t)u1.x)>mind) {

                good_matches.push_back(matches[i][0]);
                x0.push_back(m_sensor.GetPoint((size_t)u0.y,(size_t)u0.x,m_depth_storage[index-1]));
                x1.push_back(m_sensor.GetPoint((size_t)u1.y,(size_t)u1.x,m_depth_storage[index]));

            }

        }

    }

    // create of good matches visualization
    Mat img_matches;
    drawMatches(rgb0,
                kp0,
                rgb1,
                kp1,
                good_matches,
                img_matches,
                Scalar::all(-1),
                Scalar::all(-1),
                vector<char>(),
                DrawMatchesFlags::DEFAULT);

    QImage vis(img_matches.data,img_matches.cols,img_matches.rows,QImage::Format_RGB888);
    m_alignment->show_image(vis);
    m_alignment->repaint();

    // create ransac object
    CAlignRansac alignment(x0,x1);

    // optimize
    size_t ninliers = 0;
    Mat F = alignment.RunConcensus(nsamples,athresh,ninliers,this);

    // bring alignment vis back
    m_alignment->raise();

    // store transformation
    m_trafo_storage[index] = F;

    // show inlier/outlier ratio
    double ioratio = (double)ninliers/(double)good_matches.size();
    ioratio *= 100;
    stringstream ss;
    ss << "The inlier ratio is " << ioratio << "\%.";
    ui->statusBar->showMessage(ss.str().c_str());

    // refine by icp
    refine_alignement(index);

}

Mat MainWindow::transform_to_first_image(size_t index){

    Mat F = m_trafo_storage[0].clone();

    if(index==0)
        return F;

    for(size_t i=1; i<=index; i++)
        F = F*m_trafo_storage[i];

    return F;

}


void MainWindow::keyPressEvent(QKeyEvent* event) {

    //cout << event->key() << endl;

    switch(event->key()) {

    case 16777268:

        if(!ui->recordButton->isChecked()) {

            ui->recordButton->setChecked(true);
            on_recordButton_clicked(true);

        }

        break;

    case 16777216:

        if(ui->recordButton->isChecked()) {

            ui->recordButton->setChecked(false);
            on_recordButton_clicked(false);

        }

        break;

    case 16777239:  // remote with logitech

        on_storeButton_clicked();

        break;

    case Qt::Key_Return:

        on_storeButton_clicked();

        break;

    }

}

void MainWindow::on_actionExit_triggered()
{
    QApplication::exit();
}

void MainWindow::on_clearButton_clicked()
{

    // get image index
    size_t index = (size_t)ui->spinBoxStorage->value();

    if(index==0)
        return;

    index--;

    // delete from storage
    m_rgb_storage.erase(m_rgb_storage.begin()+index);
    m_depth_storage.erase(m_depth_storage.begin()+index);
    m_trafo_storage.erase(m_trafo_storage.begin()+index);

    // correct actual minimum
    size_t max = (size_t)ui->spinBoxStorage->maximum();
    max--;
    ui->spinBoxStorage->setMaximum(max);
    ui->spinBoxStorage->setValue(index);

    // update display
    if(m_rgb_storage.size()>0) {

        if(index>0)
            index--;

        emit current_image_changed(m_rgb_storage[index],m_depth_storage[index]);

    }
    else {

        ui->labeRGBStorage->clear();
        ui->labelDepthStorage->clear();
        ui->spinBoxStorage->setMinimum(0);

    }

}


void MainWindow::update_live_view() {

    float dmax = (float)ui->depthclipSlider->sliderPosition();
    float dmin = (float)ui->depthclipSlider->minimum();

    // visualize
    QImage cimg(m_rgb.data,m_rgb.cols,m_rgb.rows,QImage::Format_RGB888);
    ui->labelRGB->setPixmap(QPixmap::fromImage(cimg));

    Mat img = m_depth_buffer.back();
    Mat depthf  (Size(640,480),CV_8UC1);
    img.convertTo(depthf, CV_8UC1, 255.0/6000.0);

    Mat depthrgb;
    cvtColor(depthf, depthrgb, CV_GRAY2BGR);

    QImage imgdd(depthrgb.data,depthf.cols,depthf.rows,QImage::Format_RGB888);

    float variance = m_params->get_triangulation_threshold(); // w.r.t. depth

    for(size_t i=0; i<(size_t)img.rows-1; i++) {

        for(size_t j=0; j<(size_t)img.cols-1; j++) {

            float zq[4];
            zq[0] = m_sensor.DisparityToDepth(img.at<unsigned short>(i,j));
            zq[1] = m_sensor.DisparityToDepth((float)img.at<unsigned short>(i+1,j));
            zq[2] = m_sensor.DisparityToDepth((float)img.at<unsigned short>(i,j+1));
            zq[3] = m_sensor.DisparityToDepth((float)img.at<unsigned short>(i+1,j+1));

            if(img.at<unsigned short>(i,j)>dmax || img.at<unsigned short>(i,j)<dmin)
                imgdd.setPixel(j,i,qRgb(0,0,0));
            else if(img.at<unsigned short>(i,j)<dmax &&
                    img.at<unsigned short>(i+1,j)<dmax
                    && img.at<unsigned short>(i,j+1)<dmax
                    && img.at<unsigned short>(i+1,j+1)<dmax) {

                float mean = 0.25*(zq[0] + zq[1] + zq[2] + zq[3]);

                float var = 0;
                for(size_t k=0; k<4; k++)
                    var += (zq[k]-mean)*(zq[k]-mean);
                var = sqrt(var);

                if(var>variance)
                    imgdd.setPixel(j,i,qRgb(255,0,0));

            }
            else
                imgdd.setPixel(j,i,qRgb(0,0,0));

        }

    }

    ui->labelDepth->setPixmap(QPixmap::fromImage(imgdd));

}

void MainWindow::update_static_view(Mat& rgb, Mat& depth) {

    float dmax = (float)ui->depthclipSlider->sliderPosition();
    float dmin = (float)ui->depthclipSlider->minimum();
    float dnorm = (float)ui->depthclipSlider->maximum() - dmin;

    QImage cimg(rgb.data,rgb.cols,rgb.rows,QImage::Format_RGB888);
    ui->labeRGBStorage->setPixmap(QPixmap::fromImage(cimg));

    QImage imgdd(depth.cols,depth.rows,QImage::Format_RGB888);

    for(size_t i=0; i<(size_t)depth.rows-1; i++) {

        for(size_t j=0; j<(size_t)depth.cols-1; j++) {

            float z = (float)depth.at<unsigned short>(i,j);

            if(z>=dmax || z<=dmin)      // this is actually disparity
                imgdd.setPixel(j,i,qRgb(0,0,0));
            else {

                QColor val = QColor::fromHsvF(((z-dmin)/dnorm)*0.6667,1.0,1.0);
                imgdd.setPixel(j,i,val.rgb());

            }

        }

    }

    ui->labelDepthStorage->setPixmap(QPixmap::fromImage(imgdd));

}

void MainWindow::on_actionAbout_triggered()
{
      QMessageBox::information(this,"About","More info: http://vision.ucla.edu");
}


Mat MainWindow::estimate_world_frame() {

    Mat T = Mat::zeros(m_trafo_storage.size()-1,3,CV_32FC1);
    Mat F = Mat::eye(4,4,CV_32FC1);

    Vec3f mean;
    mean *= 0;

    for(size_t i=1; i<m_trafo_storage.size(); i++) {

        // trafo from cam i to cam 0
        F = F*m_trafo_storage[i];

        // keep translations
        for(size_t j=0; j<3; j++) {

            T.at<float>(i-1,j) = F.at<float>(j,3);
            mean[j] += F.at<float>(j,3);
        }

    }

    // first guess for origin (project onto dominant plane later)
    mean *= (1.0/(float)(m_trafo_storage.size()-1));

    // remove mean
    for(size_t i=0; i<(size_t)T.rows; i++) {

        for(size_t j=0; j<3; j++)
            T.at<float>(i,j) -= mean[j];

    }

    // compute normal by SVD
    Mat Sigma, U, Vt;
    SVD::compute(T, Sigma, U, Vt);
    Vec3f ez;

    for(size_t j=0; j<3; j++)
        ez[j] = Vt.at<float>(2,j);


    // find the first non-zero vector in T
    Vec3f ex;
    for(size_t i=0; i<(size_t)T.rows; i++) {

        for(size_t j=0; j<3; j++)
            ex[j] = T.at<float>(i,j);

        if(cv::norm(ex)>0)
            break;

    }

    // check if non-zero vector was found
    if(cv::norm(ex)==0)
        return F;

    // if yes, make it orthogonal to ez and normalize
    ex = ex - ez*(ex[0]*ez[0] + ex[1]*ez[1] + ex[2]*ez[2]);
    ex *= (1/cv::norm(ex));

    // fill in frame from world 1 to cam 0
    Mat Fw1 = Mat::eye(4,4,CV_32FC1);
    for(size_t i=0; i<3; i++) {

        Fw1.at<float>(i,0) = ex[i];
        Fw1.at<float>(i,2) = ez[i];
        Fw1.at<float>(i,3) = mean[i];

    }

    // ey by cross product
    Fw1.at<float>(0,1) = ez[1]*ex[2] - ez[2]*ex[1];
    Fw1.at<float>(1,1) = ez[2]*ex[0] - ez[0]*ex[2];
    Fw1.at<float>(2,1) = ez[0]*ex[1] - ez[1]*ex[0];

    // invert, this will be pre-multiplied to m_trafo_storage[0]
    Mat Fw1inv = Fw1.inv();

    // get point cloud of first view, FIXME: set from zmax
    //float maxr = (float)m_zmax;
    vector<Point3f> pcl;
    vector<Vec3b> colors;
    get_pcl(0,pcl,colors,0.5,Fw1inv);

    // estimate plane parameters w.r.t. Fw1
    CEstimatePlaneRansac pransac(pcl);
    size_t ninliers;
    Vec4f plane = pransac.RunConsensus(5000,5,ninliers,this);

    cout << plane[0] << " " << plane[1] << " " << plane[2] << " " << plane[3] << endl;

    // build frame from world 2 -> world 1
    Mat Fw2 = Mat::eye(4,4,CV_32FC1);

    // normal of plane gives news ez axis
    for(size_t i=0; i<3; i++)
        ez[i] = plane[i];

    // project origin on plane
    mean = ez*plane[3];

    // project (1,0,0) onto plane
    ex *= 0;
    ex[0] = 1 - ez[0];
    cv::normalize(ex);

    // set columns 0,1,3 of Fw2
    for(size_t i=0; i<3; i++) {

        Fw2.at<float>(i,0) = ex[i];
        Fw2.at<float>(i,2) = ez[i];
        Fw2.at<float>(i,3) = mean[i];

    }

    // ey by cross product
    Fw2.at<float>(0,1) = ez[1]*ex[2] - ez[2]*ex[1];
    Fw2.at<float>(1,1) = ez[2]*ex[0] - ez[0]*ex[2];
    Fw2.at<float>(2,1) = ez[0]*ex[1] - ez[1]*ex[0];

    // inverse
    Mat Fw2inv = Fw2.inv();

    // Fwinv = Fw2inv*Fw1inv
    return Fw2inv*Fw1inv;

}

void MainWindow::get_pcl(size_t index, vector<Point3f>& vertices, vector<Vec3b>& colors, float maxr, Mat F) {

    // prepare transformation
    Mat Fsr = F(Range(0,3),Range(0,4));
    bool isfid = F.at<float>(0,0)==1 && F.at<float>(0,1)==0 && F.at<float>(0,2)==0 && F.at<float>(0,3)==0 &&
                 F.at<float>(1,0)==0 && F.at<float>(1,1)==1 && F.at<float>(1,2)==0 && F.at<float>(1,3)==0 &&
                 F.at<float>(2,0)==0 && F.at<float>(2,1)==0 && F.at<float>(2,2)==1 && F.at<float>(2,3)==0;

    // allocate space for temporary variable
    vector<Point3f> xarray;
    Point3f x;
    xarray.push_back(x);

    // collect points
    for(size_t i=0; i<(size_t)m_depth_storage[index].rows; i++) {

        for(size_t j=0; j<(size_t)m_depth_storage[index].cols; j++) {

            // only do something disparity is unsaturated
            if(m_depth_storage[index].at<unsigned short>(i,j)<=ui->depthclipSlider->maximum()) {

                x = m_sensor.GetPoint(i,j,m_depth_storage[index]);
                xarray[0] = x;

                // transform if F is not the idendity
                if(!isfid)
                    cv::transform(xarray,xarray,Fsr);

                // check distance condition
                if(cv::norm(xarray[0])<maxr) {

                    vertices.push_back(xarray[0]);

                    Vec3b color = m_sensor.GetColor(x,m_rgb_storage[index]);
                    colors.push_back(color);

                }

            }

        }

    }

}

void MainWindow::get_oriented_pcl(size_t index, vector<Point3f>& vertices, vector<Point3f>& normals, vector<Vec3b>& colors, float maxr, Mat F) {

    // prepare transformation
    Mat Fsr = F(Range(0,3),Range(0,4));
    bool isfid = F.at<float>(0,0)==1 && F.at<float>(0,1)==0 && F.at<float>(0,2)==0 && F.at<float>(0,3)==0 &&
                 F.at<float>(1,0)==0 && F.at<float>(1,1)==1 && F.at<float>(1,2)==0 && F.at<float>(1,3)==0 &&
                 F.at<float>(2,0)==0 && F.at<float>(2,1)==0 && F.at<float>(2,2)==1 && F.at<float>(2,3)==0;

    // rotation for normal
    Mat R = Fsr.clone();
    R.at<float>(0,3) = 0;
    R.at<float>(1,3) = 0;
    R.at<float>(2,3) = 0;

    // allocate space for temporary variable
    vector<Point3f> xarray;
    Point3f x;
    xarray.push_back(x);
    vector<Point3f> narray;
    Point3f n;
    narray.push_back(n);

    // collect points
    for(size_t i=0; i<(size_t)m_depth_storage[index].rows; i++) {

        for(size_t j=0; j<(size_t)m_depth_storage[index].cols; j++) {

            // only do something disparity is unsaturated
            if(m_depth_storage[index].at<unsigned short>(i,j)<=ui->depthclipSlider->maximum()) {

                x = m_sensor.GetPoint(i,j,m_depth_storage[index]);
                xarray[0] = x;

                // transform if F is not the idendity
                if(!isfid)
                    cv::transform(xarray,xarray,Fsr);

                // check distance condition
                if(cv::norm(xarray[0])<maxr) {

                    // transform normal
                    narray[0] = m_sensor.GetNormal(i,j,m_depth_storage[index]);

                    if(!isfid)
                        cv::transform(narray,narray,R);

                    if(cv::norm(narray[0])>0) {

                        vertices.push_back(xarray[0]);
                        normals.push_back(narray[0]);

                        Vec3b color = m_sensor.GetColor(x,m_rgb_storage[index]);
                        colors.push_back(color);

                    }

                }

            }

        }

    }

}


void MainWindow::get_mesh(size_t index, vector<Point3f>& vertices, vector<Vec3b>& colors, vector<Vec4i>& faces, float maxr, Mat F) {

    // prepare transformation
    Mat Fsr = F(Range(0,3),Range(0,4));
    bool isfid = F.at<float>(0,0)==1 && F.at<float>(0,1)==0 && F.at<float>(0,2)==0 && F.at<float>(0,3)==0 &&
                 F.at<float>(1,0)==0 && F.at<float>(1,1)==1 && F.at<float>(1,2)==0 && F.at<float>(1,3)==0 &&
                 F.at<float>(2,0)==0 && F.at<float>(2,1)==0 && F.at<float>(2,2)==1 && F.at<float>(2,3)==0;

    // allocate space for temporary variable
    vector<Point3f> xarray;
    Point3f x;
    xarray.push_back(x);

    // allocate map for topology
    map<size_t,size_t> indexhash;
    map<size_t,float> depthhash;

    // collect points
    for(size_t i=0; i<(size_t)m_depth_storage[index].rows; i++) {

        for(size_t j=0; j<(size_t)m_depth_storage[index].cols; j++) {

            unsigned short d = m_depth_storage[index].at<unsigned short>(i,j);
            float z = m_sensor.DisparityToDepth(d);

            // only do something disparity is unsaturated
            if(m_depth_storage[index].at<unsigned short>(i,j)<=ui->depthclipSlider->maximum()) {

                x = m_sensor.GetPoint(i,j,m_depth_storage[index]);
                xarray[0] = x;

                // transform if F is not the idendity
                if(!isfid)
                    cv::transform(xarray,xarray,Fsr);

                // check distance condition
                if(cv::norm(xarray[0])<maxr) {

                    vertices.push_back(xarray[0]);

                    Vec3b color = m_sensor.GetColor(x,m_rgb_storage[index]);
                    colors.push_back(color);

                    // insert into hash map: linear image index -> selected vertices
                    indexhash.insert(pair<size_t,size_t>(i*m_depth_storage[index].cols+j,vertices.size()-1));
                    depthhash.insert(pair<size_t,float>(i*m_depth_storage[index].cols+j,z));

                }

            }

        }

    }

    // now iterate through index hash
    map<size_t,float>::iterator ittl, ittr, itbr, itbl;

    float variance = m_params->get_triangulation_threshold();

    for(ittl=depthhash.begin(); ittl!=depthhash.end(); ittl++) {

        // see if all the neighbors are present
        ittr = depthhash.find(ittl->first+1);
        itbr = depthhash.find(ittl->first+m_depth_storage[index].cols+1);
        itbl = depthhash.find(ittl->first+m_depth_storage[index].cols);

        // if yes check variance
        if(ittr!=depthhash.end() && itbr!=depthhash.end() && itbl!=depthhash.end()) {

            float zq[4];
            zq[0] = ittl->second;
            zq[1] = ittr->second;
            zq[2] = itbr->second;
            zq[3] = itbl->second;

            float mean = 0.25*(zq[0] + zq[1] + zq[2] + zq[3]);

            float var = 0;
            for(size_t k=0; k<4; k++)
                var += (zq[k]-mean)*(zq[k]-mean);
            var = sqrt(var);

            if(var<variance) {

                Vec4i face;

                face[0] = indexhash[ittl->first];
                face[1] = indexhash[itbl->first];
                face[2] = indexhash[itbr->first];
                face[3] = indexhash[ittr->first];

                faces.push_back(face);

            }

        }

    }

}

void MainWindow::on_actionSave_triggered()
{

    // because of dialogue
    m_timer.stop();

    size_t index = (size_t)ui->spinBoxStorage->value();

    if(index>0)
        index--;
    else return;

    QString filename = QFileDialog::getSaveFileName(this, tr("Save file..."),
                               ".",
                               tr("*.png;;*.exr;;*.ply;;*.pgm;;*.txt"));

    bool error = false;

    if(filename.endsWith(".png"))
        error = save_as_png(index,filename);
    else if(filename.endsWith(".exr"))
        error = save_as_exr(index,filename);
    else if(filename.endsWith(".txt"))
        error = save_trafo(index,filename);
    else if (filename.endsWith(".ply")) {

        if(m_params->triangulate())
            error = save_mesh_as_ply(index,filename);
        else
            error = save_pcl_as_ply(index,filename);

    } else if(filename.endsWith(".pgm"))
        error = save_as_pgm(index,filename);
    else
        error = 0;

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

}

void MainWindow::on_actionOpen_triggered()
{

    QStringList filenames = QFileDialog::getOpenFileNames(this,tr("Open file..."),".",tr("*.exr"));

    if(filenames.size()==0)
        return;

    for(size_t i=0; i<filenames.size(); i++) {

        RgbaInputFile file(filenames[i].toStdString().c_str());

        Box2i dw = file.dataWindow();

        size_t width = dw.max.x - dw.min.x + 1;
        size_t height = dw.max.y - dw.min.y + 1;

        Array2D<Rgba> in(height,width);

        file.setFrameBuffer (&in[0][0] - dw.min.x - dw.min.y * width, 1, width);
        file.readPixels (dw.min.y, dw.max.y);

        // convert to opencv format
        Mat depth(height,width,CV_16UC1);
        Mat rgb(height,width,CV_8UC3);

        for(size_t i=0; i<(size_t)depth.rows; i++) {

            for(size_t j=0; j<(size_t)depth.cols; j++) {

                rgb.at<Vec3b>(i,j)[0] = in[i][j].r;
                rgb.at<Vec3b>(i,j)[1] = in[i][j].g;
                rgb.at<Vec3b>(i,j)[2] = in[i][j].b;
                depth.at<unsigned short>(i,j) = (unsigned short)in[i][j].a;

            }

        }

        m_rgb_storage.push_back(rgb);
        m_depth_storage.push_back(depth);

        // store a idendity matrix as transformation
        Mat trafo = Mat::eye(4,4,CV_32FC1);
        m_trafo_storage.push_back(trafo);

        // display
        emit current_image_changed(rgb,depth);

        // adjust counter
        ui->spinBoxStorage->setMaximum(m_rgb_storage.size());

        if(m_rgb_storage.size()==1)
            ui->spinBoxStorage->setMinimum(1);

        ui->spinBoxStorage->setValue(m_rgb_storage.size());

    }

}

void MainWindow::on_actionSave_all_triggered()
{

    m_timer.stop();

    QString filename = QFileDialog::getSaveFileName(this, tr("Save file..."),
                               ".",
                               tr("*.png;;*.exr;;*.ply;;*.pgm;;*.txt"));

    int format = 0;

    if(filename.endsWith(".png"))
        format = 0;
    else if(filename.endsWith(".exr"))
        format = 1;
    else if (filename.endsWith(".ply"))
        format = 2;
    else if(filename.endsWith(".pgm"))
        format = 3;
    else if(filename.endsWith(".txt"))
        format = 4;
    else
        return;

    filename.chop(4);

    bool error = false;

    QProgressDialog progress("Saving...", "Abort",0,(int)m_rgb_storage.size(),this);
    progress.setWindowTitle("KinectScan");
    progress.setWindowModality(Qt::WindowModal);

    for(size_t i=0; i<m_rgb_storage.size(); i++)  {

        progress.setValue(i);
        if (progress.wasCanceled())
            break;

        stringstream ss;
        ss.fill('0');
        ss.width(4);
        ss << i+1;

        string prefix = filename.toStdString() + ss.str();

        switch(format) {

        case 0:
        {
            error = save_as_png(i,QString((prefix+string(".png")).c_str()));

            break;
        }
        case 1:
        {
            error = save_as_exr(i,QString((prefix+string(".exr")).c_str()));

            break;
        }
        case 2:
        {

            QString fn = QString((prefix+string(".ply")).c_str());

            if(m_params->triangulate())
                error = save_mesh_as_ply(i,fn);
            else
                error = save_pcl_as_ply(i,fn);

            break;

        }
        case 3:
        {

            error = save_as_pgm(i,QString((prefix+string(".pgm")).c_str()));

            break;

        }

        case 4:
        {

            error = save_trafo(i,QString((prefix+string(".txt")).c_str()));

            break;

        }

        }

    }

    progress.setValue(m_rgb_storage.size());

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

}

void MainWindow::on_alignAllButton_clicked()
{

    if(m_rgb_storage.size()<2)
        return;

    m_timer.stop();

    // show window
    m_alignment->show();
    m_alignment->activateWindow();
    m_alignment->raise();

    // get params
    size_t nfeat, noctaves, nsamples;
    double pthresh, ethresh, ratio, athresh;
    m_params->get_alignment_parameters(nfeat,noctaves,pthresh,ethresh,ratio,nsamples,athresh);

    // init non-free module
    initModule_nonfree();

    // create SIFT object
    SIFT detector(nfeat,noctaves,pthresh,ethresh,0.5);

    for(size_t index=1; index<m_rgb_storage.size(); index++) {

        // warp to depth image plane
        Mat rgb0 = m_sensor.WarpRGBToDepth(m_depth_storage[index-1],m_rgb_storage[index-1]);
        Mat rgb1 = m_sensor.WarpRGBToDepth(m_depth_storage[index],m_rgb_storage[index]);

        // detect and compute descriptors
        vector<KeyPoint> kp0, kp1;
        Mat desc0, desc1;
        detector(rgb0,Mat(),kp0,desc0);
        detector(rgb1,Mat(),kp1,desc1);

        // matching
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
        vector<vector<DMatch> > matches;
        matcher->knnMatch(desc0,desc1,matches,2);

        // compute 3d points
        vector<DMatch> good_matches;
        vector<Vec3f> x0, x1;

        int maxd = ui->depthclipSlider->maximum();
        int mind = ui->depthclipSlider->minimum();

        for (size_t i = 0; i<matches.size(); i++) {

            // check if match is good
            if(matches[i][0].distance/matches[i][1].distance<ratio) {

                // get both locations
                Point2f u0, u1;
                u0 = kp0[matches[i][0].queryIdx].pt;
                u1 = kp1[matches[i][0].trainIdx].pt;

                // if both points have depth, compute their 3d location
                if(m_depth_storage[index-1].at<unsigned short>((size_t)u0.y,(size_t)u0.x)<maxd
                   && m_depth_storage[index].at<unsigned short>((size_t)u1.y,(size_t)u1.x)<maxd
                   && m_depth_storage[index-1].at<unsigned short>((size_t)u0.y,(size_t)u0.x)>mind
                   && m_depth_storage[index].at<unsigned short>((size_t)u1.y,(size_t)u1.x)>mind) {

                    good_matches.push_back(matches[i][0]);
                    x0.push_back(m_sensor.GetPoint((size_t)u0.y,(size_t)u0.x,m_depth_storage[index-1]));
                    x1.push_back(m_sensor.GetPoint((size_t)u1.y,(size_t)u1.x,m_depth_storage[index]));

                }

            }

        }

        // create of good matches visualization
        Mat img_matches;
        drawMatches(rgb0,
                    kp0,
                    rgb1,
                    kp1,
                    good_matches,
                    img_matches,
                    Scalar::all(-1),
                    Scalar::all(-1),
                    vector<char>(),
                    DrawMatchesFlags::DEFAULT);

        QImage vis(img_matches.data,img_matches.cols,img_matches.rows,QImage::Format_RGB888);
        m_alignment->show_image(vis);
        m_alignment->repaint();

        // create ransac object
        CAlignRansac alignment(x0,x1);

        // optimize
        size_t ninliers = 0;
        Mat F = alignment.RunConcensus(nsamples,athresh,ninliers,this);

        // bring alignment vis back
        m_alignment->show();
        m_alignment->activateWindow();
        m_alignment->raise();

        // store transformation
        m_trafo_storage[index] = F;

        // show inlier/outlier ratio
        double ioratio = (double)ninliers/(double)good_matches.size();
        ioratio *= 100;
        stringstream ss;
        ss << "The inlier ratio is " << ioratio << "\%.";
        ui->statusBar->showMessage(ss.str().c_str());

        // refine by icp
        refine_alignement(index);

    }

    // close alignment window (hide?)
    m_alignment->hide();

    // if checked, estimate the world coordinate system
    //if(m_params->center_wc())
    //    m_trafo_storage[0] = estimate_world_frame();

}

void MainWindow::on_actionPreferences_triggered() {

    m_params->show();

}

void MainWindow::on_recordButton_clicked(bool checked) {

    if(checked) {

        m_timer.stop();

        QString filename = QFileDialog::getSaveFileName(this, tr("Save file..."),
                                                        ".",
                                                        tr("*.oni"));

        if(filename.isEmpty()) {

            ui->recordButton->setChecked(false);
            ui->recordButton->setText("Record");

            return;
        }


        //stringstream no;
        //no.fill('0');
        //no.width(8);
        //no << rand();

        //string filename = string("/home/jbalzer/Dump/")+no.str()+string(".oni");
        //string filename = string("/home/jbalzer/Dump/raw.oni"); //+no.str()+string(".oni");

        m_timer.start();

        bool error = m_sensor.StartRecording(filename.toStdString().c_str());
        //bool error = m_sensor.StartRecording(filename.c_str());

        if(error) {

            ui->statusBar->showMessage("Could not start capture.");
            ui->recordButton->setChecked(false);

        }
        else
            ui->recordButton->setText("Stop");

    }
    else {  // recording in progress

        ui->recordButton->setText("Record");
        m_sensor.StopRecording();
        return;

    }

}


void MainWindow::on_saveParams_clicked() {

    QString filename = QFileDialog::getSaveFileName(this, tr("Save file..."),".",tr("*.txt"));

    ofstream out(filename.toStdString().c_str());

    if(!out.is_open())
        return;

    out << m_sensor;

    out.close();

}

void MainWindow::refine_alignement(size_t index) {

    // check whether there is something to do
    if(index==m_rgb_storage.size() || m_rgb_storage.size()<2)
        return;

    float zmax = m_sensor.DisparityToDepth(ui->depthclipSlider->sliderPosition());

    vector<Point3f> x0, n0, x1, n1;
    vector<Vec3b> c0, c1;

    get_oriented_pcl(index-1,x0,n0,c0,zmax);     // one-based index, identity trafo
    get_oriented_pcl(index,x1,n1,c1,zmax);
    c0.clear();                                  // clear stuff we don't need here
    c1.clear();

    // create icp object and iterate
    CPointToPlaneICP icp(x0,n0,x1,m_trafo_storage[index]);
    icp.Iterate(m_params->get_no_icp_steps());

    // set result
    m_trafo_storage[index] = icp.GetResult();

}

