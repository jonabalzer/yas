#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calignransac.h"
#include "cplaneransac.h"
#include "icp.h"

#include <sstream>
#include <fstream>
#include <algorithm>
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <omp.h>


#include "poisson/MarchingCubes.h"
#include "poisson/Octree.h"
#include "poisson/SparseMatrix.h"
#include "poisson/PPolynomial.h"
#include "poisson/MultiGridOctreeData.h"

#include <QFileDialog>
#include <QProgressDialog>
#include <QMessageBox>

#include <opencv2/nonfree/nonfree.hpp>

using namespace std;
using namespace PoissonRec;
using namespace cv;

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
    m_params(new Params()),
    m_mesh()
{

    ui->setupUi(this);
    ui->labelDepth->setScaledContents(true);
    ui->labelRGB->setScaledContents(true);
    this->setFixedSize(this->width(),this->height());
    ui->spinBoxStorage->setFocusPolicy(Qt::NoFocus);

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_stepButton_clicked()));
    connect(this,SIGNAL(current_image_changed(cv::Mat&,cv::Mat&)),this,SLOT(update_static_view(cv::Mat&,cv::Mat&)));
    connect(this,SIGNAL(current_pcl_changed(std::vector<cv::Point3f>,std::vector<cv::Vec3b>)),m_glview,SLOT(set_pcl(std::vector<cv::Point3f>,std::vector<cv::Vec3b>)));
    connect(this,SIGNAL(current_mesh_changed(PoissonRec::CoredVectorMeshData<PoissonRec::PlyVertex<float> >)),m_glview,SLOT(set_mesh(PoissonRec::CoredVectorMeshData<PoissonRec::PlyVertex<float> >)));
    connect(m_params,SIGNAL(cam_params_changed(CCam,CDepthCam)),this,SLOT(configure_sensor(CCam,CDepthCam)));
    connect(m_params,SIGNAL(cam_params_changed(CCam,CDepthCam)),m_glview,SLOT(configure_cam(CCam,CDepthCam)));
    connect(m_params,SIGNAL(save_params_clicked()),this,SLOT(on_saveParams_clicked()));

    // load sensor data
    m_params->on_applyButton_clicked();

    // get min/max disparity (!) to set up slider
    size_t dmin, dmax;
    m_sensor.GetDisparityRange(dmin,dmax);
    ui->depthClipSlider->setMinimum(dmax);      // the slider refers to depth!
    ui->depthClipSlider->setMaximum(dmin);
    ui->depthClipSlider->setSliderPosition(dmin);
    ui->minDepthClipSlider->setMinimum(dmax);      // the slider refers to depth!
    ui->minDepthClipSlider->setMaximum(dmin);
    ui->minDepthClipSlider->setSliderPosition(dmax);

    float zmax = m_sensor.DisparityToDepth(dmin);
    QString label;
    label.setNum(zmax,'f',2);
    ui->maxDepth->setText(label);
    float zmin = m_sensor.DisparityToDepth(dmax);
    label.setNum(zmin,'f',2);
    ui->minDepth->setText(label);

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

    size_t dmin, dmax;
    m_sensor.GetDisparityRange(dmin,dmax);
    ui->depthClipSlider->setMinimum(dmax);      // the slider refers to depth!
    ui->depthClipSlider->setMaximum(dmin);
    ui->depthClipSlider->setSliderPosition(dmin);
    ui->minDepthClipSlider->setMinimum(dmax);      // the slider refers to depth!
    ui->minDepthClipSlider->setMaximum(dmin);
    ui->minDepthClipSlider->setSliderPosition(dmax);

    float zmax = m_sensor.DisparityToDepth(dmin);
    QString label;
    label.setNum(zmax,'f',2);
    ui->maxDepth->setText(label);
    float zmin = m_sensor.DisparityToDepth(dmax);
    label.setNum(zmin,'f',2);
    ui->minDepth->setText(label);

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

            unsigned short d = m_depth_storage[index].at<unsigned short>(i,j);

            if(warp)
                 val.a = half(wdepth.at<float>(i,j));
            else {

                if(m_params->save_depth()) {

                    float depth = m_sensor.DisparityToDepth(d);

                    val.a = half(depth);

                }
                else
                    val.a = half(d);

            }

            out[i][j] = val;

        }

    }

    Header header(m_rgb_storage[index].cols,m_rgb_storage[index].rows);
    header.insert ("comments", StringAttribute ("YAS"));

    Mat Fcv;

    // only save trafo if we store raw data!!!
    if(!warp)
        Fcv = m_trafo_storage.at(index);
    else
        Fcv = Mat::eye(4,4,CV_32FC1);

    Matrix44<float> Fexr;

    // copy into open exr format
    for(u_int i=0; i<4; i++) {

        for(u_int j=0; j<4; j++)
            Fexr[i][j] = Fcv.at<float>(i,j);

    }

    // attach to header
    header.insert ("view", M44fAttribute (Fexr));

    // create and write file
    RgbaOutputFile file(fn.toStdString().c_str(),header, WRITE_RGBA);
    file.setFrameBuffer (&out[0][0],1,m_rgb_storage[index].cols);
    file.writePixels (m_rgb_storage[index].rows);

    return 0;

}

bool MainWindow::save_normal_map(size_t index, QString fn) {

    /* note that if the transformation is stored for external normal field
     * integration, it has to be w.r.t. to the depth cam coordinate system
     */
    Mat F = transform_to_first_image(index);
    float maxz = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());
    float minz = m_sensor.DisparityToDepth(ui->minDepthClipSlider->sliderPosition());

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

    CDenseArray<vec3> nf(m_depth_storage[index].rows,m_depth_storage[index].cols);

    for(size_t i=0; i<nf.NRows(); i++) {

        for(size_t j=0; j<nf.NCols(); j++) {

            // only do something disparity is unsaturated
            if(m_depth_storage[index].at<unsigned short>(i,j)<=ui->depthClipSlider->maximum()) {

                x = m_sensor.GetPoint(i,j,m_depth_storage[index]);

                if(x.z<maxz && x.z>minz) {

                    // transform normal
                    narray[0] = m_sensor.GetNormal(i,j,m_depth_storage[index]);

                    if(!isfid)
                        cv::transform(narray,narray,R);

                    if(cv::norm(narray[0])>0) {

                        vec3 nr4r = { narray[0].x, narray[0].y, narray[0].z };

                        nf(i,j) = nr4r;

                    }

                }

            }

        }

    }

    return nf.WriteToFile(fn.toStdString().c_str());

}

bool MainWindow::save_trafo(size_t index, QString fn) {

    ofstream out(fn.toStdString().c_str());

    if(!out)
        return 1;

    // use a copy of the cam to write frame into
    if(m_params->warp_to_rgb()) {

        CCam cam(m_sensor.GetRGBCam());

        Mat& F = cam.GetExtrinsics();

        // depth to rgb
        Mat Fd2r = F.clone();

        // depth to world
        Mat Fc2w = transform_to_first_image(index);

        // world to depth
        Mat Fw2c = Fc2w.inv();

        F = Fd2r*Fw2c;

        out << cam << endl;

    }
    else {

        CDepthCam cam(m_sensor.GetDepthCam());

        Mat& F = cam.GetExtrinsics();

        // depth to world
        Mat Fc2w = transform_to_first_image(index);

        // world to depth
        Mat Fw2c = Fc2w.inv();

        F = Fw2c;

        out << cam << endl;

    }

    out.close();

    return 0;

}

bool MainWindow::save_pcl_as_ply(size_t index, QString fn) {

    // get transformation to first frame, if it exists
    Mat F = transform_to_first_image(index);
    vector<Point3f> points;
    vector<Point3f> normals;
    vector<Vec3b> colors;

    float zmax = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());
    float zmin = m_sensor.DisparityToDepth(ui->minDepthClipSlider->sliderPosition());

    get_oriented_pcl(index,points,normals,colors,zmin,zmax,F);

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

bool MainWindow::save_as_pgm(size_t index, QString fn) {

    vector<int> params;
    params.push_back(CV_IMWRITE_PXM_BINARY);
    params.push_back(1);

    return !imwrite(fn.toStdString().c_str(),m_depth_storage[index],params);

}

void MainWindow::on_actionUpdateClipDepth_triggered()
{

    int dmax = ui->depthClipSlider->sliderPosition();
    float zmax = m_sensor.DisparityToDepth(dmax);
    int dmin = ui->minDepthClipSlider->sliderPosition();
    float zmin = m_sensor.DisparityToDepth(dmin);

    QString label;
    label.setNum(zmax,'f',2);
    ui->maxDepth->setText(label);
    label.setNum(zmin,'f',2);
    ui->minDepth->setText(label);

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

    m_glview->clear_data();
    m_glview->show();

    int val = ui->spinBoxStorage->value();

    std::cout << "[MainWindow::on_action3D_View_triggered]" << std::endl;
    emit current_mesh_changed(m_mesh);

    if(val>0)
        on_spinBoxStorage_valueChanged(val);

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

        //if(!m_glview->isHidden()) {

            Mat F = transform_to_first_image(arg1-1);
            vector<Point3f> points;
            vector<Vec3b> colors;

            float zmax = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());

            get_pcl(arg1-1,points,colors,zmax,F);

            emit current_pcl_changed(points,colors);

        //}

    }

}

Mat MainWindow::transform_to_first_image(size_t index){

    Mat F = m_trafo_storage[0].clone();

    if(index==0)
        return F;

    for(size_t i=1; i<=index; i++) {

        F = F*m_trafo_storage[i];

    }

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

        // update image display
        emit current_image_changed(m_rgb_storage[index],m_depth_storage[index]);

        // update opengl viewer
        Mat F = transform_to_first_image(index);
        vector<Point3f> points;
        vector<Vec3b> colors;

        float zmax = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());

        get_pcl(index,points,colors,zmax,F);

        emit current_pcl_changed(points,colors);


    }
    else {

        ui->labeRGBStorage->clear();
        ui->labelDepthStorage->clear();
        ui->spinBoxStorage->setMinimum(0);
        m_glview->clear_data();

    }


}


void MainWindow::update_live_view() {

    float dmax = (float)ui->depthClipSlider->sliderPosition();
    float dmin = (float)ui->depthClipSlider->minimum();

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

    float dmax = (float)ui->depthClipSlider->sliderPosition();
    float dmin = (float)ui->minDepthClipSlider->sliderPosition();

    if(dmin>=dmax)
        dmin = (float)ui->depthClipSlider->minimum();
    float dnorm = (float)ui->depthClipSlider->maximum() - dmin;

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
      QMessageBox::information(this,"About","(c) J. Balzer, T. Moerwald. \n More info on YAS: http://vision.ucla.edu");
}


Mat MainWindow::estimate_world_frame() {

    Vec3f mean, ex, ez;

    // get point cloud of first view
    vector<Point3f> pcl;
    vector<Vec3b> colors;
    float zmax = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());
    get_pcl(0,pcl,colors,zmax,Mat::eye(4,4,CV_32FC1));

    // get alignment params
    size_t nfeat, noctaves, nsamples;
    double pthresh, ethresh, ratio, athresh;
    m_params->get_alignment_parameters(nfeat,noctaves,pthresh,ethresh,ratio,nsamples,athresh);

    // estimate plane parameters w.r.t. Fw1
    CEstimatePlaneRansac pransac(pcl);
    size_t ninliers;
    Vec4f plane = pransac.RunConsensus(nsamples,athresh,ninliers,this);

    cout << plane[0] << " " << plane[1] << " " << plane[2] << " " << plane[3] << endl;

    // flip normal
    if(plane[2]<0)
        plane = -plane;

    // show inlier/outlier ratio
    double ioratio = (double)ninliers/(double)pcl.size();
    ioratio *= 100;
    stringstream ss;
    ss << "The inlier ratio for plane estimation is " << ioratio << "\%.";
    ui->statusBar->showMessage(ss.str().c_str());

    // build frame from world 2 -> world 1
    Mat Fw2 = Mat::eye(4,4,CV_32FC1);

    // normal of plane gives news ez axis
    for(size_t i=0; i<3; i++)
        ez[i] = plane[i];

    // project origin on plane
    mean = ez*plane[3];

    // project (1,0,0) onto plane
    ex *= 0;
    ex[0] = 1;
    ex = ex - ez[0]*ez;
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
    return Fw2inv; //*Fw1inv;

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
            if(m_depth_storage[index].at<unsigned short>(i,j)<=ui->depthClipSlider->maximum()) {

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

void MainWindow::get_oriented_pcl(size_t index, vector<Point3f>& vertices, vector<Point3f>& normals, vector<Vec3b>& colors, float minr, float maxr, Mat F) {

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
            if(m_depth_storage[index].at<unsigned short>(i,j)<=ui->depthClipSlider->maximum()) {

                x = m_sensor.GetPoint(i,j,m_depth_storage[index]);

                if(x.z<maxr && x.z>minr) {

                    xarray[0] = x;

                    // transform if F is not the idendity
                    if(!isfid)
                        cv::transform(xarray,xarray,Fsr);

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

        XForm4x4<float> id = XForm4x4<float>::Identity();

        if(m_params->ply_binary())
            PoissonRec::PlyWritePolygons(filename.toStdString().c_str(),&m_mesh,PLY_BINARY_NATIVE,nullptr,0,id);
        else
            PoissonRec::PlyWritePolygons(filename.toStdString().c_str(),&m_mesh,PLY_ASCII,nullptr,0,id);

    }
    else if(filename.endsWith(".pgm"))
        error = save_as_pgm(index,filename);
    else if(filename.endsWith(".r4r"))
        error = save_normal_map(index,filename);
    else
        error = 0;

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

}

void MainWindow::on_actionOpen_triggered()
{

    m_timer.stop();

    QStringList filenames = QFileDialog::getOpenFileNames(this,tr("Open file..."),".",tr("*.exr"));

    if(filenames.size()==0)
        return;

    for(size_t i=0; i<filenames.size(); i++) {

        RgbaInputFile file(filenames[i].toStdString().c_str());

        // get transformation attribute
        const M44fAttribute* Fexra = file.header().findTypedAttribute <M44fAttribute> ("view");

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
        Mat Fcv = Mat::eye(4,4,CV_32FC1);

        if(Fexra!=0) {

            Matrix44<float> Fexr = Fexra->value();

            for(u_int i=0; i<4; i++) {

                for(u_int j=0; j<4; j++)
                    Fcv.at<float>(i,j) = Fexr[i][j];

            }

        }

        m_trafo_storage.push_back(Fcv);

    }

     // adjust counter
    ui->spinBoxStorage->setMaximum(m_rgb_storage.size());

    if(m_rgb_storage.size()>=1)
        ui->spinBoxStorage->setMinimum(1);

    ui->spinBoxStorage->setValue(m_rgb_storage.size());

    // display
    emit current_image_changed(m_rgb_storage[m_rgb_storage.size()-1],m_depth_storage[m_depth_storage.size()-1]);


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
    else if(filename.endsWith(".r4r"))
        format = 5;
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

        case 5:
        {

            error = save_normal_map(i,QString((prefix+string(".r4r")).c_str()));

            break;

        }


        }

    }

    progress.setValue(m_rgb_storage.size());

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

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

    float zmax = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());
    float zmin = m_sensor.DisparityToDepth(ui->minDepthClipSlider->sliderPosition());

    vector<Point3f> x0, n0, x1, n1;
    vector<Vec3b> c0, c1;

    get_oriented_pcl(index-1,x0,n0,c0,zmin,zmax);     // one-based index, identity trafo
    get_oriented_pcl(index,x1,n1,c1,zmin,zmax);
    c0.clear();                                  // clear stuff we don't need here
    c1.clear();

    // create icp object and iterate
    CPointToPlaneICP icp(x0,n0,x1,m_trafo_storage[index]);
    icp.Iterate(m_params->get_no_icp_steps());

    // set result
    m_trafo_storage[index] = icp.GetResult();

}


void MainWindow::on_actionPoisson_triggered()
{

  // set parameters
    int Depth, boundaryType, kernelDepth, adaptiveExponent, isoDivide, solverDivide, minDepth, minIters, maxSolveDepth;
    float samplesPerNode, scaleFactor, constraintWeight, accuracy;
    bool addBarycenter,polygonMesh, useConfidence;

    m_params->get_reconstruction_parameters(Depth,constraintWeight,samplesPerNode,scaleFactor,minIters,accuracy,useConfidence,polygonMesh);

    boundaryType = 1;
    kernelDepth = Depth-2;
    addBarycenter = true;
    adaptiveExponent = 1;
    isoDivide = 8;
    minDepth = 5;
    solverDivide = 8;
    maxSolveDepth = Depth;

    XForm4x4<Real> xForm = XForm4x4<Real>::Identity();

    // merge all point clouds
    float zmax = m_sensor.DisparityToDepth(ui->depthClipSlider->sliderPosition());
    float zmin = m_sensor.DisparityToDepth(ui->minDepthClipSlider->sliderPosition());

    vector<cv::Point3f> points, normals;
    vector<Vec3b> colors;

    for(size_t i=0; i<m_depth_storage.size(); i++) {

        Mat F = transform_to_first_image(i);
        get_oriented_pcl(i,points,normals,colors,zmin,zmax,F);
        colors.clear();

    }

    // create octree
    PoissonRec::Octree<2,false> tree = PoissonRec::Octree<2,false>();
    tree.threads = omp_get_num_procs();

    PoissonRec::OctNode< PoissonRec::TreeNodeData<false> , Real >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );

    // init finite elements
    tree.setBSplineData( Depth , boundaryType );

    tree.setTree( points,
                  normals,
                  Depth,
                  minDepth,
                  kernelDepth,
                  samplesPerNode,
                  scaleFactor,
                  useConfidence,
                  constraintWeight,
                  adaptiveExponent,
                  xForm);

    tree.ClipTree();
    tree.finalize(isoDivide);
    tree.SetLaplacianConstraints();

    // solve linear system
    tree.LaplacianMatrixIteration( solverDivide,
                                   false,
                                   minIters,
                                   accuracy,
                                   maxSolveDepth,
                                   -1);


    // compute iso value
    float isoValue = tree.GetIsoValue();

    // delete old mesh
    m_mesh.reset();

    // marching cubes
    tree.GetMCIsoTriangles( isoValue,
                            isoDivide,
                            &m_mesh,
                            0,
                            1,
                            addBarycenter,
                            polygonMesh );

  ui->statusBar->showMessage("Poisson reconstruction finished...");

//  if(!m_glview->isHidden())
    emit current_mesh_changed(m_mesh);

}

void MainWindow::on_actionCurrent_triggered()
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

    int maxd = ui->depthClipSlider->maximum();
    int mind = ui->depthClipSlider->minimum();

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

    // if there are not enough correspondences return
    if(x0.size()<3) {

        ui->statusBar->showMessage("Insufficient number of correspondences...");
        return;

    }

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

void MainWindow::on_actionAll_triggered()
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

        /*
         * warp the rgb image to the depth image plane: this can be done with higher
         * precision because we only need to compute projections of backprojected pixels
         * (the latter backprojection only being available for the depth sensor). we
         * need the color image to compute putative correspondences based on photometry.
         */
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

        int maxd = ui->depthClipSlider->maximum();
        int mind = ui->depthClipSlider->minimum();

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

        // if there are not enough correspondences return
        if(x0.size()<3) {

            ui->statusBar->showMessage("Insufficient number of correspondences...");
            return;

        }


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
    if(m_params->center_wc())
        m_trafo_storage[0] = estimate_world_frame();

}
