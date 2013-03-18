#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calignransac.h"
#include "cplaneransac.h"

#include <sstream>
#include <fstream>
#include <algorithm>
#include <set>

#include <QFileDialog>
#include <QProgressDialog>
#include <QMessageBox>


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_sensors(),
    m_timer(this),
    m_rgb(Size(640,480),CV_8UC3,Scalar(0)),
    m_depth_buffer(5),
    m_rgb_storage(),
    m_depth_storage(),
    m_trafo_storage(),
    m_zmax(6000),
    m_viewer(new ViewerWindow),
    m_alignment(new AlignWindow)
{
    ui->setupUi(this);

    ui->labelDepth->setScaledContents(true);
    ui->labelRGB->setScaledContents(true);
    this->setFixedSize(this->width(),this->height());

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_stepButton_clicked()));
    connect(this,SIGNAL(current_image_changed(Mat&,Mat&)),this,SLOT(updata_static_view(Mat&,Mat&)));
    connect(this,SIGNAL(current_image_changed(Mat&,Mat&)),m_viewer,SLOT(on_current_image_changed(Mat&,Mat&)));

    if(m_sensors.OpenPrimeSenseModule())
        QMessageBox::critical(this,"Error","Could not open source. Make sure the Kinect sensor is connected to your computer and drivers are working properly.");

}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_alignment;
}

bool MainWindow::on_stepButton_clicked()
{

    Mat depth;

    // get new data (this return zero matrices if something is wrong with the sensor
    m_sensors.Get(m_rgb,depth);

    // push depth into ring buffer
    m_depth_buffer.push_back(depth.clone());

    // update live view
    update_live_view();

    return 0;

}

void MainWindow::on_runButton_clicked()
{
    if(m_sensors.IsSane())
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

    Array2D<Rgba> out(m_rgb.rows,m_rgb.cols);

    for(size_t i=0; i<(size_t)m_rgb.rows; i++) {

        for(size_t j=0; j<(size_t)m_rgb.cols; j++) {

            Rgba val;
            val.r = half(m_rgb_storage[index].at<Vec3b>(i,j)[0]);
            val.g = half(m_rgb_storage[index].at<Vec3b>(i,j)[1]);
            val.b = half(m_rgb_storage[index].at<Vec3b>(i,j)[2]);

            double z = (double)m_depth_storage[index].at<unsigned short>(i,j);

            if(z<m_zmax)
                val.a = half(z);
            else
                val.a = half(0);

            out[i][j] = val;

        }

    }

    RgbaOutputFile file(fn.toStdString().c_str(),m_rgb.cols,m_rgb.rows, WRITE_RGBA);
    file.setFrameBuffer (&out[0][0],1,m_rgb.cols);
    file.writePixels (m_rgb.rows);

    return 0;

}

bool MainWindow::save_pcl_as_ply(size_t index, QString fn) {

    // get transformation to first frame, if it exists
    Mat F = transform_to_first_image(index);
    vector<Point3f> points;
    vector<Vec3b> colors;

    get_pcl(index,points,colors,m_zmax,F);

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
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "end_header" << endl;

    for(size_t i=0; i<points.size(); i++)
        out << points[i].x << " " << points[i].y << " " << points[i].z << " " << (unsigned int)colors[i][0] << " " << (unsigned int)colors[i][1] << " " << (unsigned int)colors[i][2] << endl;

    out.close();

    return 0;

}


bool MainWindow::save_mesh_as_ply(size_t index, QString fn) {

    // get transformation to first frame, if it exists
    Mat F = transform_to_first_image(index);
    vector<Point3f> vertices;
    vector<Vec3b> colors;
    vector<Vec4i> faces;

    get_mesh(index,vertices,colors,faces,m_zmax,F);

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

    return imwrite(fn.toStdString().c_str(),m_depth_storage[index]);

}

void MainWindow::on_actionUpdateClipDepth_triggered()
{

    int slidermax = ui->depthclipSlider->maximum();
    m_zmax = (double)ui->depthclipSlider->sliderPosition()*(10000.0/(float)slidermax);

    stringstream ss;
    ss << (size_t)m_zmax;

    ui->maxDepth->setText(QString(ss.str().c_str()));

    int index = ui->spinBoxStorage->value();

    if(index>0)
        emit current_image_changed(m_rgb_storage[index-1],m_depth_storage[index-1]);

}

unsigned short MainWindow::get_smoothed_depth(size_t i, size_t j) {

    size_t s = (double)m_depth_buffer.size();

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
   m_viewer->show();
   m_viewer->raise();
   m_viewer->activateWindow();


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

    }

}


void MainWindow::on_alignButton_clicked()
{

    // get image index
    size_t index = (size_t)ui->spinBoxStorage->value();

    // check whether there is something to do
    if(index==m_rgb_storage.size() || m_rgb_storage.size()<2)
        return;
        //index--;
    //else return; // there are no images to align

    // stop acqusition to have computational resources
    m_timer.stop();

    // show window
    m_alignment->show();
    m_alignment->activateWindow();
    m_alignment->raise();

    // get parameters
    size_t nfeat, noct, ninliers, nosamples, nmatches;
    double pthresh, ethresh, goodfeatures, acceptthreshold;
    nfeat = ui->noFeatEdit->text().toInt();
    noct = ui->noOctavesEdit->text().toInt();
    pthresh = ui->pointThresholdEdit->text().toDouble();
    ethresh = ui->edgeThresholdEdit->text().toDouble();
    goodfeatures = ui->goodMatchEdit->text().toDouble();
    nosamples = (size_t)ui->nosamplesEdit->text().toInt();
    acceptthreshold = ui->acceptanceEdit->text().toDouble();
    vector<float> f = m_sensors.GetFocalLengths();
    vector<float> c = m_sensors.GetPrincipalPoint();

    // create ransac object
    CAlignRansac alignment(f[0],f[1],c[0],c[1]);

    Mat vis = alignment.GenerateHypotheses(m_rgb_storage[index-1],
                                           m_rgb_storage[index],
                                           m_depth_storage[index-1],
                                           m_depth_storage[index],
                                           nfeat,
                                           noct,
                                           pthresh,
                                           ethresh,
                                           goodfeatures,
                                           nmatches);

    QImage cvis(vis.data,vis.cols,vis.rows,QImage::Format_RGB888);
    m_alignment->show_image(cvis);
    m_alignment->repaint();

    // optimize
    Mat F = alignment.RunConcensus(nosamples,acceptthreshold,ninliers,this);

    // bring alignment vis back
    m_alignment->raise();

    // store transformation
    m_trafo_storage[index] = F;

    // show inlier/outlier ratio
    double ratio = (double)ninliers/(double)nmatches;
    ratio *= 100;
    stringstream ss;
    ss << "The inlier ratio is " << ratio << "\%.";
    ui->statusBar->showMessage(ss.str().c_str());


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

    switch(event->key()) {

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

    // visualize
    QImage cimg(m_rgb.data,m_rgb.cols,m_rgb.rows,QImage::Format_RGB888);
    ui->labelRGB->setPixmap(QPixmap::fromImage(cimg));

    Mat img = m_depth_buffer.back();
    Mat depthf  (Size(640,480),CV_8UC1); // exchange this with member function
    img.convertTo(depthf, CV_8UC1, 255.0/6000.0);

    Mat depthrgb;
    cvtColor(depthf, depthrgb, CV_GRAY2BGR);

    //QImage imgdd(depthf.data,depthf.cols,depthf.rows,QImage::Format_Indexed8);
    QImage imgdd(depthrgb.data,depthf.cols,depthf.rows,QImage::Format_RGB888);

    double variance = ui->triEdit->text().toDouble();

    for(size_t i=0; i<(size_t)img.rows-1; i++) {

        for(size_t j=0; j<(size_t)img.cols-1; j++) {

            double zq[4];
            zq[0] = (double)img.at<unsigned short>(i,j);
            zq[1] = (double)img.at<unsigned short>(i+1,j);
            zq[2] = (double)img.at<unsigned short>(i,j+1);
            zq[3] = (double)img.at<unsigned short>(i+1,j+1);

            if(zq[0]>m_zmax)
                imgdd.setPixel(j,i,qRgb(0,0,0));
            else if(zq[0]<m_zmax && zq[1]<m_zmax && zq[2]<m_zmax && zq[3]<m_zmax) {

                double mean = 0.25*(zq[0] + zq[1] + zq[2] + zq[3]);

                double var = 0;
                for(size_t k=0; k<4; k++)
                    var += (zq[k]-mean)*(zq[k]-mean);
                var = sqrt(var);

                if(var>variance)
                    imgdd.setPixel(j,i,qRgb(255,0,0));

            }

        }

    }

    ui->labelDepth->setPixmap(QPixmap::fromImage(imgdd));

}

void MainWindow::updata_static_view(Mat& rgb, Mat& depth) {

    QImage cimg(rgb.data,rgb.cols,rgb.rows,QImage::Format_RGB888);
    ui->labeRGBStorage->setPixmap(QPixmap::fromImage(cimg));

    //Mat depthf  (Size(640,480),CV_8UC1); // exchange this with member function
    //depth.convertTo(depthf, CV_8UC1, 255.0/6000.0);

    //Mat depthrgb;
    //cvtColor(depthf, depthrgb, CV_GRAY2BGR);
    //QImage imgdd(depthrgb.data,depthf.cols,depthf.rows,QImage::Format_RGB888);

    QImage imgdd(depth.cols,depth.rows,QImage::Format_RGB888);

    double variance = ui->triEdit->text().toDouble();

    for(size_t i=0; i<(size_t)depth.rows-1; i++) {

        for(size_t j=0; j<(size_t)depth.cols-1; j++) {

            float z = (float)depth.at<unsigned short>(i,j);

            if(z>m_zmax || z==0)
                imgdd.setPixel(j,i,qRgb(0,0,0));
            else {

                QColor val = QColor::fromHsvF(z/m_zmax,1.0,1.0);
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

    Mat Sigma, U, Vt;
    SVD::compute(T, Sigma, U, Vt);
    Vec3f n, ex;

    for(size_t j=0; j<3; j++) {

        n[j] = Vt.at<float>(2,j);
        ex[j] = T.at<float>(0,j);       // connection to first cam position
    }

    // normalize ez, FIXME: no necessary because of SVD props, check if all frames are set!
    Vec3f ez = n *= (1/cv::norm(n));

    // make ex and n normal
    ex = ex - ez*(ex[0]*ez[0] + ex[1]*ez[1] + ex[2]*ez[2]);
    ex *= (1/cv::norm(ex));

    // fill in frame from world to cam 0
    Mat Fw = Mat::eye(4,4,CV_32FC1);
    for(size_t i=0; i<3; i++) {

        Fw.at<float>(i,0) = ex[i];
        Fw.at<float>(i,2) = ez[i];
        Fw.at<float>(i,3) = mean[i];

    }

    // ey by cross product
    Fw.at<float>(0,1) = ez[1]*ex[2] - ez[2]*ex[1];
    Fw.at<float>(1,1) = ez[2]*ex[0] - ez[0]*ex[2];
    Fw.at<float>(2,1) = ez[0]*ex[1] - ez[1]*ex[0];

    // invert
    Mat Fwinv = Fw.inv();

    // get point cloud of first view, FIXME: set from zmax
//    float maxr = (float)m_zmax;
//    vector<Point3f> pcl;
//    vector<Vec3b> colors;
//    get_pcl(0,pcl,colors,600,Fwinv);

//    CEstimatePlaneRansac pransac(pcl);
//    size_t ninliers;
//    Vec4f plane = pransac.RunConsensus(5000,5,ninliers,this);

//    cout << plane[0] << " " << plane[1] << " " << plane[2] << " " << plane[3] << endl;

//    // the normal must be rotated back to cam 0 coordinates, use old variable
//    for(size_t i=0; i<3; i++) {

//        n[i] = 0;

//        for(size_t j=0; j<3; j++) {

//            n[i] += Fw.at<float>(i,j)*plane[j];

//        }

//    }

//    // project origin on plane
//    mean = mean - n*(mean[0]*n[0]+mean[1]*n[1]+mean[2]*n[2]-plane[3]);

//    // ez is already normal, project ex on plane
//    ez = n;
//    ex = ex - ez*(ex[0]*ez[0] + ex[1]*ez[1] + ex[2]*ez[2]);

//    // update Fw
//    for(size_t i=0; i<3; i++) {

//        Fw.at<float>(i,0) = ex[i];
//        Fw.at<float>(i,2) = ez[i];
//        Fw.at<float>(i,3) = mean[i];

//    }

//    // ey by cross product
//    Fw.at<float>(0,1) = ez[1]*ex[2] - ez[2]*ex[1];
//    Fw.at<float>(1,1) = ez[2]*ex[0] - ez[0]*ex[2];
//    Fw.at<float>(2,1) = ez[0]*ex[1] - ez[1]*ex[0];




    return Fwinv;

}

void MainWindow::get_pcl(size_t index, vector<Point3f>& vertices, vector<Vec3b>& colors, float maxr, Mat F) {

    // prepare transformation
    Mat Fsr = F(Range(0,3),Range(0,4));
    bool isfid = F.at<float>(0,0)==1 && F.at<float>(0,1)==0 && F.at<float>(0,2)==0 && F.at<float>(0,3)==0 &&
                 F.at<float>(1,0)==0 && F.at<float>(1,1)==1 && F.at<float>(1,2)==0 && F.at<float>(1,3)==0 &&
                 F.at<float>(2,0)==0 && F.at<float>(2,1)==0 && F.at<float>(2,2)==1 && F.at<float>(2,3)==0;

    // get camera intrinsics
    vector<float> f = m_sensors.GetFocalLengths();
    vector<float> c = m_sensors.GetPrincipalPoint();

    // allocate space for temporary variable
    vector<Point3f> xarray;
    Point3f x;
    xarray.push_back(x);

    // collect points
    for(size_t i=0; i<(size_t)m_depth_storage[index].rows; i++) {

        for(size_t j=0; j<(size_t)m_depth_storage[index].cols; j++) {

            float z = (float)m_depth_storage[index].at<unsigned short>(i,j);

            // only do something if we have a depth value
            if(z>0) {

                x.x = (z/f[0])*((double)j-c[0]);
                x.y = (z/f[1])*((double)i-c[1]);
                x.z = z;

                // transform if F is not the idendity
                if(!isfid) {

                    xarray[0] = x;
                    cv::transform(xarray,xarray,Fsr);
                    x = xarray[0];

                }

                // check distance condition
                if(cv::norm(x)<maxr) {

                    vertices.push_back(x);
                    colors.push_back(m_rgb_storage[index].at<Vec3b>(i,j));

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

    // get camera intrinsics
    vector<float> f = m_sensors.GetFocalLengths();
    vector<float> c = m_sensors.GetPrincipalPoint();

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

            float z = (float)m_depth_storage[index].at<unsigned short>(i,j);

            // only do something if we have a depth value
            if(z>0) {

                x.x = (z/f[0])*((double)j-c[0]);
                x.y = (z/f[1])*((double)i-c[1]);
                x.z = z;

                // transform if F is not the idendity
                if(!isfid) {

                    xarray[0] = x;
                    cv::transform(xarray,xarray,Fsr);
                    x = xarray[0];

                }

                // check distance condition
                if(cv::norm(x)<maxr) {

                    vertices.push_back(x);
                    colors.push_back(m_rgb_storage[index].at<Vec3b>(i,j));

                    // insert into hash map: linear image index -> selected vertices
                    indexhash.insert(pair<size_t,size_t>(i*m_depth_storage[index].cols+j,vertices.size()-1));
                    depthhash.insert(pair<size_t,float>(i*m_depth_storage[index].cols+j,z));

                }

            }

        }

    }

    // now iterate through index hash
    map<size_t,float>::iterator ittl, ittr, itbr, itbl;
    float variance = ui->triEdit->text().toFloat();

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
                               tr("*.png;;*.exr;;*.ply;;*.pgm"));

    bool error = false;

    if(filename.endsWith(".png"))
        error = save_as_png(index,filename);
    else if(filename.endsWith(".exr"))
        error = save_as_exr(index,filename);
    else if (filename.endsWith(".ply")) {

        if(ui->triangulateCheckBox->isChecked())
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
                               tr("*.png;;*.exr;;*.ply"));

    int format = 0;

    if(filename.endsWith(".png"))
        format = 0;
    else if(filename.endsWith(".exr"))
        format = 1;
    else if (filename.endsWith(".ply"))
        format = 2;
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

            if(ui->triangulateCheckBox->isChecked())
                error = save_mesh_as_ply(i,fn);
            else
                error = save_pcl_as_ply(i,fn);

            break;

        }

        }

    }

    progress.setValue(m_rgb_storage.size());

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

}

void MainWindow::on_triangulateCheckBox_stateChanged(int arg1)
{

    if(arg1==0)
        ui->triEdit->setEnabled(false);
    else
        ui->triEdit->setEnabled(true);

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

    for(size_t i=0; i<m_rgb_storage.size()-1; i++) {

        // get parameters
        size_t nfeat, noct, ninliers, nosamples, nmatches;
        double pthresh, ethresh, goodfeatures, acceptthreshold;
        nfeat = ui->noFeatEdit->text().toInt();
        noct = ui->noOctavesEdit->text().toInt();
        pthresh = ui->pointThresholdEdit->text().toDouble();
        ethresh = ui->edgeThresholdEdit->text().toDouble();
        goodfeatures = ui->goodMatchEdit->text().toDouble();
        nosamples = (size_t)ui->nosamplesEdit->text().toInt();
        acceptthreshold = ui->acceptanceEdit->text().toDouble();
        vector<float> f = m_sensors.GetFocalLengths();
        vector<float> c = m_sensors.GetPrincipalPoint();

        // create ransac object
        CAlignRansac alignment(f[0],f[1],c[0],c[1]);

        Mat vis = alignment.GenerateHypotheses(m_rgb_storage[i],
                                               m_rgb_storage[i+1],
                                               m_depth_storage[i],
                                               m_depth_storage[i+1],
                                               nfeat,
                                               noct,
                                               pthresh,
                                               ethresh,
                                               goodfeatures,
                                               nmatches);

        QImage cvis(vis.data,vis.cols,vis.rows,QImage::Format_RGB888);
        m_alignment->show_image(cvis);
        m_alignment->repaint();
        m_alignment->raise();

        // optimize
        Mat F = alignment.RunConcensus(nosamples,acceptthreshold,ninliers,this);

        // store transformation
        m_trafo_storage[i+1] = F;

        // show inlier/outlier ratio
        double ratio = (double)ninliers/(double)nmatches;
        ratio *= 100;
        stringstream ss;
        ss << "The inlier ratio is " << ratio << "\%.";
        ui->statusBar->showMessage(ss.str().c_str());

        // wait for a second
        QTimer timer;
        timer.setSingleShot(true);
        timer.setInterval(1000);
        timer.start();

    }

    // if checked, estimate the world coordinate system
    if(ui->centerCheckBox->isChecked())
        m_trafo_storage[0] = estimate_world_frame();

}
