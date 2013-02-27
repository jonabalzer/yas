#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calignransac.h"
#include <sstream>
#include <fstream>
#include <QFileDialog>
#include <QProgressDialog>
#include <QMessageBox>
#include <algorithm>


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_cap(CV_CAP_OPENNI),
    m_timer(this),
    m_rgb(Size(640,480),CV_8UC3,Scalar(0)),
    //m_depth(Size(640,480),CV_16UC1),
    m_rgb_storage(),
    m_depth_storage(),
    m_trafo_storage(),
    m_zmax(6000),
    m_depth_buffer(5),
    m_viewer(new ViewerWindow),
    m_alignment(new AlignWindow)
{
    ui->setupUi(this);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_stepButton_clicked()));
    ui->labelDepth->setScaledContents(true);
    ui->labelRGB->setScaledContents(true);
    this->setFixedSize(this->width(),this->height());

    if(!m_cap.isOpened())
        QMessageBox::critical(this,"Error","Could not open source. Make sure the Kinect sensor is connected to your computer and drivers are working properly.");

    connect(this,SIGNAL(current_image_changed(Mat&,Mat&)),this,SLOT(updata_static_view(Mat&,Mat&)));
    connect(this,SIGNAL(current_image_changed(Mat&,Mat&)),m_viewer,SLOT(on_current_image_changed(Mat&,Mat&)));

}

MainWindow::~MainWindow()
{
    m_cap.release();
    delete ui;
    delete m_alignment;
}

bool MainWindow::on_stepButton_clicked()
{

    if(m_cap.grab()) {

        ui->statusBar->clearMessage();

        m_cap.retrieve(m_rgb, CV_CAP_OPENNI_BGR_IMAGE );
        Mat depth;
        m_cap.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP );      // get rid of this!
        m_depth_buffer.push_back(depth.clone());

        // update visualization
        update_live_view();

    } else {
        ui->statusBar->showMessage("Capture failed.");
        m_timer.stop();
        return 1;
    }

    return 0;

}

void MainWindow::on_runButton_clicked()
{
    if(m_cap.isOpened())
        m_timer.start(1);
    else
        QMessageBox::critical(this,"Error","Could not open source. Make sure the Kinect sensor is connected to your computer and drivers are working properly.");

}

void MainWindow::on_pauseButton_clicked()
{
    m_timer.stop();

}

void MainWindow::on_saveButton_clicked()
{

    // because of dialogue
    m_timer.stop();

    size_t index = (size_t)ui->spinBoxStorage->value();

    if(index>0)
        index--;
    else return;

    QString filename = QFileDialog::getSaveFileName(this, tr("Save file..."),
                               ".",
                               tr("(*.png *.exr *.ply)"));

    bool error = false;

    if(filename.endsWith(".png"))
        error = save_as_png(index,filename);
    else if(filename.endsWith(".exr"))
        error = save_as_exr(index,filename);
    else if (filename.endsWith(".ply"))
        error = save_as_ply(index,filename);
    else
        error = 1;

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

//    m_timer.start();

}


void MainWindow::on_saveAllButton_clicked()
{

    m_timer.stop();

    QString dirname = QFileDialog::getExistingDirectory(this,tr("Choose folder..."), ".");

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

        string prefix = dirname.toStdString() + string("/img") + ss.str();


        switch(ui->formatBox->currentIndex()) {

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
            error = save_as_ply(i,QString((prefix+string(".ply")).c_str()));

            break;
        }

        }

    }

    progress.setValue(m_rgb_storage.size());

    if(error)
        QMessageBox::warning(this,"Error","Could not write to disk");

    //m_timer.start();

}

bool MainWindow::save_as_png(size_t index, QString fn) {

    imwrite(fn.toStdString().c_str(),m_rgb_storage[index]);

    return 0;

}
bool MainWindow::save_as_exr(size_t index, QString fn) {

    Array2D<Rgba> out(m_rgb.rows,m_rgb.cols);

    update_zmax();

    for(size_t i=0; i<(size_t)m_rgb.rows; i++) {

        for(size_t j=0; j<(size_t)m_rgb.cols; j++) {

            Rgba val;
            val.b = half(m_rgb_storage[index].at<Vec3b>(i,j)[0]);
            val.g = half(m_rgb_storage[index].at<Vec3b>(i,j)[1]);
            val.r = half(m_rgb_storage[index].at<Vec3b>(i,j)[2]);

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


bool  MainWindow::save_as_ply(size_t index, QString fn) {

    // get transformation to first frame, if it exists
    Mat F = transforms_to_first_image(index);
    Mat Fsr = F(Range(0,3),Range(0,4));

    ofstream out(fn.toStdString().c_str());

    if(!out)
        return 1;

    vector<size_t> indices;

    double variance = ui->triEdit->text().toDouble();

    for(size_t i=0; i<(size_t)m_depth_storage[index].rows-1; i++) {

        for(size_t j=0; j<(size_t)m_depth_storage[index].cols-1; j++) {

            double zq[4];
            zq[0] = (double)m_depth_storage[index].at<unsigned short>(i,j);
            zq[1] = (double)m_depth_storage[index].at<unsigned short>(i+1,j);
            zq[2] = (double)m_depth_storage[index].at<unsigned short>(i,j+1);
            zq[3] = (double)m_depth_storage[index].at<unsigned short>(i+1,j+1);

            if(zq[0]<m_zmax && zq[1]<m_zmax && zq[2]<m_zmax && zq[3]<m_zmax) {

                double mean = 0.25*(zq[0] + zq[1] + zq[2] + zq[3]);

                double var = 0;
                for(size_t k=0; k<4; k++)
                    var += (zq[k]-mean)*(zq[k]-mean);
                var = sqrt(var);

                if(var<variance)
                    indices.push_back(i*m_depth_storage[index].cols+j);

            }

        }

    }

    out << "ply" << endl;
    out <<  "format ascii 1.0" << endl;
    out <<  "comment" << endl;
    out << "element vertex " << m_depth_storage[index].rows*m_depth_storage[index].cols << endl;
    out << "property float32 x" << endl;
    out << "property float32 y" << endl;
    out << "property float32 z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "element face " << indices.size() << endl;
    out << "property list uchar int vertex_index" << endl;
    out << "end_header" << endl;

    double fu, fv, cu, cv;
    fu = ui->fuEdit->text().toDouble();
    fv = ui->fvEdit->text().toDouble();
    cu = ui->cuEdit->text().toDouble();
    cv = ui->cvEdit->text().toDouble();

    update_zmax();

    for(size_t i=0; i<(size_t)m_depth_storage[index].rows; i++) {

        for(size_t j=0; j<(size_t)m_depth_storage[index].cols; j++) {

            vector<Point3d> xarray;
            Point3d x;

            double z = (double)m_depth_storage[index].at<unsigned short>(i,j);

            if(z<m_zmax) {

                x.x = (z/fu)*((double)j-cu);
                x.y = (z/fv)*((double)i-cv);
                x.z = z;
                xarray.push_back(x);
                cv::transform(xarray,xarray,Fsr);

            }
            else {

                x.x = 0;
                x.y = 0;
                x.z = 0;
                xarray.push_back(x);

            }

            out << xarray[0].x << " " << xarray[0].y << " " << xarray[0].z << " " << (unsigned int)m_rgb_storage[index].at<Vec3b>(i,j)[2] << " " << (unsigned int)m_rgb_storage[index].at<Vec3b>(i,j)[1] << " " << (unsigned int)m_rgb_storage[index].at<Vec3b>(i,j)[0] << endl;

        }

    }

    // write faces (maybe as triangles)
    for(size_t k=0; k<indices.size(); k++)
        out << 4 << " " << indices[k] << " " << indices[k]+m_depth_storage[index].cols << " " << indices[k]+m_depth_storage[index].cols+1 << " " << indices[k]+1 << endl;

    out.close();

    return 0;

}

void MainWindow::on_actionUpdateClipDepth_triggered()
{

     m_zmax = (double)ui->depthclipSlider->sliderPosition()*(6000.0/99.0);

    stringstream ss;
    size_t zmax = (size_t)m_zmax;
    ss << zmax;

    ui->maxDepth->setText(QString(ss.str().c_str()));

    int index = ui->spinBoxStorage->value();

    if(index>0)
        emit current_image_changed(m_rgb_storage[index-1],m_depth_storage[index-1]);

}

void MainWindow::update_zmax() {

    m_zmax = (double)ui->depthclipSlider->sliderPosition()*(6000.0/99.0);

}

unsigned short MainWindow::get_smoothed_depth(size_t i, size_t j) {

    size_t s = (double)m_depth_buffer.size();

    unsigned short vals[s];

    for(size_t k=0; k<s; k++)
        vals[k] = m_depth_buffer[k].at<unsigned short>(i,j);

    sort(vals,vals+s);

    if(s%2==1)
        return vals[(size_t)((s+1)/2)-1];
    else
        return 0.5*(vals[s/2-1]+vals[s/2]);

   //return (double)m_depth_buffer.back().at<unsigned short>(i,j);

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

   //m_viewer->



   //m_viewer->repaint();

}

void MainWindow::on_loadButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Open file..."),
                                                    ".",
                                                    tr("*.exr"));

    if(filename.isEmpty())
        return;

    RgbaInputFile file(filename.toStdString().c_str());

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

            rgb.at<Vec3b>(i,j)[0] = in[i][j].b;
            rgb.at<Vec3b>(i,j)[1] = in[i][j].g;
            rgb.at<Vec3b>(i,j)[2] = in[i][j].r;
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


    // get cam parameters
    float fu, fv, cu, cv;
    size_t nfeat, noct, ninliers, nosamples, nmatches;
    double pthresh, ethresh, goodfeatures, acceptthreshold;
    nfeat = ui->noFeatEdit->text().toInt();
    noct = ui->noOctavesEdit->text().toInt();
    pthresh = ui->pointThresholdEdit->text().toDouble();
    ethresh = ui->edgeThresholdEdit->text().toDouble();
    goodfeatures = ui->goodMatchEdit->text().toDouble();
    nosamples = (size_t)ui->nosamplesEdit->text().toInt();
    acceptthreshold = ui->acceptanceEdit->text().toDouble();

    fu = ui->fuEdit->text().toFloat();
    fv = ui->fvEdit->text().toFloat();
    cu = ui->cuEdit->text().toFloat();
    cv = ui->cvEdit->text().toFloat();

    // create ransac object
    CAlignRansac alignment(fu,fv,cu,cv);

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
    m_alignment->show_image(cvis.rgbSwapped());
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


Mat MainWindow::transforms_to_first_image(size_t index){

    Mat F = Mat::eye(4,4,CV_32FC1);

    if(index==0 || index>=m_trafo_storage.size())
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

    case Qt::Key_L:

        on_loadButton_clicked();

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

        QImage black(ui->labeRGBStorage->size(),QImage::Format_RGB888);
        black.fill(qRgb(0,0,0));

        ui->labeRGBStorage->setPixmap(QPixmap::fromImage(black));
        ui->labelDepthStorage->setPixmap(QPixmap::fromImage(black));

        ui->spinBoxStorage->setMinimum(0);
    }

}


void MainWindow::update_live_view() {

    // visualize
    QImage cimg(m_rgb.data,m_rgb.cols,m_rgb.rows,QImage::Format_RGB888);
    ui->labelRGB->setPixmap(QPixmap::fromImage(cimg.rgbSwapped()));

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
    ui->labeRGBStorage->setPixmap(QPixmap::fromImage(cimg.rgbSwapped()));

    Mat depthf  (Size(640,480),CV_8UC1); // exchange this with member function
    depth.convertTo(depthf, CV_8UC1, 255.0/6000.0);

    Mat depthrgb;
    cvtColor(depthf, depthrgb, CV_GRAY2BGR);

    QImage imgdd(depthrgb.data,depthf.cols,depthf.rows,QImage::Format_RGB888);

    double variance = ui->triEdit->text().toDouble();

    for(size_t i=0; i<(size_t)depth.rows-1; i++) {

        for(size_t j=0; j<(size_t)depth.cols-1; j++) {

            double zq[4];
            zq[0] = (double)depth.at<unsigned short>(i,j);
            zq[1] = (double)depth.at<unsigned short>(i+1,j);
            zq[2] = (double)depth.at<unsigned short>(i,j+1);
            zq[3] = (double)depth.at<unsigned short>(i+1,j+1);

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

    ui->labelDepthStorage->setPixmap(QPixmap::fromImage(imgdd));

}
