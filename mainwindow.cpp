#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sstream>
#include <fstream>
#include <QFileDialog>
#include <algorithm>


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_cap(CV_CAP_OPENNI),
    m_timer(this),
    m_rgb(Size(640,480),CV_8UC3,Scalar(0)),
    m_depth(Size(640,480),CV_16UC1),
    m_zmax(6000),
    m_depths(5)
{
    ui->setupUi(this);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(on_stepButton_clicked()));
    ui->labelDepth->setScaledContents(true);
    ui->labelRGB->setScaledContents(true);

    if(!m_cap.isOpened())
        ui->statusBar->showMessage("ERROR: Could not open source!");

    m_img_counter[0] = 0;
    m_img_counter[1] = 0;
    m_img_counter[2] = 0;
}

MainWindow::~MainWindow()
{
    m_cap.release();
    delete ui;
}

void MainWindow::on_stepButton_clicked()
{

    if(m_cap.grab()) {

        ui->statusBar->clearMessage();

        m_cap.retrieve(m_rgb, CV_CAP_OPENNI_BGR_IMAGE );
        m_cap.retrieve(m_depth, CV_CAP_OPENNI_DEPTH_MAP );

        m_depths.push_back(m_depth.clone());

        QImage imgd(m_rgb.data,m_rgb.cols,m_rgb.rows,QImage::Format_RGB888);
        ui->labelRGB->setPixmap(QPixmap::fromImage(imgd.rgbSwapped()));

        Mat depthf  (Size(640,480),CV_8UC1);
        m_depth.convertTo(depthf, CV_8UC1, 255.0/6000.0);
        //m_depths.back().convertTo(depthf, CV_8UC1, 255.0/6000.0);

        QImage imgdd(depthf.data,depthf.cols,depthf.rows,QImage::Format_Indexed8);


        for(size_t i=0; i<(size_t)m_depth.rows; i++) {

            uchar* lpt = imgdd.scanLine(i);

            for(size_t j=0; j<(size_t)m_depth.cols; j++) {

                if((double)m_depth.at<unsigned short>(i,j)>m_zmax)
                    lpt[j] = 0;
                    //imgdd.setPixel(j,i,0);

            }
        }


        ui->labelDepth->setPixmap(QPixmap::fromImage(imgdd));



    } else {
        ui->statusBar->showMessage("Capture failed.");
        m_timer.stop();
    }

}

void MainWindow::on_runButton_clicked()
{
    m_timer.start(1);
}

void MainWindow::on_pauseButton_clicked()
{
    m_timer.stop();
}

void MainWindow::on_saveButton_clicked()
{

    // stop timer to make sure that the buffer does not get flushed
    m_timer.stop();
    on_stepButton_clicked();

    stringstream ss;
    ss.fill('0');
    ss.width(4);

    switch(ui->formatBox->currentIndex()) {

    case 0:
    {
        ss << m_img_counter[0];
        string filename = ui->dirEdit->text().toStdString() + "/img" + ss.str() + ".png";
        m_img_counter[0]++;

         if(imwrite(filename.c_str(),m_rgb))
            ui->statusBar->showMessage("Could not write file.");

         break;

    }
    case 1:
    {

        ss << m_img_counter[1];
        string filename = ui->dirEdit->text().toStdString() + "/img" + ss.str() + ".exr";
        m_img_counter[1]++;

        Array2D<Rgba> out(m_depth.rows,m_depth.cols);

        update_zmax();

        for(size_t i=0; i<(size_t)m_depth.rows; i++) {

            for(size_t j=0; j<(size_t)m_depth.cols; j++) {

                Rgba val;
                val.b = half(m_rgb.at<Vec3b>(i,j)[0]);
                val.g = half(m_rgb.at<Vec3b>(i,j)[1]);
                val.r = half(m_rgb.at<Vec3b>(i,j)[2]);
                //val.a = half(m_depth.at<unsigned short>(i,j));      // depth map in two bytes
                half z = half(get_smoothed_depth(i,j));

                if(z<m_zmax)
                    val.a = z;
                else
                    val.a = 0;

                out[i][j] = val;


            }

        }

        RgbaOutputFile file(filename.c_str(),m_depth.cols,m_depth.rows, WRITE_RGBA);
        file.setFrameBuffer (&out[0][0],1,m_depth.cols);
        file.writePixels (m_depth.rows);

        break;

    }

    case 2:
    {

        ss << m_img_counter[2];
        string filename = ui->dirEdit->text().toStdString() + "/img" + ss.str() + ".ply";
        m_img_counter[2]++;

        if(save_as_ply(filename))
            ui->statusBar->showMessage("Could not write file.");

    }
    default:



        break;

    }

    m_timer.start(1);

}


bool  MainWindow::save_as_ply(string fn) {

    ofstream out(fn.c_str());

    if(!out)
        return 1;

    vector<size_t> indices;

    double variance = ui->triEdit->text().toDouble();

    for(size_t i=0; i<(size_t)m_depth.rows-1; i++) {

        for(size_t j=0; j<(size_t)m_depth.cols-1; j++) {

            double zq[4];
            zq[0] = get_smoothed_depth(i,j); //(double)m_depth.at<unsigned short>(i,j);
            zq[1] = get_smoothed_depth(i+1,j); //(double)m_depth.at<unsigned short>(i+1,j);
            zq[2] = get_smoothed_depth(i,j+1); //(double)m_depth.at<unsigned short>(i,j+1);
            zq[3] = get_smoothed_depth(i+1,j+1); //(double)m_depth.at<unsigned short>(i+1,j+1);

            if(zq[0]<m_zmax && zq[1]<m_zmax && zq[2]<m_zmax && zq[3]<m_zmax) {

                double mean = 0.25*(zq[0] + zq[1] + zq[2] + zq[3]);

                double var = 0;
                for(size_t k=0; k<4; k++)
                    var += (zq[k]-mean)*(zq[k]-mean);
                var = sqrt(var);

                if(var<variance)
                    indices.push_back(i*m_depth.cols+j);


            }

        }

    }

    out << "ply" << endl;
    out <<  "format ascii 1.0" << endl;
    out <<  "comment" << endl;
    out << "element vertex " << m_depth.rows*m_depth.cols << endl;
    out << "property float32 x" << endl;
    out << "property float32 y" << endl;
    out << "property float32 z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "element face " << indices.size() << endl;
    out << "property list uchar int vertex_index" << endl;
    out << "end_header" << endl;

    double fu, fv, cu, cv, x, y, z;
    fu = ui->fuEdit->text().toDouble();
    fv = ui->fvEdit->text().toDouble();
    cu = ui->cuEdit->text().toDouble();
    cv = ui->cvEdit->text().toDouble();

    update_zmax();

    for(size_t i=0; i<(size_t)m_depth.rows; i++) {

        for(size_t j=0; j<(size_t)m_depth.cols; j++) {


            //z = (double)m_depth.at<unsigned short>(i,j);
            z = get_smoothed_depth(i,j);

            if(z<m_zmax) {

                x = (z/fu)*((double)j-cu);
                y = (z/fv)*((double)i-cv);

            }
            else {

                x = 0;
                y = 0;
                z = 0;

            }

            out << x << " " << y << " " << z << " " << (unsigned int)m_rgb.at<Vec3b>(i,j)[2] << " " << (unsigned int)m_rgb.at<Vec3b>(i,j)[1] << " " << (unsigned int)m_rgb.at<Vec3b>(i,j)[0] << endl;

        }

    }

    // write faces (maybe as triangles)
    for(size_t k=0; k<indices.size(); k++)
            out << 4 << " " << indices[k] << " " << indices[k]+m_depth.cols << " " << indices[k]+m_depth.cols+1 << " " << indices[k]+1 << endl;

    out.close();

    return 0;

}

void MainWindow::on_actionUpdateClipDepth_triggered()
{

    update_zmax();

    stringstream ss;
    size_t zmax = (size_t)m_zmax;
    ss << zmax;

    ui->maxDepth->setText(QString(ss.str().c_str()));
}

void MainWindow::update_zmax() {

    m_zmax = (double)ui->depthclipSlider->sliderPosition()*(6000.0/99.0);

}

void MainWindow::on_actionSet_Directory_triggered()
{

    on_pauseButton_clicked();
    QString filename = QFileDialog::getExistingDirectory(this,tr("Choose folder..."), ".");
    ui->dirEdit->setText(filename);

}

double MainWindow::get_smoothed_depth(size_t i, size_t j) {

//    size_t s = (double)m_depths.size();

//    double vals[s];

//    for(size_t k=0; k<s; k++) {
//        vals[k] = (double)m_depths[k].at<unsigned short>(i,j);
//        //cout << vals[k] << endl;

//      }

//    sort(vals,vals+s);

//    if(s%2==1)
//        return vals[(size_t)((s+1)/2)-1];
//    else
//        return 0.5*(vals[s/2-1]+vals[s/2]);

   return (double)m_depths.back().at<unsigned short>(i,j);

}


void MainWindow::on_action3D_View_triggered()
{
    m_viewer = new ViewerWindow(this);
    m_viewer->show();


}
