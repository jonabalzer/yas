#include "viewerwindow.h"
#include "ui_viewerwindow.h"


using namespace std;

ViewerWindow::ViewerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ViewerWindow)
{
    ui->setupUi(this);

    connect(this,SIGNAL(current_image_changed(Mat&,Mat&)),ui->pcPanel,SLOT(set_pcl(Mat&,Mat&)));

}

ViewerWindow::~ViewerWindow()
{
    delete ui;
}



void ViewerWindow::on_current_image_changed(Mat& rgb, Mat& depth) {

    emit this->current_image_changed(rgb,depth);

}
