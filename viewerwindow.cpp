#include "viewerwindow.h"
#include "ui_viewerwindow.h"

ViewerWindow::ViewerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ViewerWindow)
{
    ui->setupUi(this);
}

ViewerWindow::~ViewerWindow()
{
    delete ui;
}

void ViewerWindow::on_pushButton_clicked()
{
    ui->pcPanel->updateGL();

}
