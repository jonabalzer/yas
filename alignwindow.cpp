#include "alignwindow.h"
#include "ui_alignwindow.h"

#include "mainwindow.h"

AlignWindow::AlignWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AlignWindow)
{
    ui->setupUi(this);



    //MainWindow* mw = qobject_cast<MainWindow*>(parent);
    //ui->spinBox->setMaximum(mw->storage_size());

}

AlignWindow::~AlignWindow()
{
    delete ui;
}

void AlignWindow::show_image(QImage img) {

    ui->corrLabel->setPixmap(QPixmap::fromImage(img));

}
