#ifndef VIEWERWINDOW_H
#define VIEWERWINDOW_H

#include "opencv2/opencv.hpp"
#include <QMainWindow>

using namespace cv;

namespace Ui {
class ViewerWindow;
}

class ViewerWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ViewerWindow(QWidget *parent = 0);
    ~ViewerWindow();
    
public slots:

    void on_current_image_changed(Mat& rgb, Mat& depth);

signals:

    void current_image_changed(Mat& rgb, Mat& depth);

private:
    Ui::ViewerWindow *ui;
};

#endif // VIEWERWINDOW_H
