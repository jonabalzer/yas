#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <ImfRgbaFile.h>
#include <ImfArray.h>
#include "opencv2/opencv.hpp"
#include <boost/circular_buffer.hpp>

using namespace cv;
using namespace Imf;
using namespace Imath;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private slots:
    void on_stepButton_clicked();

    void on_runButton_clicked();

    void on_pauseButton_clicked();

    void on_saveButton_clicked();

    void on_actionUpdateClipDepth_triggered();

    void on_actionSet_Directory_triggered();

private:
    Ui::MainWindow *ui;
    VideoCapture m_cap;
    QTimer m_timer;
    Mat m_rgb;
    Mat m_depth;
    size_t m_img_counter[3];
    double m_zmax;

    bool save_as_ply(string fn);
    void update_zmax();
    boost::circular_buffer<Mat> m_depths;

    double get_smoothed_depth(size_t i, size_t j);

};

#endif // MAINWINDOW_H
