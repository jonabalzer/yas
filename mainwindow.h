#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QTimer>
#include <QKeyEvent>
#include <ImfRgbaFile.h>
#include <ImfArray.h>
#include "opencv2/opencv.hpp"
#include <boost/circular_buffer.hpp>
#include "viewerwindow.h"
#include "alignwindow.h"

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

    size_t storage_size() { return m_rgb_storage.size(); };
    
private slots:
    bool on_stepButton_clicked();

    void on_runButton_clicked();

    void on_pauseButton_clicked();

    void on_saveButton_clicked();

    void on_actionUpdateClipDepth_triggered();

    void on_action3D_View_triggered();

    void on_loadButton_clicked();

    void on_storeButton_clicked();

    void on_spinBoxStorage_valueChanged(int arg1);

    void on_saveAllButton_clicked();

    void on_alignButton_clicked();

private:
    Ui::MainWindow *ui;

    VideoCapture m_cap;
    QTimer m_timer;

    // current data
    Mat m_rgb;
    Mat m_depth;    // remove this if we work with the ring buffer

    // saved images
    vector<Mat> m_rgb_storage;          //!< stored rgb images
    vector<Mat> m_depth_storage;        //!< stored depth images
    vector<Mat> m_trafo_storage;        //!< stored transformations between them

    // maximum depth
    double m_zmax;

    // depth buffer
    boost::circular_buffer<Mat> m_depth_buffer;

    // windows for tools
    ViewerWindow* m_viewer;
    AlignWindow* m_alignment;


    bool save_as_ply(size_t index, QString fn);
    bool save_as_png(size_t index, QString fn);
    bool save_as_exr(size_t index, QString fn);

    void update_zmax();
    unsigned short get_smoothed_depth(size_t i, size_t j);
    Mat get_depth_from_buffer();

    QImage convert_depth(Mat& depth);
    QImage convert_rgb(Mat& depth);

    Mat transforms_to_first_image(size_t index);

protected:

    virtual void keyPressEvent(QKeyEvent* event);


};

#endif // MAINWINDOW_H
