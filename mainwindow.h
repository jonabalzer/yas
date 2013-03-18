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
#include "psensors.h"
#include "dsensor.h"

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

signals:

    void current_image_changed(Mat& rgb, Mat& depth);

public slots:

    // overloading to make sure the application quits when main window closes
    virtual bool close() { on_actionExit_triggered(); };

private slots:

    bool on_stepButton_clicked();

    void on_runButton_clicked();

    void on_pauseButton_clicked();

    void on_actionUpdateClipDepth_triggered();

    void on_action3D_View_triggered();

    void on_storeButton_clicked();

    void on_spinBoxStorage_valueChanged(int arg1);

    void on_alignButton_clicked();

    void on_actionExit_triggered();

    void on_clearButton_clicked();

    void updata_static_view(Mat& rgb, Mat& depth);

    void on_actionAbout_triggered();

    void on_actionSave_triggered();

    void on_actionOpen_triggered();

    void on_actionSave_all_triggered();

    void on_triangulateCheckBox_stateChanged(int arg1);

    void on_alignAllButton_clicked();

private:

    // ui
    Ui::MainWindow *ui;

    // video capture
    //CPrimeSensors m_sensors;
    CDepthColorSensor m_sensors;
    QTimer m_timer;

    // current data
    Mat m_rgb;
    boost::circular_buffer<Mat> m_depth_buffer;

    // saved images
    vector<Mat> m_rgb_storage;          //!< stored rgb images
    vector<Mat> m_depth_storage;        //!< stored depth images
    vector<Mat> m_trafo_storage;        //!< stored transformations between them

    // maximum depth
    double m_zmax;

    // windows for tools
    ViewerWindow* m_viewer;
    AlignWindow* m_alignment;

    // save routines
    bool save_pcl_as_ply(size_t index, QString fn);
    bool save_mesh_as_ply(size_t index, QString fn);
    bool save_as_png(size_t index, QString fn);
    bool save_as_pgm(size_t index, QString fn);
    bool save_as_exr(size_t index, QString fn);

    // helper routine
    unsigned short get_smoothed_depth(size_t i, size_t j);
    Mat get_depth_from_buffer();
    void update_live_view();

    // geometry/alignment functions
    Mat transform_to_first_image(size_t index);
    Mat estimate_world_frame();

    /*! \brief Converts an image in the storage into a colored 3d point cloud for export or visualization.
     * \param[in] index number of stored depth image
     * \param[out] vertices point cloud in 3d
     * \param[out] colors color of points
     * \param[in] maxr threshold on the distance points can have to the origin (after application of the transform \f$F\f$)
     * \param[in] F coordinate to apply to the points
     */
    void get_pcl(size_t index, vector<Point3f>& vertices, vector<Vec3b>& colors, float maxr = 10000, Mat F = Mat::eye(4,4,CV_32FC1));

    /*! \brief Converts an image in the storage into a colored 3d mesh.
     * \param[in] index number of stored depth image
     * \param[out] vertices point cloud in 3d
     * \param[out] colors color of points
     * \param[out] faces connectivity of the mesh
     * \param[in] maxr threshold on the distance points can have to the origin (after application of the transform \f$F\f$)
     * \param[in] F coordinate to apply to the points
     */
    void get_mesh(size_t index, vector<Point3f>& vertices, vector<Vec3b>& colors, vector<Vec4i>& faces, float maxr = 10000, Mat F = Mat::eye(4,4,CV_32FC1));

protected:

    //! Handles keyboard input.
    virtual void keyPressEvent(QKeyEvent* event);

};

#endif // MAINWINDOW_H
