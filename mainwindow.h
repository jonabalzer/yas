/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer
//
// All rights reserved.
//
// This file is part of the R4R library.
//
// The R4R library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The R4R library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the R4R library. If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QKeyEvent>

#include <ImfRgbaFile.h>
#include <ImfArray.h>
#include <ImfStandardAttributes.h>
#include <ImfMatrixAttribute.h>
#include <ImfAttribute.h>

#include "opencv2/opencv.hpp"

#include <boost/circular_buffer.hpp>

#include "alignwindow.h"
#include "dsensor.h"
#include "glwidget.h"
#include "params.h"

#include "poisson/Ply.h"

using namespace Imf;
using namespace Imath;

namespace Ui {
class MainWindow;
//class Params;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    size_t storage_size() { return m_rgb_storage.size(); }

signals:

    void current_image_changed(cv::Mat& rgb, cv::Mat& depth);
    void current_pcl_changed(const std::vector<cv::Point3f>& points, const std::vector<cv::Vec3b>& colors);
    void current_mesh_changed(const PoissonRec::CoredVectorMeshData<PoissonRec::PlyVertex<float> > &mesh);

public slots:

    // overloading to make sure the application quits when main window closes
    virtual bool close() { on_actionExit_triggered(); return 0; }

private slots:

    bool on_stepButton_clicked();

    void on_runButton_clicked();

    void on_pauseButton_clicked();

    void on_actionUpdateClipDepth_triggered();

    void on_action3D_View_triggered();

    void on_storeButton_clicked();

    void on_spinBoxStorage_valueChanged(int arg1);

    void on_actionExit_triggered();

    void on_clearButton_clicked();

    void update_static_view(cv::Mat& rgb, cv::Mat& depth);

    void on_actionAbout_triggered();

    void on_actionSave_triggered();

    void on_actionOpen_triggered();

    void on_actionSave_all_triggered();

    void on_actionPreferences_triggered();

    void on_recordButton_clicked(bool checked);

    void configure_sensor(const CCam& rgb, const CDepthCam& depth);

    void on_saveParams_clicked();

    void on_actionPoisson_triggered();

    void on_actionCurrent_triggered();

    void on_actionAll_triggered();

    void on_actionSplit_ONI_Stream_triggered();

private:

    // ui
    Ui::MainWindow *ui;

    // video capture
    CDepthColorSensor m_sensor;
    QTimer m_timer;

    // current data
    cv::Mat m_rgb;
    boost::circular_buffer<cv::Mat> m_depth_buffer;

    // saved images
    std::vector<cv::Mat> m_rgb_storage;          //!< stored rgb images
    std::vector<cv::Mat> m_depth_storage;        //!< stored depth images
    std::vector<cv::Mat> m_trafo_storage;        //!< stored transformations between them

    // windows for tools
    AlignWindow* m_alignment;
    QGLViewerWidget* m_glview;
    Params* m_params;

    // the reconstruction
    PoissonRec::CoredVectorMeshData<PoissonRec::PlyVertex<float> > m_mesh;

    // save routines
    bool save_pcl_as_ply(size_t index, QString fn);
    bool save_as_png(size_t index, QString fn);
    bool save_as_pgm(size_t index, QString fn);
    bool save_as_exr(size_t index, QString fn);
    bool save_trafo(size_t index, QString fn);
    bool save_normal_map(size_t index, QString fn);

    // helper routine
    unsigned short get_smoothed_depth(size_t i, size_t j);
    cv::Mat get_depth_from_buffer();
    void update_live_view();

    // geometry/alignment functions
    cv::Mat transform_to_first_image(size_t index);
    cv::Mat estimate_world_frame();
    void refine_alignement(size_t index);

    /*! \brief Converts an image in the storage into a colored 3d point cloud for export or visualization.
     * \param[in] index number of stored depth image
     * \param[out] vertices point cloud in 3d
     * \param[out] colors color of points
     * \param[in] maxr threshold on the distance points can have to the origin (after application of the transform \f$F\f$)
     * \param[in] F coordinate to apply to the points
     */
    void get_pcl(size_t index, std::vector<cv::Point3f>& vertices, std::vector<cv::Vec3b>& colors, float maxr = 10000, cv::Mat F = cv::Mat::eye(4,4,CV_32FC1));

    void get_oriented_pcl(size_t index, std::vector<cv::Point3f>& vertices, std::vector<cv::Point3f>& normals, std::vector<cv::Vec3b>& colors, float minr = 0, float maxr = 10000, cv::Mat F = cv::Mat::eye(4,4,CV_32FC1));

protected:

    //! Handles keyboard input.
    virtual void keyPressEvent(QKeyEvent* event);

};

#endif // MAINWINDOW_H
