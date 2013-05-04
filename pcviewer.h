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

#ifndef PCVIEWER_H
#define PCVIEWER_H

#include "opencv2/opencv.hpp"

#include <QGLWidget>
#include <QWheelEvent>

using namespace cv;

class PCViewer : public QGLWidget
{
    Q_OBJECT
public:
    explicit PCViewer(QWidget *parent = 0);




signals:
    
public slots:

    void set_pcl(Mat& rgb, Mat& depth);

    void on_update_extrinsics(double fu, double fv, double cu, double cv);

private:

    // initialize OpenGL states (triggered by Qt)
    void initializeGL();

    // draw the scene (triggered by Qt)
    void paintGL();

    // handle resize events (triggered by Qt)
    void resizeGL( int w, int h );


    // data members
    Mat m_rgb;
    Mat m_depth;
    //GLdouble m_K[16];
    GLdouble m_F[16];

    void translate(float dx, float dy, float dz);
    void rotate(const float* axis, float angle);

protected:

    //virtual void mousePressEvent(QMouseEvent*);
    //virtual void mouseReleaseEvent(QMouseEvent*);
    //virtual void mouseMoveEvent(QMouseEvent*);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);

};

#endif // PCVIEWER_H
