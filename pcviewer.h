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

protected:

    //virtual void mousePressEvent(QMouseEvent*);
    //virtual void mouseReleaseEvent(QMouseEvent*);
    //virtual void mouseMoveEvent(QMouseEvent*);
    virtual void wheelEvent(QWheelEvent* event);

};

#endif // PCVIEWER_H
