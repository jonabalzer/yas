#ifndef PCVIEWER_H
#define PCVIEWER_H

#include "opencv2/opencv.hpp"

#include <QGLWidget>

class PCViewer : public QGLWidget
{
    Q_OBJECT
public:
    explicit PCViewer(QWidget *parent = 0);




signals:
    
public slots:

    void set_pcl(cv::Mat& rgb, cv::Mat& depth);

private:

    // initialize OpenGL states (triggered by Qt)
    void initializeGL();

    // draw the scene (triggered by Qt)
    void paintGL();

    // handle resize events (triggered by Qt)
    void resizeGL( int w, int h );


    // data members
    cv::Mat m_rgb;
    cv::Mat m_depth;

};

#endif // PCVIEWER_H
