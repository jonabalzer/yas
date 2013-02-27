#include "pcviewer.h"
//#include <GL/glut.h>
#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

PCViewer::PCViewer(QWidget *parent) :
    QGLWidget(parent),
    m_rgb(),
    m_depth()
{

    setAttribute(Qt::WA_NoSystemBackground, true);
    setFocusPolicy(Qt::StrongFocus);
    setAcceptDrops( true );
    setCursor(Qt::PointingHandCursor);
}

void PCViewer::set_pcl(cv::Mat& rgb, cv::Mat& depth) {

    m_rgb = rgb;
    m_depth = depth;

}


void PCViewer::initializeGL() {

    glShadeModel(GL_SMOOTH);
    glClearColor(0, 0, 0, 1.0);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);

    glDepthFunc(GL_LEQUAL);

    glMatrixMode(GL_PROJECTION);    // load projection matrix of kinect
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    GLfloat pos1[] = { 0.1,  0.1, -0.02, 0.0};
    GLfloat pos2[] = {-0.1,  0.1, -0.02, 0.0};
    GLfloat pos3[] = { 0.0,  0.0,  0.1,  0.0};
    GLfloat col1[] = { 0.7,  0.7,  0.8,  1.0};
    GLfloat col2[] = { 0.8,  0.7,  0.7,  1.0};
    GLfloat col3[] = { 1.0,  1.0,  1.0,  1.0};

    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0,GL_POSITION, pos1);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
    glLightfv(GL_LIGHT0,GL_SPECULAR, col1);

    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1,GL_POSITION, pos2);
    glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
    glLightfv(GL_LIGHT1,GL_SPECULAR, col2);

    glEnable(GL_LIGHT2);
    glLightfv(GL_LIGHT2,GL_POSITION, pos3);
    glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
    glLightfv(GL_LIGHT2,GL_SPECULAR, col3);

    glEnable(GL_LIGHTING);

}

void PCViewer::paintGL() {

   if(m_depth.rows==0 || m_depth.cols==0 || m_rgb.rows==0 || m_rgb.cols==0)
        return;

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glLoadIdentity();
   glPointSize(5);


   double fu, fv, cu, cv;
   fu = 515;
   fv = 515;
   cu = 320;
   cv = 240;

   glBegin(GL_POINTS);

   for(size_t i=0; i<(size_t)m_depth.rows; i++) {

       for(size_t j=0; j<(size_t)m_depth.cols; j++) {

            double z = (double)m_depth.at<unsigned short>(i,j);

            float r, g, b;
            r = (float)m_rgb.at<Vec3b>(i,j)[2]/255.0;
            g = (float)m_rgb.at<Vec3b>(i,j)[1]/255.0;
            b = (float)m_rgb.at<Vec3b>(i,j)[0]/255.0;

            glColor3f((z/fu)*((double)j-cu),(z/fv)*((double)i-cv),z);
            glVertex3d(r,g,b);

       }

   }


   glEnd();


}

void PCViewer::resizeGL( int width, int height ) {

    glViewport(0,0,width,height);
    updateGL();

}
