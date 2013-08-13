#include "pcviewer.h"

#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

PCViewer::PCViewer(QWidget *parent) :
    QGLWidget(parent),
    m_rgb(),
    m_depth() {

    setAttribute(Qt::WA_NoSystemBackground, true);
    setFocusPolicy(Qt::StrongFocus);
    setAcceptDrops(true);
    setCursor(Qt::PointingHandCursor);

}

void PCViewer::set_pcl(cv::Mat& rgb, cv::Mat& depth) {

    m_rgb = rgb;
    m_depth = depth;

    updateGL();

}


void PCViewer::initializeGL() {

    glShadeModel(GL_SMOOTH);
    glClearColor(0, 0, 0, 1.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glDepthFunc(GL_LEQUAL);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-500,500,-500,500, 1, 6000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glGetDoublev(GL_MODELVIEW_MATRIX,m_F);

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
   glMatrixMode( GL_MODELVIEW );
   glLoadMatrixd(m_F);


   glPointSize(2);

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

            glColor3f(r,g,b);
            glVertex3d((z/fu)*((double)j-cu),-(z/fv)*((double)i-cv),-z);

       }

   }


   glEnd();

}

void PCViewer::resizeGL( int width, int height ) {

    glViewport(0,0,width,height);
    updateGL();

}

void PCViewer::on_update_extrinsics(double fu, double fv, double cu, double cv) {

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLdouble K[16];




    //glGetDoublev(GL_PROJECTION_MATRIX,K);

    glMatrixMode(GL_MODELVIEW);



}

void PCViewer::translate(float dx, float dy, float dz) {

    makeCurrent();
    glLoadIdentity();
    glTranslated(dx,dy,dz);
    glMultMatrixd(m_F);
    glGetDoublev(GL_MODELVIEW_MATRIX,m_F);

}

void rotate(const float* axis, float angle) {



}

void PCViewer::wheelEvent(QWheelEvent* event) {

  float d = -(float)event->delta() / 120.0 * 0.2 * 100;
  cout << d << endl;
  translate(0,0,d);
  updateGL();
  event->accept();

}

void PCViewer::keyPressEvent(QKeyEvent* event) {

    translate(10,0,50);
    //paintGL();
    updateGL();

    this->update();
    cout << "trans" << endl;

}
