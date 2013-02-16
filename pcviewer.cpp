#include "pcviewer.h"
#include <GL/glut.h>
#include <math.h>

PCViewer::PCViewer(QWidget *parent) :
    QGLWidget(parent)
{
}


void PCViewer::initializeGL() {

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glDisable(GL_DITHER);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    gluPerspective(90,1,1,50);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0,0,0);

}

void PCViewer::paintGL() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glLineWidth(1);
    glColor3f(0, 0.7f, 0.7f);
    glTranslated(5.0, 5.0, 0.0);
    glBegin(GL_POLYGON);
    for (int i = 0; i < 4; i++){
    glVertex2f(2*cos(i*2*3.14159265/4),
    2*sin(i*2*3.14159265/4));
    }
    glEnd();
}

void PCViewer::resizeGL( int w, int h ) {

    glViewport(0,0,w,h);
    updateGL();


}
