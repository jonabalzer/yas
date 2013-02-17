#include "pcviewer.h"
#include <GL/glut.h>
#include <math.h>
#include <iostream>

using namespace std;

PCViewer::PCViewer(QWidget *parent) :
    QGLWidget(parent)
{

    setAttribute(Qt::WA_NoSystemBackground, true);
    setFocusPolicy(Qt::StrongFocus);
    setAcceptDrops( true );
    setCursor(Qt::PointingHandCursor);
}


void PCViewer::initializeGL() {

    glShadeModel(GL_SMOOTH);
    glClearColor(0, 0, 0, 1.0);
    glClearDepth(1.0f);
    //glDisable(GL_DITHER);
    glEnable(GL_DEPTH_TEST);

    glDepthFunc(GL_LEQUAL);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   //    gluPerspective(90,1,1,50);

   glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
//    glTranslated(0,0,5);

//    GLfloat pos1[] = { 0.1,  0.1, -0.02, 0.0};
//    GLfloat pos2[] = {-0.1,  0.1, -0.02, 0.0};
//    GLfloat pos3[] = { 0.0,  0.0,  0.1,  0.0};
//    GLfloat col1[] = { 0.7,  0.7,  0.8,  1.0};
//    GLfloat col2[] = { 0.8,  0.7,  0.7,  1.0};
//    GLfloat col3[] = { 1.0,  1.0,  1.0,  1.0};

//    glEnable(GL_LIGHT0);
//    glLightfv(GL_LIGHT0,GL_POSITION, pos1);
//    glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
//    glLightfv(GL_LIGHT0,GL_SPECULAR, col1);

//    glEnable(GL_LIGHT1);
//    glLightfv(GL_LIGHT1,GL_POSITION, pos2);
//    glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
//    glLightfv(GL_LIGHT1,GL_SPECULAR, col2);

//    glEnable(GL_LIGHT2);
//    glLightfv(GL_LIGHT2,GL_POSITION, pos3);
//    glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
//    glLightfv(GL_LIGHT2,GL_SPECULAR, col3);

//    glEnable(GL_LIGHTING);

}

void PCViewer::paintGL() {

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    glLoadIdentity();

//    glTranslatef(0,0,5.0f);
//    glPointSize(2);
//    glColor3f(1.0,1.0,1.0);
//    glVertex3f(0,0,0);
//    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
//    glLoadIdentity();
//    glTranslated(5.0, 5.0, 0.0);
//    glLineWidth(1);
//    glColor3f(0, 0.7f, 0.7f);

//    double radius=3;
//    glLineWidth(2);
//    int sides=5;
//    glColor3f(0, 1, 0);
//    glBegin(GL_LINE_LOOP);
//    for (int i = 0; i < sides; i++){
//    glVertex2f(radius*cos(i*2*3.14159265/sides),
//    radius*sin(i*2*3.14159265/sides));
//    glEnd();

    //glDisable(GL_DEPTH_TEST);

    renderText(0,0,-10,"FUCK");

}

void PCViewer::resizeGL( int width, int height ) {

    //glViewport(0,0,w,h);
    //updateGL();

    double xMin = 0, xMax = 10, yMin = 0, yMax = 10;
    glViewport(0,0,(GLint)width, (GLint)height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1,1,-1,1);
    if (width > height){
    height = height?height:1;
    double newWidth = (xMax - xMin) * width / height;
    double difWidth = newWidth - (xMax - xMin);
    xMin = 0.0 + difWidth / 2.0;
    xMax = 10 + difWidth / 2.0;
    } else {
    width = width?width:1;
    double newHeight = (yMax - yMin) * width / height;
    double difHeight = newHeight - (yMax - yMin);
    yMin = 0.0 + difHeight / 2.0;
    yMax = 10 + difHeight / 2.0;
    }
    gluOrtho2D(xMin, xMax, yMin, yMax);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}
