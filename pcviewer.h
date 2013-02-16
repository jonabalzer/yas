#ifndef PCVIEWER_H
#define PCVIEWER_H

#include <QGLWidget>

class PCViewer : public QGLWidget
{
    Q_OBJECT
public:
    explicit PCViewer(QWidget *parent = 0);
    
signals:
    
public slots:
    
private:

    // initialize OpenGL states (triggered by Qt)
    void initializeGL();

    // draw the scene (triggered by Qt)
    void paintGL();

    // handle resize events (triggered by Qt)
    void resizeGL( int w, int h );

};

#endif // PCVIEWER_H
