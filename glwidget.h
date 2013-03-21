#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMouseEvent>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <map>

class QGLViewerWidget : public QGLWidget
{

  Q_OBJECT

public:

  // Default constructor.
  QGLViewerWidget(QWidget* _parent=0);

  // Destructor.
  virtual ~QGLViewerWidget();

public:

  /* Sets the center and size of the whole scene.
     The _center is used as fixpoint for rotations and for adjusting
     the camera/viewer (see view_all()). */
  void set_scene_pos( const cv::Vec3f& _center, float _radius);

  /* view the whole scene: the eye point is moved far enough from the
     center so that the whole scene is visible. */
  void view_all();

  float radius() const { return m_radius; }
  const cv::Vec3f& center() const { return m_center; }

  const GLdouble* modelview_matrix() const  { return m_modelview_matrix;  }
  const GLdouble* projection_matrix() const { return m_projection_matrix; }

  float fovy() const { return 45.0f; }

protected:

private slots:

  void set_pcl(const std::vector<cv::Point3f>& points, const std::vector<cv::Vec3b>& colors);

private:

  // initialize OpenGL states (triggered by Qt)
  void initializeGL();

  // draw the scene (triggered by Qt)
  void paintGL();

  // handle resize events (triggered by Qt)
  void resizeGL(int w, int h);

protected:

  // Qt mouse events
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);

private:

  // updates projection matrix
  void update_projection_matrix();

  // translate the scene and update modelview matrix
  void translate(const cv::Vec3f& _trans);

  // rotate the scene (around its center) and update modelview matrix
  void rotate(const cv::Vec3f& _axis, float _angle);

  cv::Vec3f m_center;
  float m_radius;
  GLdouble m_projection_matrix[16];
  GLdouble m_modelview_matrix[16];

  // virtual trackball: map 2D screen point to unit sphere
  bool map_to_sphere(const QPoint& point, cv::Vec3f& result);

  QPoint           m_last_point_2d;
  cv::Vec3f        m_last_point_3d;
  bool             m_last_point_ok;

  std::vector<cv::Point3f> m_points;
  std::vector<cv::Vec3b> m_colors;

};



#endif // GLWIDGET_H
