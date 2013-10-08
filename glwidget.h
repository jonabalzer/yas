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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMouseEvent>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <map>

#include "poisson/Ply.h"
#include "tgCamera.h"
#include "tgModel.h"
#include "cam.h"


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
  void set_scene_pos(const cv::Vec3f& center, float radius);

  /* view the whole scene: the eye point is moved far enough from the
     center so that the whole scene is visible. */
  void view_all();

  float radius() const { return m_radius; }
  const cv::Vec3f& center() const { return m_center; }

  // clear the current data and redraw
  void clear_data() { m_points.clear(); m_colors.clear(); m_mesh.Clear(); updateGL(); }

protected:

private slots:

  void set_pcl(const std::vector<cv::Point3f>& points, const std::vector<cv::Vec3b>& colors);

  void set_mesh(const PoissonRec::CoredVectorMeshData<PoissonRec::PlyVertex<float> > &mesh);

  void configure_cam(const CCam& rgb, const CDepthCam& depth);

private:

  // initialize OpenGL states (triggered by Qt)
  void initializeGL();

  // draws a coordinate frame at the origin (0,0,0)
  void drawCoordinates(float length=1.0);

  void drawPoints(float size=1.0);

  void drawMesh();

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

  virtual void keyPressEvent(QKeyEvent *event);

private:

  // updates projection matrix
  void update_projection_matrix();

  // translate the scene and update modelview matrix
  void translate(const cv::Vec3f& _trans);

  // rotate the scene (around its center) and update modelview matrix
  void rotate(const cv::Vec3f& _axis, float _angle);

  cv::Vec3f m_center;
  float m_radius;
  TomGine::tgCamera m_cam_origin;
  TomGine::tgCamera m_cam_perspective;
  QPoint m_last_point_2d;
  std::vector<cv::Point3f> m_points;
  std::vector<cv::Vec3b> m_colors;

  TomGine::tgModel m_mesh;

};



#endif // GLWIDGET_H
