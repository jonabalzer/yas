#include "glwidget.h"

const double TRACKBALL_RADIUS = 0.6;

using namespace cv;
using namespace std;

QGLViewerWidget::QGLViewerWidget(QWidget* _parent)
  : QGLWidget(_parent)
{

  setAttribute(Qt::WA_NoSystemBackground,true);
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setCursor(Qt::PointingHandCursor);

  m_center = cv::Vec3f(0.0,0.0,1.0);
  m_radius = 1.0f;

  this->setWindowTitle("3D View");
}

QGLViewerWidget::~QGLViewerWidget() {}

void QGLViewerWidget::initializeGL() {

  glClearColor(0.5, 0.5, 0.5, 0.0);
  //glDisable(GL_DITHER);
  glEnable(GL_DEPTH_TEST);
  //glEnable(GL_COLOR_MATERIAL);

  //  // set lights
  //  GLfloat pos1[] = { 0.1,  0.1, -0.02, 0.0};
  //  GLfloat pos2[] = {-0.1,  0.1, -0.02, 0.0};
  //  GLfloat pos3[] = { 0.0,  0.0,  0.1,  0.0};
  //  GLfloat col1[] = { 0.7,  0.7,  0.8,  1.0};
  //  GLfloat col2[] = { 0.8,  0.7,  0.7,  1.0};
  //  GLfloat col3[] = { 1.0,  1.0,  1.0,  1.0};

  //  glEnable(GL_LIGHT0);
  //  glLightfv(GL_LIGHT0,GL_POSITION, pos1);
  //  glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
  //  glLightfv(GL_LIGHT0,GL_SPECULAR, col1);

  //  glEnable(GL_LIGHT1);
  //  glLightfv(GL_LIGHT1,GL_POSITION, pos2);
  //  glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
  //  glLightfv(GL_LIGHT1,GL_SPECULAR, col2);

  //  glEnable(GL_LIGHT2);
  //  glLightfv(GL_LIGHT2,GL_POSITION, pos3);
  //  glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
  //  glLightfv(GL_LIGHT2,GL_SPECULAR, col3);

  //  glEnable(GL_LIGHTING);

  // scene pos and size
  //  glMatrixMode(GL_MODELVIEW);
  //  glLoadIdentity();
  //  glGetDoublev(GL_MODELVIEW_MATRIX, m_modelview_matrix);
  //  set_scene_pos(Vec3f(0.0, 0.0, 0.5),1.0);

  //  TomGine::vec4 vp;
  //  glGetFloatv(GL_VIEWPORT, vp);

  //  TomGine::mat4 intrinsic = TomGine::tgCamera::Perspective(1.0f/tan(0.5f * 60.0f * M_PI / 180.0f), vp[2]/vp[3], 10.0f, 0.1f);
  //  TomGine::mat4 extrinsic;

  //  m_cam_perspective.Set(intrinsic, extrinsic, vp[2], vp[3]);
  //  m_cam_perspective.Translate(TomGine::vec3(0,0,1));
  //  m_cam_perspective.ApplyTransform();
  //  m_cam_perspective.Activate();
}

void QGLViewerWidget::resizeGL(int w, int h)
{
  m_cam_perspective.SetViewport(w, h);
  m_cam_perspective.Activate();

  //  update_projection_matrix();
  //  glViewport(0,0,w,h);
  updateGL();

}

void QGLViewerWidget::drawCoordinates(float length)
{
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor3ub(255,0,0); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(length,0.0f,0.0f);
  glColor3ub(0,255,0); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,length,0.0f);
  glColor3ub(0,0,255); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,0.0f,length);
  glEnd();
}

void QGLViewerWidget::drawPoints(float size)
{
  glPointSize(size);
  glBegin(GL_POINTS);
  for(size_t i=0; i<m_points.size(); i++)
  {
    glColor3ubv(&m_colors[i][0]);
    glVertex3fv(&m_points[i].x);
  }
  glEnd();
}


void QGLViewerWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  m_cam_perspective.Activate();

  drawPoints();

  drawCoordinates();
}

void QGLViewerWidget::set_pcl(const std::vector<cv::Point3f>& points, const std::vector<cv::Vec3b>& colors) {

  m_points = points;
  m_colors = colors;

  updateGL();
  // also update the scene_pos here, or from outside with a rough estimate: cente-> central pixel point, etc

}

void QGLViewerWidget::configure_cam(const CCam& rgb, const CDepthCam& depth)
{
  cv::Mat extrinsic = depth.GetExtrinsics();

  size_t size[2];
  float f[2];
  float c[2];
  float alpha;
  float k[5];
  float range[2];
  depth.GetIntrinsics(size, f, c, alpha, k);
  depth.GetRange(range);

//  printf("%d %d\n", size[0], size[1]);
//  printf("%f %f\n", f[0], f[1]);
//  printf("%f %f\n", c[0], c[1]);
//  printf("%f\n", alpha);
//  printf("%f %f %f %f %f\n", k[0], k[1], k[2], k[3], k[4]);
//  printf("%f %f\n", range[0], range[1]);

  TomGine::tgCamera::Parameter params;
  params.width = size[0];
  params.height = size[1];
  params.fx = f[0];
  params.fy = f[1];
  params.cx = c[0];
  params.cy = c[1];
  params.zNear = 0.5 * range[0];
  params.zFar = 2.0 * range[1];

  m_cam_origin.Set(params);

  m_cam_perspective = m_cam_origin;

//  printf("[configure_cam] camera:\n");
//  m_cam_perspective.Print();
}


void QGLViewerWidget::mousePressEvent(QMouseEvent* event)
{

  m_last_point_2d = event->pos();
  //m_last_point_ok = map_to_sphere(m_last_point_2d,m_last_point_3d);

}

void QGLViewerWidget::mouseMoveEvent(QMouseEvent* event) {

  QPoint newPoint2D = event->pos();

  // Left button: rotate around center_
  // Middle button: translate object
  // Left & middle button: zoom in/out

  //Vec3f  newPoint3D;
  //bool   newPoint_hitSphere = map_to_sphere( newPoint2D, newPoint3D );

  float dx = newPoint2D.x() - m_last_point_2d.x();
  float dy = newPoint2D.y() - m_last_point_2d.y();

  float far = m_cam_perspective.GetZFar();
  float near = m_cam_perspective.GetZNear();

  // enable GL context
  makeCurrent();

  // move in z direction
  if ( (event->buttons() == (Qt::LeftButton+Qt::MidButton)) ||
       (event->buttons() == Qt::LeftButton && event->modifiers() == Qt::ControlModifier))
  {
    m_cam_perspective.TranslateF(0.001f * (far - near) * dx);
    m_cam_perspective.TranslateF(0.001f * (far - near) * dy);

    //float value_y = m_radius * dy * 3.0 / h;
    //translate(Vec3f(0.0, 0.0, value_y));
  }


  // move in x,y direction
  else if ( (event->buttons() == Qt::MidButton) ||
            (event->buttons() == Qt::LeftButton && event->modifiers() == Qt::AltModifier) )
  {
    m_cam_perspective.TranslateS(-0.0005f * (far - near) * dx);
    m_cam_perspective.TranslateU(0.0005f * (far - near) * dy);

    //    float z = - (m_modelview_matrix[ 2]*m_center[0] +
    //         m_modelview_matrix[ 6]*m_center[1] +
    //         m_modelview_matrix[10]*m_center[2] +
    //         m_modelview_matrix[14]) /
    //                (m_modelview_matrix[ 3]*m_center[0] +
    //             m_modelview_matrix[ 7]*m_center[1] +
    //             m_modelview_matrix[11]*m_center[2] +
    //             m_modelview_matrix[15]);

    //    float aspect     = w / h;
    //    float near_plane = 0.01 * m_radius;
    //    float top        = tan(fovy()/2.0f*M_PI/180.0f) * near_plane;
    //    float right      = aspect*top;

    //    translate(Vec3f( 2.0*dx/w*right/near_plane*z,
    //            -2.0*dy/h*top/near_plane*z,
    //             0.0f));
  }



  // rotate
  else if (event->buttons() == Qt::LeftButton)
  {
    TomGine::vec3 cor(m_center[0], m_center[1], m_center[2]);
    m_cam_perspective.Orbit(cor, m_cam_perspective.GetU(), -0.05f * dx);
    m_cam_perspective.Orbit(cor, m_cam_perspective.GetS(), -0.05f * dy);

    //    printf("rotate %f %f %f\n", cor.x, cor.y, cor.z);
    //    if (m_last_point_ok) {
    //      if ((newPoint_hitSphere = map_to_sphere(newPoint2D, newPoint3D))) {

    //        Vec3f axis;
    //        //cvCrossProduct(&last_point_3D_,&newPoint3D,&axis);

    //        axis[0] = m_last_point_3d[1]*newPoint3D[2] - m_last_point_3d[2]*newPoint3D[1];
    //        axis[1] = m_last_point_3d[2]*newPoint3D[0] - m_last_point_3d[0]*newPoint3D[2];
    //        axis[2] = m_last_point_3d[0]*newPoint3D[1] - m_last_point_3d[1]*newPoint3D[0];


    //        if (cv::norm(axis) < 1e-7) {
    //          axis[0] = 1;
    //          axis[1] = 0;
    //          axis[2] = 0;
    //        } else {
    //          axis = cv::normalize(axis);
    //        }
    //        // find the amount of rotation
    //        Vec3f d = m_last_point_3d - newPoint3D;
    //        float t = 0.5 * cv::norm(d) / TRACKBALL_RADIUS;
    //        if (t < -1.0)
    //          t = -1.0;
    //        else if (t > 1.0)
    //          t = 1.0;
    //        float phi = 2.0 * asin(t);
    //        float angle = phi * 180.0 / M_PI;
    //        rotate(axis, angle);
    //      }
    //    }

  }

  m_cam_perspective.ApplyTransform();

  // remember this point
  m_last_point_2d = newPoint2D;
  //  m_last_point_3d = newPoint3D;
  //  m_last_point_ok = newPoint_hitSphere;

  // trigger redraw
  updateGL();
}



void QGLViewerWidget::mouseReleaseEvent( QMouseEvent* /* _event */ )
{
  //m_last_point_ok = false;
}


void QGLViewerWidget::wheelEvent(QWheelEvent* _event)
{

  float d = -(float)_event->delta() / 120.0 * 0.2 * m_radius;

  float far = m_cam_perspective.GetZFar();
  float near = m_cam_perspective.GetZNear();
  m_cam_perspective.TranslateF(0.01f * (far - near) * d);
  m_cam_perspective.TranslateF(0.01f * (far - near) * d);
  m_cam_perspective.ApplyTransform();

  //printf("wheel %f %f\n", d, m_radius);

  //  translate(Vec3f(0.0, 0.0, d));
  updateGL();
  _event->accept();

}

void QGLViewerWidget::keyPressEvent(QKeyEvent *event)
{
  if(event->key() == Qt::Key_Z)
  {
    m_cam_perspective = m_cam_origin;
    m_cam_perspective.Activate();
  }
  updateGL();
}


//void
//QGLViewerWidget::translate(const Vec3f& _trans) {

//  makeCurrent();
//  glLoadIdentity();
//  glTranslated(_trans[0],_trans[1],_trans[2]);
//  glMultMatrixd(m_modelview_matrix);
//  glGetDoublev(GL_MODELVIEW_MATRIX,m_modelview_matrix);

//}

//void
//QGLViewerWidget::rotate(const Vec3f& _axis,float _angle){

//  Vec3f t(m_modelview_matrix[0]*m_center[0] +
//      m_modelview_matrix[4]*m_center[1] +
//      m_modelview_matrix[8]*m_center[2] +
//      m_modelview_matrix[12],
//      m_modelview_matrix[1]*m_center[0] +
//      m_modelview_matrix[5]*m_center[1] +
//      m_modelview_matrix[9]*m_center[2] +
//      m_modelview_matrix[13],
//      m_modelview_matrix[2]*m_center[0] +
//      m_modelview_matrix[6]*m_center[1] +
//      m_modelview_matrix[10]*m_center[2] +
//      m_modelview_matrix[14] );

//  makeCurrent();
//  glLoadIdentity();
//  glTranslatef(t[0], t[1], t[2]);
//  glRotated( _angle, _axis[0], _axis[1], _axis[2]);
//  glTranslatef(-t[0], -t[1], -t[2]);
//  glMultMatrixd(m_modelview_matrix);
//  glGetDoublev(GL_MODELVIEW_MATRIX, m_modelview_matrix);

//}


//bool QGLViewerWidget::map_to_sphere(const QPoint& _v2D, Vec3f& _v3D )
//{
//  // This is actually doing the Sphere/Hyperbolic sheet hybrid thing,
//  // based on Ken Shoemake's ArcBall in Graphics Gems IV, 1993.
//  double x =  (2.0*_v2D.x() - width())/width();
//  double y = -(2.0*_v2D.y() - height())/height();
//  double xval = x;
//  double yval = y;
//  double x2y2 = xval*xval + yval*yval;

//  const double rsqr = TRACKBALL_RADIUS*TRACKBALL_RADIUS;
//  _v3D[0] = xval;
//  _v3D[1] = yval;
//  if (x2y2 < 0.5*rsqr) {
//    _v3D[2] = sqrt(rsqr - x2y2);
//  } else {
//    _v3D[2] = 0.5*rsqr/sqrt(x2y2);
//  }

//  return true;

//}

//void QGLViewerWidget::update_projection_matrix() {

//  makeCurrent();
//  glMatrixMode( GL_PROJECTION );
//  glLoadIdentity();
//  //  gluPerspective(45.0, (GLfloat) width() / (GLfloat) height(),
//  //         0.01*radius_, 100.0*radius_);
//  glOrtho(-1,1,-1,1, 1, 3); // FIXME: no glut

//  glGetDoublev( GL_PROJECTION_MATRIX, m_projection_matrix);
//  glMatrixMode( GL_MODELVIEW );

//}

//void QGLViewerWidget::view_all() {

//  translate( Vec3f( -(m_modelview_matrix[0]*m_center[0] +
//             m_modelview_matrix[4]*m_center[1] +
//      m_modelview_matrix[8]*m_center[2] +
//      m_modelview_matrix[12]),
//      -(m_modelview_matrix[1]*m_center[0] +
//      m_modelview_matrix[5]*m_center[1] +
//      m_modelview_matrix[9]*m_center[2] +
//      m_modelview_matrix[13]),
//      -(m_modelview_matrix[2]*m_center[0] +
//      m_modelview_matrix[6]*m_center[1] +
//      m_modelview_matrix[10]*m_center[2] +
//      m_modelview_matrix[14] +
//      3.0*m_radius) ) );

//}


void
QGLViewerWidget::set_scene_pos( const Vec3f& _cog, float _radius )
{
  m_center = _cog;
  m_radius = _radius;

  //update_projection_matrix();
  //view_all();

}


