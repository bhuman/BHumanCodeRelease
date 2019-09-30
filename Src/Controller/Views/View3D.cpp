/**
 * @file Controller/Views/View3D.cpp
 *
 * Implementation of class View3D
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#include <Platform/OpenGL.h>
#include "View3D.h"
#include "Controller/RoboCupCtrl.h"

#include <QGLWidget>
#include <QMouseEvent>
#include <QSettings>

class View3DWidget : public QGLWidget, public SimRobot::Widget
{
public:
  View3DWidget(View3D& view3D) : view3D(view3D)
  {
    setFocusPolicy(Qt::StrongFocus);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view3D.fullName);
    rotation = QPointF(settings.value("RotationX").toFloat(), settings.value("RotationY").toFloat());
    settings.endGroup();
  }

  ~View3DWidget()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view3D.fullName);
    settings.setValue("RotationX", rotation.rx());
    settings.setValue("RotationY", rotation.ry());
    settings.endGroup();
  }

private:
  QPointF rotation;
  int width;
  int height;
  View3D& view3D;
  bool dragging = false;
  QPoint dragStart;

  void resizeGL(int newWidth, int newHeight) override
  {
    width = newWidth;
    height = newHeight;
  }

  void paintGL() override
  {
    GLdouble aspect = height ? static_cast<GLdouble>(width) / static_cast<GLdouble>(height) : static_cast<GLdouble>(width);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(1.5); // required
    glPointSize(2.5);
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);

    if(!glIsList(View3D::cubeId) || view3D.needsUpdate())
      view3D.updateDisplayLists();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(view3D.background.x(), view3D.background.y(), view3D.background.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(25, aspect, 1, 100);

    glTranslated(0.0f, 0.0f, -view3D.getViewDistance());
    glRotated(rotation.x(), 1.0f, 0.0f, 0.0f);
    glRotated(rotation.y(), 0.0f, 0.0f, 1.0f);

    glCallList(View3D::cubeId);
    glCallList(View3D::colorsId);

    view3D.lastBackground = view3D.background;
  }

  void mousePressEvent(QMouseEvent* event) override
  {
    QWidget::mousePressEvent(event);

    if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
    {
      dragStart = event->pos();
      dragging = true;
    }
  }

  void mouseReleaseEvent(QMouseEvent* event) override
  {
    QWidget::mouseReleaseEvent(event);

    dragging = false;
  }

  void mouseMoveEvent(QMouseEvent* event) override
  {
    QWidget::mouseMoveEvent(event);

    if(dragging)
    {
      QPoint diff(event->pos() - dragStart);
      dragStart = event->pos();
      rotation.ry() += diff.x();
      rotation.rx() += diff.y();
      updateGL();
    }
  }

  void mouseDoubleClickEvent(QMouseEvent* event) override
  {
    QWidget::mouseDoubleClickEvent(event);

    rotation = QPointF();
    updateGL();
  }

  void wheelEvent(QWheelEvent* event) override
  {
    if(event->delta())
    {
      if(event->orientation() == Qt::Horizontal)
        rotation.ry() += static_cast<float>(event->delta()) * 0.2f;
      else
        rotation.rx() += static_cast<float>(event->delta()) * 0.2f;
      updateGL();
    }
    else
      QGLWidget::wheelEvent(event);
  }

  QSize sizeHint() const override { return QSize(320, 240); }

  QWidget* getWidget() override { return this; }

  void update() override
  {
    if(view3D.background != view3D.lastBackground || view3D.needsUpdate())
      QGLWidget::update();
  }
};

View3D::View3D(const QString& fullName, const Vector3f& background) :
  background(background), fullName(fullName), icon(":/Icons/tag_green.png")
{}

SimRobot::Widget* View3D::createWidget()
{
  return new View3DWidget(*this);
}
