/**
* @file SimObjectWidget.h
* Declaration of class SimObjectWidget
* @author Colin Graf
*/

#pragma once

#include "Platform/OpenGL.h"
#include <QGLWidget>

#include "SimRobotCore2.h"
#include "SimObjectRenderer.h"

class QSignalMapper;
class SimObject;
class Simulation;

/**
* @class SimObjectWidget
* A class that implements the 3D-view for simulated objects
*/
class SimObjectWidget : public QGLWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  /**
  * Constructor
  * @param simObject The object that should be displayed
  */
  SimObjectWidget(SimObject& simObject);

  /** Destructor */
  virtual ~SimObjectWidget();

private:
  const SimRobot::Object& object; /**< The object that should be displayed */
  SimObjectRenderer objectRenderer; /**< For rendering the object */
  int fovy;

  bool wkey, akey, skey, dkey;

  virtual QWidget* getWidget() {return this;}
  virtual void update();
  virtual QMenu* createEditMenu() const;
  virtual QMenu* createUserMenu() const;

  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int width, int height);
  virtual void paintEvent(QPaintEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseDoubleClickEvent(QMouseEvent* event);
  virtual void keyPressEvent(QKeyEvent* event);
  virtual void keyReleaseEvent(QKeyEvent* event);
  virtual bool event(QEvent* event);
  virtual void wheelEvent(QWheelEvent* event);
  virtual QSize sizeHint() const {return QSize(320, 240);}

private slots:
  void copy();
  void setSurfaceShadeMode(int style);
  void setPhysicsShadeMode(int style);
  void setDrawingsShadeMode(int style);
  void setDrawingsOcclusion(int flag);
  void setCameraMode(int mode);
  void setFovY(int fovy);
  void setDragPlane(int plane);
  void setDragMode(int mode);
  void resetCamera();
  void toggleCameraMode();
  void fitCamera();
  void toggleRenderFlag(int flag);
  void exportAsImage(int resolution);
};
