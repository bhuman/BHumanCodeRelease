/**
 * @file SimObjectWidget.cpp
 * Implementation of class SimObjectWidget
 * @author Colin Graf
 */

#include <QMouseEvent>
#include <QApplication>
#include <QSettings>
#include <QMenu>
#include <QPinchGesture>
#include <QSignalMapper>
#include <QClipboard>
#include <QFileDialog>

#include "SimObjectWidget.h"
#include "CoreModule.h"
#include "Platform/Assert.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"

SimObjectWidget::SimObjectWidget(SimObject& simObject) : QGLWidget(QGLFormat(QGL::SampleBuffers), 0, Simulation::simulation->renderer.getWidget()),
  object(dynamic_cast<SimRobot::Object&>(simObject)), objectRenderer(simObject),
  wkey(false), akey(false), skey(false), dkey(false)
{
  setFocusPolicy(Qt::StrongFocus);
  grabGesture(Qt::PinchGesture);
  setAttribute(Qt::WA_AcceptTouchEvents);

  // load layout settings
  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(object.getFullName());

  objectRenderer.setSurfaceShadeMode(SimRobotCore2::Renderer::ShadeMode(settings->value("SurfaceShadeMode", int(objectRenderer.getSurfaceShadeMode())).toInt()));
  objectRenderer.setPhysicsShadeMode(SimRobotCore2::Renderer::ShadeMode(settings->value("PhysicsShadeMode", int(objectRenderer.getPhysicsShadeMode())).toInt()));
  objectRenderer.setDrawingsShadeMode(SimRobotCore2::Renderer::ShadeMode(settings->value("DrawingsShadeMode", int(objectRenderer.getDrawingsShadeMode())).toInt()));
  objectRenderer.setCameraMode(SimRobotCore2::Renderer::CameraMode(settings->value("CameraMode", int(objectRenderer.getCameraMode())).toInt()));
  fovy = settings->value("FovY", objectRenderer.getFovY()).toInt();
  objectRenderer.setDragPlane(SimRobotCore2::Renderer::DragAndDropPlane(settings->value("DragPlane", int(objectRenderer.getDragPlane())).toInt()));
  objectRenderer.setDragMode(SimRobotCore2::Renderer::DragAndDropMode(settings->value("DragMode", int(objectRenderer.getDragMode())).toInt()));
  objectRenderer.setRenderFlags(settings->value("RenderFlags", objectRenderer.getRenderFlags()).toInt());

  float pos[3];
  float target[3];
  objectRenderer.getCamera(pos, target);
  pos[0] = settings->value("cameraPosX", pos[0]).toFloat();
  pos[1] = settings->value("cameraPosY", pos[1]).toFloat();
  pos[2] = settings->value("cameraPosZ", pos[2]).toFloat();
  target[0] = settings->value("cameraTargetX", target[0]).toFloat();
  target[1] = settings->value("cameraTargetY", target[1]).toFloat();
  target[2] = settings->value("cameraTargetZ", target[2]).toFloat();
  objectRenderer.setCamera(pos, target);

  settings->endGroup();
}

SimObjectWidget::~SimObjectWidget()
{
  // save layout settings
  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(object.getFullName());

  settings->setValue("SurfaceShadeMode", int(objectRenderer.getSurfaceShadeMode()));
  settings->setValue("PhysicsShadeMode", int(objectRenderer.getPhysicsShadeMode()));
  settings->setValue("DrawingsShadeMode", int(objectRenderer.getDrawingsShadeMode()));
  settings->setValue("CameraMode", int(objectRenderer.getCameraMode()));
  settings->setValue("FovY", objectRenderer.getFovY());
  settings->setValue("DragPlane", int(objectRenderer.getDragPlane()));
  settings->setValue("DragMode", int(objectRenderer.getDragMode()));
  settings->setValue("RenderFlags", objectRenderer.getRenderFlags());

  float pos[3];
  float target[3];
  objectRenderer.getCamera(pos, target);

  settings->setValue("cameraPosX", pos[0]);
  settings->setValue("cameraPosY", pos[1]);
  settings->setValue("cameraPosZ", pos[2]);
  settings->setValue("cameraTargetX", target[0]);
  settings->setValue("cameraTargetY", target[1]);
  settings->setValue("cameraTargetZ", target[2]);

  settings->endGroup();
}

void SimObjectWidget::initializeGL()
{
  objectRenderer.init(isSharing());
}

void SimObjectWidget::paintGL()
{
  objectRenderer.draw();
}

void SimObjectWidget::resizeGL(int width, int height)
{
  objectRenderer.resize(fovy, width, height);
}

void SimObjectWidget::mouseMoveEvent(QMouseEvent* event)
{
  QGLWidget::mouseMoveEvent(event);
  const Qt::KeyboardModifiers m = QApplication::keyboardModifiers();
  if(objectRenderer.moveDrag(event->x() * devicePixelRatio(), event->y() * devicePixelRatio(), m & Qt::ShiftModifier ? (m & Qt::ControlModifier ? SimObjectRenderer::dragRotateWorld : SimObjectRenderer::dragRotate) : (m & Qt::ControlModifier ? SimObjectRenderer::dragNormalObject : SimObjectRenderer::dragNormal)))
  {
    event->accept();
    update();
  }
}

void SimObjectWidget::mousePressEvent(QMouseEvent* event)
{
  QGLWidget::mousePressEvent(event);

  if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
  {
    const Qt::KeyboardModifiers m = QApplication::keyboardModifiers();
    if(objectRenderer.startDrag(event->x() * devicePixelRatio(), event->y() * devicePixelRatio(), m & Qt::ShiftModifier ? (m & Qt::ControlModifier ? SimObjectRenderer::dragRotateWorld : SimObjectRenderer::dragRotate) : (m & Qt::ControlModifier ? SimObjectRenderer::dragNormalObject : SimObjectRenderer::dragNormal)))
    {
      event->accept();
      update();
    }
  }
}

void SimObjectWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QGLWidget::mouseReleaseEvent(event);

  if(objectRenderer.releaseDrag(event->x() * devicePixelRatio(), event->y() * devicePixelRatio()))
  {
    event->accept();
    update();
  }
}

void SimObjectWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  QGLWidget::mouseDoubleClickEvent(event);

  if(event->button() == Qt::LeftButton)
  {
    SimRobotCore2::Object* selectedObject = objectRenderer.getDragSelection();
    if(selectedObject)
      CoreModule::application->selectObject(*selectedObject);
  }
}

void SimObjectWidget::keyPressEvent(QKeyEvent* event)
{
  if(event->modifiers() != 0)
  {
    QGLWidget::keyPressEvent(event);
    return;
  }

  switch(event->key())
  {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      event->accept();
      objectRenderer.zoom(-100., -1, -1);
      update();
      break;

    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      event->accept();
      objectRenderer.zoom(100., -1, -1);
      update();
      break;

    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
      event->accept();
      switch(event->key())
      {
        case Qt::Key_W:
          wkey = true;
          break;
        case Qt::Key_A:
          akey = true;
          break;
        case Qt::Key_S:
          skey = true;
          break;
        case Qt::Key_D:
          dkey = true;
          break;
      }
      objectRenderer.setCameraMove(akey, dkey, wkey, skey);
      update();
      break;

    default:
      QGLWidget::keyPressEvent(event);
      break;
  }
}

void SimObjectWidget::keyReleaseEvent(QKeyEvent* event)
{
  if(event->modifiers() != 0)
  {
    QGLWidget::keyReleaseEvent(event);
    return;
  }

  switch(event->key())
  {
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
      event->accept();
      update();
      if(!event->isAutoRepeat())
      {
        switch(event->key())
        {
          case Qt::Key_W:
            wkey = false;
            break;
          case Qt::Key_A:
            akey = false;
            break;
          case Qt::Key_S:
            skey = false;
            break;
          case Qt::Key_D:
            dkey = false;
            break;
        }
        objectRenderer.setCameraMove(akey, dkey, wkey, skey);
      }
      break;

    default:
      QGLWidget::keyReleaseEvent(event);
      break;
  }
}

bool SimObjectWidget::event(QEvent* event)
{
  if(event->type() == QEvent::Gesture)
  {
    QPinchGesture* pinch = static_cast<QPinchGesture*>(static_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture));
    if(pinch && (pinch->changeFlags() & QPinchGesture::ScaleFactorChanged))
    {
#ifdef FIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      pinch->setLastScaleFactor(1.f);
#endif
      float change = static_cast<float>(pinch->scaleFactor() > pinch->lastScaleFactor()
                                        ? -pinch->scaleFactor() / pinch->lastScaleFactor()
                                        : pinch->lastScaleFactor() / pinch->scaleFactor());
      objectRenderer.zoom(change * 100.f, -1, -1);
      update();
      return true;
    }
  }
  return QGLWidget::event(event);
}

void SimObjectWidget::wheelEvent(QWheelEvent* event)
{
#ifndef MACOS
  if(event->orientation() == Qt::Vertical)
  {
    objectRenderer.zoom(event->delta(), event->x() * devicePixelRatio(), event->y() * devicePixelRatio());
    update();
    event->accept();
    return;
  }
#else
  if(event->delta())
  {
    if(event->orientation() == Qt::Horizontal)
      objectRenderer.rotateCamera(static_cast<float>(event->delta()) * -0.002f, 0.f);
    else
      objectRenderer.rotateCamera(0.f, static_cast<float>(event->delta()) * -0.002f);
    update();
    return;
  }
#endif
  QGLWidget::wheelEvent(event);
}

void SimObjectWidget::update()
{
  QGLWidget::update();
}

QMenu* SimObjectWidget::createEditMenu() const
{
  QMenu* menu = new QMenu(tr("&Edit"));

  QAction* action = menu->addAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"));
  action->setShortcut(QKeySequence(QKeySequence::Copy));
  action->setStatusTip(tr("Copy the rendered object to the clipboard"));
  connect(action, SIGNAL(triggered()), this, SLOT(copy()));

  return menu;
}

QMenu* SimObjectWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr(&object == Simulation::simulation->scene ? "S&cene" : "&Object"));

  QSignalMapper* flagsSignalMapper = new QSignalMapper(menu);
  connect(flagsSignalMapper, SIGNAL(mapped(int)), SLOT(toggleRenderFlag(int)));

  {
    QMenu* subMenu = menu->addMenu(tr("&Drag and Drop"));
    QAction* action = subMenu->menuAction();
    action->setIcon(QIcon(":/Icons/DragPlane.png"));
    action->setStatusTip(tr("Select the drag and drop dynamics mode and plane along which operations are performed"));
    QActionGroup* actionGroup = new QActionGroup(subMenu);
    QSignalMapper* signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setDragPlane(int)));
    action = subMenu->addAction(tr("X/Y Plane"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::xyPlane);
    action->setShortcut(QKeySequence(Qt::Key_Z));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("X/Z Plane"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::xzPlane);
    action->setShortcut(QKeySequence(Qt::Key_Y));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("Y/Z Plane"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::yzPlane);
    action->setShortcut(QKeySequence(Qt::Key_X));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getDragPlane()))->setChecked(true);
    subMenu->addSeparator();
    actionGroup = new QActionGroup(subMenu);
    signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setDragMode(int)));
    action = subMenu->addAction(tr("&Keep Dynamics"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::keepDynamics);
    action->setShortcut(QKeySequence(Qt::Key_7));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Reset Dynamics"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::resetDynamics);
    action->setShortcut(QKeySequence(Qt::Key_8));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("A&dopt Dynamics"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::adoptDynamics);
    action->setShortcut(QKeySequence(Qt::Key_9));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Apply Dynamics"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::applyDynamics);
    action->setShortcut(QKeySequence(Qt::Key_0));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getDragMode()))->setChecked(true);
  }

  menu->addSeparator();

  {
    QAction* action = menu->menuAction();
    action = menu->addAction(tr("&Reset Camera"));
    action->setIcon(QIcon(":/Icons/camera.png"));
    action->setShortcut(QKeySequence(Qt::Key_R));
    connect(action, SIGNAL(triggered()), this, SLOT(resetCamera()));

    /*
    action = subMenu->addAction(tr("&Fit"));
    action->setShortcut(QKeySequence(Qt::Key_F));
    connect(action, SIGNAL(triggered()), this, SLOT(fitCamera()));
    subMenu->addSeparator();
     */
  }

  {
    QMenu* subMenu = menu->addMenu(tr("&Vertical Opening Angle"));
    QAction* action = subMenu->menuAction();
    action->setIcon(QIcon(":/Icons/opening_angle.png"));
    QActionGroup* actionGroup = new QActionGroup(subMenu);
    QSignalMapper* signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setFovY(int)));
    action = subMenu->addAction(tr("&20°"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, 20);
    action->setShortcut(QKeySequence(Qt::Key_1));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&40°"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, 40);
    action->setShortcut(QKeySequence(Qt::Key_2));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&60°"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, 60);
    action->setShortcut(QKeySequence(Qt::Key_3));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&80°"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, 80);
    action->setShortcut(QKeySequence(Qt::Key_4));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("100°"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, 100);
    action->setShortcut(QKeySequence(Qt::Key_5));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("120°"));
    actionGroup->addAction(action);
    signalMapper->setMapping(action, 120);
    action->setShortcut(QKeySequence(Qt::Key_6));
    action->setCheckable(true);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getFovY()))->setChecked(true);
  }

  menu->addSeparator();

  {
    QMenu* subMenu = menu->addMenu(tr("&Appearances Rendering"));
    QActionGroup* actionGroup = new QActionGroup(subMenu);
    QSignalMapper* signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setSurfaceShadeMode(int)));
    QAction* action = subMenu->menuAction();
    action->setIcon(QIcon(":/Icons/layers.png"));
    action->setStatusTip(tr("Select different shading techniques for displaying the scene"));
    action = subMenu->addAction(tr("&Off"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::noShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Wire Frame"));
    actionGroup->addAction(action);
    action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::wireframeShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Flat Shading"));
    actionGroup->addAction(action);
    action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_F));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::flatShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Smooth Shading"));
    actionGroup->addAction(action);
    action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_M));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::smoothShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getSurfaceShadeMode()))->setChecked(true);
  }

  {
    QMenu* subMenu = menu->addMenu(tr("&Physics Rendering"));
    QActionGroup* actionGroup = new QActionGroup(subMenu);
    QSignalMapper* signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setPhysicsShadeMode(int)));
    QAction* action = subMenu->menuAction();
    //action->setIcon(QIcon(":/Icons/layers.png"));
    action->setStatusTip(tr("Select different shading techniques for displaying the physical representation of objects"));
    action = subMenu->addAction(tr("&Off"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::noShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Wire Frame"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::wireframeShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Flat Shading"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_F));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::flatShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Smooth Shading"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_M));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::smoothShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getPhysicsShadeMode()))->setChecked(true);
  }

  {
    QMenu* subMenu = menu->addMenu(tr("&Drawings Rendering"));
    QActionGroup* actionGroup = new QActionGroup(subMenu);
    QSignalMapper* signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setDrawingsShadeMode(int)));
    QAction* action = subMenu->menuAction();
    action->setIcon(QIcon(":/Icons/chart_line.png"));
    action->setStatusTip(tr("Select different shading techniques for displaying controller drawings"));
    action = subMenu->addAction(tr("&Off"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::noShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Wire Frame"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_W));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::wireframeShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Flat Shading"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_F));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::flatShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("&Smooth Shading"));
    actionGroup->addAction(action);
    //action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_M));
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::smoothShading);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getDrawingsShadeMode()))->setChecked(true);

    subMenu->addSeparator();

    subMenu = subMenu->addMenu(tr("&Occlusion"));
    actionGroup = new QActionGroup(subMenu);
    signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(setDrawingsOcclusion(int)));
    action = subMenu->menuAction();
    action->setStatusTip(tr("Select different drawings occlusion modes"));

    action = subMenu->addAction(tr("&On"));
    actionGroup->addAction(action);
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::enableDrawingsOcclusion);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));

    action = subMenu->addAction(tr("O&ff"));
    actionGroup->addAction(action);
    action->setCheckable(true);
    signalMapper->setMapping(action, 0);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));

    action = subMenu->addAction(tr("&Transparent"));
    actionGroup->addAction(action);
    action->setCheckable(true);
    signalMapper->setMapping(action, SimRobotCore2::Renderer::enableDrawingsTransparentOcclusion);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));

    qobject_cast<QAction*>(signalMapper->mapping(objectRenderer.getRenderFlags() & (SimRobotCore2::Renderer::enableDrawingsOcclusion | SimRobotCore2::Renderer::enableDrawingsTransparentOcclusion)))->setChecked(true);
  }

  menu->addSeparator();

  QAction* action = menu->addAction(tr("Enable &Lights"));
  action->setStatusTip(tr("Enable lightning"));
  action->setCheckable(true);
  action->setChecked(objectRenderer.getRenderFlags() & SimRobotCore2::Renderer::enableLights);
  flagsSignalMapper->setMapping(action, SimRobotCore2::Renderer::enableLights);
  connect(action, SIGNAL(triggered()), flagsSignalMapper, SLOT(map()));

  action = menu->addAction(tr("Enable &Textures"));
  action->setStatusTip(tr("Enable textures"));
  action->setCheckable(true);
  action->setChecked(objectRenderer.getRenderFlags() & SimRobotCore2::Renderer::enableTextures);
  flagsSignalMapper->setMapping(action, SimRobotCore2::Renderer::enableTextures);
  connect(action, SIGNAL(triggered()), flagsSignalMapper, SLOT(map()));

  action = menu->addAction(tr("Enable &Multisample"));
  action->setStatusTip(tr("Enable multisampling"));
  action->setCheckable(true);
  action->setChecked(objectRenderer.getRenderFlags() & SimRobotCore2::Renderer::enableMultisample);
  flagsSignalMapper->setMapping(action, SimRobotCore2::Renderer::enableMultisample);
  connect(action, SIGNAL(triggered()), flagsSignalMapper, SLOT(map()));

  menu->addSeparator();

  action = menu->addAction(tr("Show &Coordinate System"));
  action->setStatusTip(tr("Show the coordinate system of the displayed object"));
  action->setCheckable(true);
  action->setChecked(objectRenderer.getRenderFlags() & SimRobotCore2::Renderer::showCoordinateSystem);
  flagsSignalMapper->setMapping(action, SimRobotCore2::Renderer::showCoordinateSystem);
  connect(action, SIGNAL(triggered()), flagsSignalMapper, SLOT(map()));

  action = menu->addAction(QIcon(":/Icons/transmit_go.png"), tr("Show &Sensors"));
  action->setStatusTip(tr("Show the values of the sensors in the scene view"));
  action->setCheckable(true);
  action->setChecked(objectRenderer.getRenderFlags() & SimRobotCore2::Renderer::showSensors);
  flagsSignalMapper->setMapping(action, SimRobotCore2::Renderer::showSensors);
  connect(action, SIGNAL(triggered()), flagsSignalMapper, SLOT(map()));

  menu->addSeparator();

  {
    QMenu* subMenu = menu->addMenu(tr("Export as Image..."));
    QSignalMapper* signalMapper = new QSignalMapper(subMenu);
    connect(signalMapper, SIGNAL(mapped(int)), SLOT(exportAsImage(int)));
    action = subMenu->addAction(tr("3840x2160"));
    signalMapper->setMapping(action, (3840 << 16) | 2160);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("2880x1620"));
    signalMapper->setMapping(action, (2880 << 16) | 1620);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("1920x1080"));
    signalMapper->setMapping(action, (1920 << 16) | 1080);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    action = subMenu->addAction(tr("1280x1024"));
    signalMapper->setMapping(action, (1280 << 16) | 1024);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
  }

  return menu;
}
void SimObjectWidget::copy()
{
  QApplication::clipboard()->clear();
  QApplication::clipboard()->setPixmap(QPixmap::fromImage(grabFrameBuffer()));
}

void SimObjectWidget::exportAsImage(int resolution)
{
  int width = resolution >> 16;
  int height = resolution & 0xffff;

  QSettings& settings = CoreModule::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                                                  tr("Export as Image"), settings.value("ExportDirectory", "").toString(), tr("Portable Network Graphic (*.png)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  QImage image;
  {
    unsigned int winWidth, winHeight;
    objectRenderer.getSize(winWidth, winHeight);

    // render object using a temporary widget
    QGLFormat format = this->format();
    format.setDoubleBuffer(false);
    QGLWidget widget(format, 0, 0, Qt::CustomizeWindowHint);
    widget.setWindowOpacity(0.f);
    widget.show();
    widget.setMaximumSize(width / devicePixelRatio(), height / devicePixelRatio());
    widget.resize(width / devicePixelRatio(), height / devicePixelRatio());
    widget.makeCurrent();
    objectRenderer.resize(fovy, width, height);
    objectRenderer.init(false);
    objectRenderer.draw();
    image = widget.grabFrameBuffer();

    makeCurrent();
    objectRenderer.resize(fovy, winWidth, winHeight);
  }

  image.save(fileName);
}

void SimObjectWidget::setSurfaceShadeMode(int style)
{
  objectRenderer.setSurfaceShadeMode(SimRobotCore2::Renderer::ShadeMode(style));
  update();
}

void SimObjectWidget::setPhysicsShadeMode(int style)
{
  objectRenderer.setPhysicsShadeMode(SimRobotCore2::Renderer::ShadeMode(style));
  update();
}

void SimObjectWidget::setDrawingsShadeMode(int style)
{
  objectRenderer.setDrawingsShadeMode(SimRobotCore2::Renderer::ShadeMode(style));
  update();
}

void SimObjectWidget::setDrawingsOcclusion(int flag)
{
  unsigned int flags = objectRenderer.getRenderFlags();
  flags &= ~(SimRobotCore2::Renderer::enableDrawingsOcclusion | SimRobotCore2::Renderer::enableDrawingsTransparentOcclusion);
  flags |= flag;
  objectRenderer.setRenderFlags(flags);
  update();
}

void SimObjectWidget::setCameraMode(int mode)
{
  objectRenderer.setCameraMode(SimRobotCore2::Renderer::CameraMode(mode));
  update();
}

void SimObjectWidget::setFovY(int fovy)
{
  unsigned int width, height;
  this->fovy = fovy;
  objectRenderer.getSize(width, height);
  makeCurrent();
  objectRenderer.resize(fovy, width, height);
  update();
}

void SimObjectWidget::setDragPlane(int plane)
{
  objectRenderer.setDragPlane(SimRobotCore2::Renderer::DragAndDropPlane(plane));
  update();
}

void SimObjectWidget::setDragMode(int mode)
{
  objectRenderer.setDragMode(SimRobotCore2::Renderer::DragAndDropMode(mode));
  update();
}

void SimObjectWidget::resetCamera()
{
  objectRenderer.resetCamera();
  update();
}

void SimObjectWidget::toggleCameraMode()
{
  objectRenderer.toggleCameraMode();
  update();
}

void SimObjectWidget::fitCamera()
{
  /*
  objectRenderer.fitCamera();
  update();
   */
}

void SimObjectWidget::toggleRenderFlag(int flag)
{
  unsigned int flags = objectRenderer.getRenderFlags();
  if(flags & flag)
    flags &= ~flag;
  else
    flags |= flag;
  objectRenderer.setRenderFlags(flags);

  update();
}
