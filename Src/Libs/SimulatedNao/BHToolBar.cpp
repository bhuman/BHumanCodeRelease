#include "BHToolBar.h"
#include "ConsoleRoboCupCtrl.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"

#include <QMenu>
#include <QTimer>

QMenu* BHToolBar::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("B-Human"));

  QIcon standIcon(":/Icons/icons8-standing-man-50.png");
  standIcon.setIsMask(true);
  QAction* standAction = new QAction(standIcon, tr("&MotionRequest stand"), menu);
  standAction->setCheckable(true);
  standAction->setChecked(currentMotion == stand);

  QIcon sitDownIcon(":/Icons/icons8-squats-50.png");
  sitDownIcon.setIsMask(true);
  QAction* sitDownAction = new QAction(sitDownIcon, tr("&MotionRequest sitDown"), menu);
  sitDownAction->setCheckable(true);
  sitDownAction->setChecked(currentMotion == sitDown);

  QIcon headAngleIcon(":/Icons/icons8-drag-50.png");
  headAngleIcon.setIsMask(true);
  QAction* headAngleAct = new QAction(headAngleIcon, tr("Move Head &Freely"), menu);
  headAngleAct->setCheckable(true);
  headAngleAct->setChecked(headMotionRequest);

  connect(standAction, &QAction::triggered, this, &BHToolBar::requestStand);
  connect(sitDownAction, &QAction::triggered, this, &BHToolBar::requestSitDown);
  connect(headAngleAct, &QAction::toggled, this, &BHToolBar::headAngle);

  // if the following order is changed, also remember to change in BHToolBar::getActionsFromParent()
  menu->addAction(standAction);
  menu->addAction(sitDownAction);
  menu->addSeparator();
  menu->addAction(headAngleAct);
  return menu;
}

void BHToolBar::requestStand(bool active)
{
  QAction* pAction = qobject_cast<QAction*>(sender());
  QWidget* menu = qobject_cast<QWidget*>(pAction->parent());
  QAction* standAction = nullptr;
  QAction* sitDownAction = nullptr;
  if(menu)
    getActionsFromParent(standAction, sitDownAction, menu);
  if(active && currentMotion != stand)
  {
    setStand();
    if(sitDownAction)
      sitDownAction->setChecked(false);
  }
  else if(!active && currentMotion == stand)
    setNone();
}

void BHToolBar::requestSitDown(bool active)
{
  QAction* pAction = qobject_cast<QAction*>(sender());
  QWidget* menu = qobject_cast<QWidget*>(pAction->parent());
  QAction* standAction = nullptr;
  QAction* sitDownAction = nullptr;
  if(menu)
    getActionsFromParent(standAction, sitDownAction, menu);
  if(active && currentMotion != sitDown)
  {
    setSitDown();
    if(standAction)
      standAction->setChecked(false);
  }
  else if(!active && currentMotion == sitDown)
    setNone();
}

void BHToolBar::setNone()
{
  currentMotion = none;
  console.executeConsoleCommand("set representation:MotionRequest unchanged");
}

void BHToolBar::setSitDown()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::playDead;

  console.setRepresentation("MotionRequest", moReq);

  currentMotion = sitDown;
}

void BHToolBar::setStand()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::stand;
  moReq.standHigh = true;

  console.setRepresentation("MotionRequest", moReq);

  currentMotion = stand;
}

void BHToolBar::headAngle(bool active)
{
  if(active)
  {
    HeadMotionRequest hmreq;
    hmreq.mode = HeadMotionRequest::panTiltMode;
    hmreq.cameraControlMode = HeadMotionRequest::autoCamera;
    hmreq.pan = JointAngles::off;
    hmreq.tilt = JointAngles::off;
    hmreq.speed = 150_deg;
    console.setRepresentation("HeadMotionRequest", hmreq);
  }
  else
    console.executeConsoleCommand("set representation:HeadMotionRequest unchanged");

  headMotionRequest = active;
}

void BHToolBar::getActionsFromParent(QAction*& standAction, QAction*& sitDownAction, QWidget* parent)
{
  standAction = parent->actions().at(0);
  sitDownAction = parent->actions().at(1);
}
