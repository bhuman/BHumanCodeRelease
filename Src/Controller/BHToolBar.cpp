#include "BHToolBar.h"
#include "ConsoleRoboCupCtrl.h"
#include "Platform/SystemCall.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"

#include <QMenu>
#include <QTimer>

QMenu* BHToolBar::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("B-Human"));
  connect(menu, SIGNAL(destroyed()), this, SLOT(unsetMenuPointer()));

  QAction* standAction = new QAction(QIcon(":/Icons/stand.png"), tr("&MotionRequest stand"), menu);
  standAction->setCheckable(true);
  standAction->setChecked(false);

  QAction* sitDownAction = new QAction(QIcon(":/Icons/sitDown.png"), tr("&MotionRequest sitDown"), menu);
  sitDownAction->setCheckable(true);
  sitDownAction->setChecked(false);

  if(currentMotion == preUp || currentMotion == standUp || currentMotion == standing)
    standAction->setChecked(true);
  else if(currentMotion == sitDown || currentMotion == isDown)
    sitDownAction->setChecked(true);

  standAction->setDisabled(motionButtonsDisabled);
  sitDownAction->setDisabled(motionButtonsDisabled);

  QAction* headAngleAct = new QAction(QIcon(":/Icons/headAngle.png"), tr("Move Head &Freely"), menu);
  headAngleAct->setCheckable(true);
  headAngleAct->setChecked(headAngleRequest);

  connect(standAction, SIGNAL(triggered(bool)), this, SLOT(requestStand(bool)));
  connect(sitDownAction, SIGNAL(triggered(bool)), this, SLOT(requestSitDown(bool)));
  connect(headAngleAct, SIGNAL(toggled(bool)), this, SLOT(headAngle(bool)));

  // if the following order is changed, also remember to change in BHToolBar::getActionsFromParent()
  menu->addAction(standAction);
  menu->addAction(sitDownAction);
  menu->addSeparator();
  menu->addAction(headAngleAct);
  return menu;
}

void BHToolBar::unsetMenuPointer()
{
  menu = nullptr;
  if(!motionButtonsDisabled && (
       currentMotion == none || currentMotion == standing || currentMotion == isDown ||
       (lastRequest.isValid() && lastRequest.secsTo(QTime::currentTime()) >= 3)))
    motionButtonsDisabled = false;
}

void BHToolBar::enableMotionButtons()
{
  motionButtonsDisabled = false;
  if(!menu)
    return;
  QAction* standAction = nullptr;
  QAction* sitDownAction = nullptr;
  getActionsFromParent(standAction, sitDownAction, menu);
  if(sitDownAction && sitDownAction)
    enableMotionButtons(standAction, sitDownAction);
}

void BHToolBar::requestStand(bool active)
{
  motionButtonsDisabled = true;
  QAction* pAction = qobject_cast<QAction*>(sender());
  menu = pAction->parentWidget();
  QAction* standAction = nullptr;
  QAction* sitDownAction = nullptr;
  if(menu)
    getActionsFromParent(standAction, sitDownAction, menu);
  if(standAction && sitDownAction)
    disableMotionButtons(standAction, sitDownAction);
  //did nothing, first request
  if(currentMotion == none && active)
  {
    lastRequest = QTime::currentTime();
    currentMotion = preUp;
    //TODO: robot might jump after sitDown (workaround: playDead before stand) //this is bad, robot might fall if standing (due to lag)
    setPlayDead();
    QTimer::singleShot(200, this, SLOT(setStand()));
  }
  //last request was stand, enable actions
  else if(currentMotion == standing && !active)
  {
    currentMotion = none;
    console.executeConsoleCommand("set representation:MotionRequest unchanged");
    QTimer::singleShot(300, this, SLOT(enableMotionButtons()));
  }
  //last request was sitDown, uncheck sitDownAction
  else if(currentMotion == isDown && active)
  {
    lastRequest = QTime::currentTime();
    currentMotion = preUp;
    if(sitDownAction)
      sitDownAction->setChecked(false);
    //TODO: robot might jump after sitDown (workaround: playDead before stand) //this is bad, robot might fall if standing (due to lag)
    setPlayDead();
    QTimer::singleShot(200, this, SLOT(setStand()));
  }
  //between states
  else if(currentMotion == preUp || currentMotion == standUp || currentMotion == sitDown)
  {
    if(standAction)
      standAction->setChecked(!active);
  }
  //what the?
  else
  {
    motionButtonsDisabled = false;
    currentMotion = none;
    console.executeConsoleCommand("set representation:MotionRequest unchanged");
    if(standAction)
      standAction->setChecked(!active);
    QTimer::singleShot(300, this, SLOT(enableMotionButtons()));
  }
}

void BHToolBar::requestSitDown(bool active)
{
  motionButtonsDisabled = true;
  QAction* pAction = qobject_cast<QAction*>(sender());
  menu = pAction->parentWidget();
  QAction* standAction = nullptr;
  QAction* sitDownAction = nullptr;
  if(menu)
    getActionsFromParent(standAction, sitDownAction, menu);
  if(standAction && sitDownAction)
    disableMotionButtons(standAction, sitDownAction);
  //did nothing, first request
  if(currentMotion == none && active)
  {
    lastRequest = QTime::currentTime();
    currentMotion = sitDown;
    setSitDown();
    QTimer::singleShot(4000, this, SLOT(setPlayDead()));
  }
  //last request was sitDown, enable actions
  else if(currentMotion == isDown && !active)
  {
    currentMotion = none;
    console.executeConsoleCommand("set representation:MotionRequest unchanged");
    QTimer::singleShot(300, this, SLOT(enableMotionButtons()));
  }
  //last request was stand, uncheck standAction
  else if(currentMotion == standing && active)
  {
    lastRequest = QTime::currentTime();
    currentMotion = sitDown;
    if(standAction)
      standAction->setChecked(false);
    setSitDown();
    QTimer::singleShot(4000, this, SLOT(setPlayDead()));
  }
  //between states
  else if(currentMotion == sitDown || currentMotion == preUp || currentMotion == standUp)
  {
    if(sitDownAction)
      sitDownAction->setChecked(!active);
  }
  //what the?
  else
  {
    motionButtonsDisabled = false;
    currentMotion = none;
    console.executeConsoleCommand("set representation:MotionRequest unchanged");
    if(sitDownAction)
      sitDownAction->setChecked(!active);
    QTimer::singleShot(300, this, SLOT(enableMotionButtons()));
  }
}

void BHToolBar::setPlayDead()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::playDead;

  console.setRepresentation("MotionRequest", moReq);

  if(currentMotion == sitDown)
  {
    currentMotion = isDown;

    QTimer::singleShot(300, this, SLOT(enableMotionButtons()));
  }
}

void BHToolBar::setSitDown()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::sitDown;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::setStand()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::stand;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::playDead;

  console.setRepresentation("MotionRequest", moReq);

  currentMotion = standUp;

  QTimer::singleShot(3000, this, SLOT(setMotionToStanding()));
  QTimer::singleShot(4000, this, SLOT(enableMotionButtons()));
}

void BHToolBar::headAngle(bool active)
{
  if(active)
  {
    HeadAngleRequest hareq;
    hareq.pan = JointAngles::off;
    hareq.tilt = JointAngles::off;
    hareq.speed = 150_deg;
    console.setRepresentation("HeadAngleRequest", hareq);
  }
  else
    console.executeConsoleCommand("set representation:HeadAngleRequest unchanged");

  headAngleRequest = active;
}

void BHToolBar::setMotionToStanding()
{
  currentMotion = standing;
}

void BHToolBar::disableMotionButtons(QAction* standAction, QAction* sitDownAction)
{
  standAction->setEnabled(false);
  sitDownAction->setEnabled(false);
}

void BHToolBar::enableMotionButtons(QAction* standAction, QAction* sitDownAction)
{
  standAction->setEnabled(true);
  sitDownAction->setEnabled(true);
}

void BHToolBar::getActionsFromParent(QAction*& standAction, QAction*& sitDownAction, QWidget* parent)
{
  standAction = parent->actions().at(0);
  sitDownAction = parent->actions().at(1);
}
