/**
 * @file Controller/BHToolBar.h
 *
 * B-HumanTool with awesome buttons.
 *
 * @author Florian Maaß
 */

#pragma once

#include <QAction>
#include <QObject>
#include <QTime>

class ConsoleRoboCupCtrl;
class QMenu;

class BHToolBar : public QObject
{
  Q_OBJECT

  ConsoleRoboCupCtrl& console;

public:
  BHToolBar(ConsoleRoboCupCtrl& console) : console(console) {}

  QMenu* createUserMenu() const;

private:
  QWidget* menu = nullptr;

  enum Motion { none, preUp, standUp, standing, sitDown, isDown };
  volatile Motion currentMotion = none;
  volatile bool motionButtonsDisabled = false;
  volatile bool headAngleRequest = false;

  void disableMotionButtons(QAction* standAction, QAction* sitDownAction);
  void enableMotionButtons(QAction* standAction, QAction* sitDownAction);
  void getActionsFromParent(QAction*& standAction, QAction*& sitDownAction, QWidget* parent);
  QTime lastRequest;

private slots:
  void unsetMenuPointer();
  void enableMotionButtons();
  void requestStand(bool active);
  void requestSitDown(bool active);
  void setPlayDead();
  void setStand();
  void setSitDown();
  void setMotionToStanding();
  void headAngle(bool active);
};
