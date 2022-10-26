/**
 * @file SimulatedNao/BHToolBar.h
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
  enum Motion { none, stand, sitDown };
  Motion currentMotion = none;
  bool headMotionRequest = false;

  void getActionsFromParent(QAction*& standAction, QAction*& sitDownAction, QWidget* parent);
  void setNone();
  void setSitDown();
  void setStand();

private slots:
  void requestStand(bool active);
  void requestSitDown(bool active);
  void headAngle(bool active);
};
