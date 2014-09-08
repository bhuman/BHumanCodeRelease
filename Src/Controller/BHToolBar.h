/**
* @file Controller/BHToolBar.h
*
* B-HumanTool with awesome buttons.
*
* @author Florian Maaß
*/

#pragma once

#include <QMenu>

class ConsoleRoboCupCtrl;

class BHToolBar : public QObject
{
  Q_OBJECT

public:
  BHToolBar(ConsoleRoboCupCtrl& console) : console(console) {}

  QMenu* createUserMenu() const;

private:
  ConsoleRoboCupCtrl& console;

private slots:
  void stand();
  void sitDown();
  void setPlayDead();
  void setStand();
  void setSitDown();
  void headAngle(bool active);
  void pressChestButton();
  void releaseChestButton();
  void unchangeButtons();
};
