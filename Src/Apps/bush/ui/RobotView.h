#pragma once

#include <QGroupBox>
#include <map>
#include "Session.h"
#include "models/Status.h"

struct Robot;
class TeamSelector;
class QMouseEvent;
class QDropEvent;
class QDragEnterEvent;
class QLabel;
class QProgressBar;
class QWidget;

class RobotView : public QGroupBox
{
  Q_OBJECT

  TeamSelector* teamSelector;
  Robot* robot;
  unsigned short playerNumber;
  unsigned short pos;
  QLabel* cPlayerNumber = nullptr;
  QWidget* statusWidget;
  QLabel* pingBarWLAN;
  QLabel* pingBarLAN;
  QProgressBar* powerBar;
  QLabel* logStatus;
  QLabel* dumpStatus;
public:
  RobotView(TeamSelector* teamSelector,
            Robot* robot,
            unsigned short playerNumber = 0,
            unsigned short position = 0);
  void update();
  void setRobot(Robot* robot);
  unsigned short getPlayerNumber() const { return playerNumber; }
  QString getRobotName() const;
  bool isSelected() const;
protected:
  void mouseMoveEvent(QMouseEvent* me);
  void dragEnterEvent(QDragEnterEvent* e);
  void dropEvent(QDropEvent* e);
public slots:
  void setSelected(bool selected);
private slots:
  void setPing(ENetwork network, const Robot* robot, double ping);
  void setStatus(const Robot* robot, const Status* status);
signals:
  void robotChanged();
};
