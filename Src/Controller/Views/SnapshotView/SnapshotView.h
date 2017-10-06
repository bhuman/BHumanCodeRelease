#pragma once

#include <SimRobot.h>
#include <string>
#include <QIcon>

#include "Controller/RobotConsole.h"

class RobotConsole;
class QWidget;

class SnapshotView : public SimRobot::Object
{
public:
  SnapshotView(const QString& fullName, RobotConsole& console);

private:
  const QString fullName;
  const QIcon icon;
  RobotConsole& console;

  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  friend class SnapshotWidget;
};
