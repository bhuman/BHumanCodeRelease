#include "Controller/RobotConsole.h"
#include "SnapshotView.h"
#include "SnapshotWidget.h"

#include <QString>

SnapshotView::SnapshotView(const QString& fullName, RobotConsole& console) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console)
{}

SimRobot::Widget* SnapshotView::createWidget()
{
  return new SnapshotWidget(*this);
}