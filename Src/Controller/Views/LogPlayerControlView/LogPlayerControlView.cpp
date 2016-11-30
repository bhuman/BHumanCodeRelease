#include "LogPlayerControlView.h"
#include "LogPlayerControlWidget.h"

SimRobot::Widget* LogPlayerControlView::createWidget()
{
  return new LogPlayerControlWidget(*this);
}