#include "FootView.h"
#include "FootViewWidget.h"
#include "Controller/RobotConsole.h"

SimRobot::Widget* FootView::createWidget()
{
  robotConsole.handleConsole("mr FootGroundContactState FootGroundContactStateProvider");
  robotConsole.handleConsole("dr representation:FootGroundContactState");
  robotConsole.handleConsole("dr representation:JointSensorData");
  robotConsole.handleConsole("dr representation:RobotDimensions");

  return new FootViewWidget(robotConsole, footGroundContactState, jointSensorData, robotDimensions);
}