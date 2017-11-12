#pragma once

#include "SimRobot.h"

#include <QWidget>

class QProgressBar;
class RobotConsole;
struct FootGroundContactState;
struct JointSensorData;
struct RobotDimensions;

class FootViewWidget : public QWidget, public SimRobot::Widget
{
private:
  RobotConsole& robotConsole;
  const FootGroundContactState& footGroundContactState;
  const JointSensorData& jointSensorData;
  const RobotDimensions& robotDimensions;

  QProgressBar* leftTotalWeightIndicator = nullptr;
  QProgressBar* rightTotalWeightIndicator = nullptr;

  unsigned lastUpdateTimeStamp = 0;

  friend class FootWidget;

public:
  FootViewWidget(RobotConsole& robotConsole, const FootGroundContactState& footGroundContactState,
                 const JointSensorData& jointSensorData, const RobotDimensions& robotDimensions);

  virtual QWidget* getWidget() { return this; }
  virtual void update();

private:
  bool needsRepaint();

  void updateTotalWeightIndicators();
};
