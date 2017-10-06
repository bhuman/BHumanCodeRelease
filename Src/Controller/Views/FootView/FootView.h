#pragma once

#include "SimRobot.h"

#include <QIcon>
#include <QString>

struct FootGroundContactState;
struct JointSensorData;
struct RobotDimensions;
class RobotConsole;

class FootView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */

  RobotConsole& robotConsole;
  const FootGroundContactState& footGroundContactState;
  const JointSensorData& jointSensorData;
  const RobotDimensions& robotDimensions;

  friend class FootViewWidget;

public:
  FootView(const QString& fullName, RobotConsole& robotConsole, const FootGroundContactState& footGroundContactState,
           const JointSensorData& jointSensorData, const RobotDimensions& robotDimensions) :
    fullName(fullName), icon(":/Icons/tag_green.png"), robotConsole(robotConsole),
    footGroundContactState(footGroundContactState), jointSensorData(jointSensorData), robotDimensions(robotDimensions)
  {}

  /**
   * The method returns a new instance of a widget for this view.
   * The caller has to delete the returned instance. (Qt will take care of this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }
};