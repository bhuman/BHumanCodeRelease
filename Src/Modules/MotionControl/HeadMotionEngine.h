/**
* @file Modules/MotionControl/HeadMotionEngine.h
* This file declares a module that creates head joint angles from desired head motion.
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Math/Geometry.h"

MODULE(HeadMotionEngine)
  REQUIRES(HeadAngleRequest)
  REQUIRES(FilteredJointData)
  REQUIRES(FrameInfo)
  REQUIRES(JointCalibration)
  REQUIRES(GroundContactState)
  PROVIDES_WITH_MODIFY(HeadJointRequest)
END_MODULE

class HeadMotionEngine: public HeadMotionEngineBase
{
private:
  float requestedPan;
  float requestedTilt;
  Vector2<> lastSpeed;
  Geometry::Circle deathPoints[4];

  /**
  * The update method to generate the head joint angles from desired head motion.
  */
  void update(HeadJointRequest& headJointRequest);

public:
  HeadMotionEngine();
};
