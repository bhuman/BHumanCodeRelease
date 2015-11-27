/**
 * @file Modules/MotionControl/HeadMotionEngine.h
 * This file declares a module that creates head joint angles from desired head motion.
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author Colin Graf
 */

#pragma once

#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"

MODULE(HeadMotionEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadAngleRequest),
  REQUIRES(JointAngles),
  REQUIRES(JointCalibration),
  PROVIDES(HeadJointRequest),
});

class HeadMotionEngine : public HeadMotionEngineBase
{
private:
  float requestedPan;
  float requestedTilt;
  Vector2f lastSpeed;
  Geometry::Circle deathPoints[4];

  /**
  * The update method to generate the head joint angles from desired head motion.
  */
  void update(HeadJointRequest& headJointRequest);

public:
  HeadMotionEngine();
};
