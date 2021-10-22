/**
 * @file Modules/MotionControl/HeadMotionEngine.h
 * This file declares a module that creates head joint angles from desired head motion.
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author Colin Graf
 */

#pragma once

#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadMotionGenerator.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"

MODULE(HeadMotionEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadAngleRequest),
  REQUIRES(HeadLimits),
  REQUIRES(JointAngles),
  USES(JointRequest),
  PROVIDES(HeadMotionGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(10.f) maxAcceleration, /**< Maximum angle acceleration (rad/s^2). */
    (float)(1.f) maxAccelerationNoGroundContact, /**< Maximum angle acceleration (rad/s^2) when not having ground contact. */
    (int)(800) stopAndGoModeFrequency, /**< Milliseconds between 2 stops in stopAndGoMode. */
  }),
});

class HeadMotionEngine : public HeadMotionEngineBase
{
  void update(HeadMotionGenerator& headMotionGenerator) override;

  Vector2f lastSpeed = Vector2f::Zero();
};
