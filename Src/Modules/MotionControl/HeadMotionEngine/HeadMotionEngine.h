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
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"

MODULE(HeadMotionEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadAngleRequest),
  REQUIRES(HeadLimits),
  REQUIRES(JointAngles),
  PROVIDES(HeadMotionEngineOutput),
  DEFINES_PARAMETERS(
  {,
    (int)(800) stopAndGoModeFrequenzy, /* Milliseconds between 2 stops in stopAndGoMode */
  }),
});

class HeadMotionEngine : public HeadMotionEngineBase
{
private:
  Angle requestedPan;
  Angle requestedTilt;
  Vector2f lastSpeed;

public:
  HeadMotionEngine();

  /**
   * The update method to generate the head joint angles from desired head motion.
   */
  void update(HeadMotionEngineOutput& headMotionEngineOutput) override;
};
