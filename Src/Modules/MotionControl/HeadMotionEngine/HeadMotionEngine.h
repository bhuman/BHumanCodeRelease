/**
 * @file Modules/MotionControl/HeadMotionEngine.h
 * This file declares a module that creates head joint angles from desired head motion.
 * @author Alexander Härtl
 * @author Colin Graf
 * @author Felix Wenk
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadMotionGenerator.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Math/Range.h"
#include "Framework/Module.h"

MODULE(HeadMotionEngine,
{,
  REQUIRES(CameraCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(HeadMotionRequest),
  REQUIRES(HeadLimits),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(HeadMotionGenerator),
  LOADS_PARAMETERS(
  {,
    (float) maxAcceleration, /**< Maximum angle acceleration (rad/s^2). */
    (float) maxAccelerationIsMoving, /**< Maximum angle acceleration (rad/s^2) to accept head is not moving. */
    (int) stopAndGoModeFrequency, /**< Milliseconds between 2 stops in stopAndGoMode. */
  }),
});

STREAMABLE(HeadAngleRequest,
{,
  (Angle)(0) pan,   /**< Head pan target angle. */
  (Angle)(0) tilt,  /**< Head tilt target angle. */
  (Angle)(1) speed, /**< Maximum joint speed to reach target angles in rad/s. */
  (bool)(false) stopAndGoMode, /**< The Head will slow down and stop every \c stopAndGoModeFrequency milliseconds */
  (bool)(false) disableClippingAndInterpolation, /**< Set the head angles directly, do not clamp head angles to HeadLimits. */
});

class HeadMotionEngine : public HeadMotionEngineBase
{
public:
  HeadMotionEngine();

private:
  void update(HeadMotionGenerator& headMotionGenerator) override;

  void updateHeadAngleRequest(HeadAngleRequest& headAngleRequest) const;

  void calculatePanTiltAngles(const Vector3f& hip2Target, Vector2a& panTilt) const;

  void adjustTiltBoundToShoulder(Angle pan, Rangea& tiltBound) const;

  Rangea panBounds;
  HeadAngleRequest theHeadAngleRequest;
  Vector2f lastSpeed = Vector2f::Zero();
};
