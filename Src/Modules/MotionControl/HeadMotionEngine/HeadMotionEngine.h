/**
 * @file Modules/MotionControl/HeadMotionEngine.h
 * This file declares a module that creates head joint angles from desired head motion.
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author Colin Graf
 * @author Felix Wenk
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadMotionGenerator.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Math/Range.h"
#include "Framework/Module.h"

MODULE(HeadMotionEngine,
{,
  REQUIRES(CameraIntrinsics),
  REQUIRES(CameraCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadMotionRequest),
  REQUIRES(HeadLimits),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(HeadMotionGenerator),
  DEFINES_PARAMETERS(
  {,
    (Angle)(27_deg) defaultTilt,
    (float)(20.f) maxAcceleration, /**< Maximum angle acceleration (rad/s^2). */
    (float)(1.f) maxAccelerationNoGroundContact, /**< Maximum angle acceleration (rad/s^2) when not having ground contact. */
    (int)(800) stopAndGoModeFrequency, /**< Milliseconds between 2 stops in stopAndGoMode. */
    (Rangea)(5_deg, -10_deg) lowerCamThreshold,
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

  void updateHeadAngleRequest(HeadAngleRequest& headAngleRequest, bool& lastWasLower) const;

  void calculatePanTiltAngles(const Vector3f& hip2Target, CameraInfo::Camera camera, Vector2a& panTilt) const;

  void adjustTiltBoundToShoulder(Angle pan, CameraInfo::Camera camera, Rangea& tiltBound) const;

  Rangea panBounds;
  HeadAngleRequest theHeadAngleRequest;
  Vector2f lastSpeed = Vector2f::Zero();
  bool lastWasLower = false;
};
