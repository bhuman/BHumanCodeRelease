/**
 * @file Representations/MotionControl/HeadMotionRequest.h
 * This file declares a struct that represents the requested head motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct HeadMotionRequest
 * A struct that represents the requested head motion.
 */
STREAMABLE(HeadMotionRequest,
{
  ENUM(Mode,
  {,
    panTiltMode,        /**< Use \c pan, \c tilt and \c speed. */
    targetMode,         /**< (A target relative to the center of hip.) Use \c target and \c speed. */
    targetOnGroundMode, /**< Use \c target and \c speed. */
  });

  ENUM(CameraControlMode,
  {,
    autoCamera,
    lowerCamera,
    upperCamera,
  }),

  (Mode)(panTiltMode) mode, /**< The active head motion mode. */
  (CameraControlMode)(lowerCamera) cameraControlMode, /**< The active camera control mode. */
  (bool)(false) watchField, /**< True, if as much as possible of the field should be watched instead of centering the target in the image. */
  (Angle)(0) pan,           /**< Head pan target angle in radians. */
  (Angle)(0) tilt,          /**< Head tilt target angle in radians. */
  (Angle)(1) speed,         /**< Maximum joint speed to reach target angles in radians/s. */
  (Vector3f) target,        /**< Look at target relative to the robot. */
});

STREAMABLE(TeamHeadControlState,
{,
  (bool)(false) checksBall,
  (bool)(false) usesActiveVision,
});
