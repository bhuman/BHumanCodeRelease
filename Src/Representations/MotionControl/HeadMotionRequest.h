/**
* @file Representations/MotionControl/HeadMotionRequest.h
* This file declares a class that represents the requested head motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Math/Vector3.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class HeadMotionRequest
* A class that represents the requested head motion.
*/
STREAMABLE(HeadMotionRequest,
{
public:
  ENUM(Mode,
    panTiltMode,        /**< Use \c pan, \c tilt and \c speed. */
    targetMode,         /**< (A target relative to the center of hip.) Use \c target and \c speed. */
    targetOnGroundMode  /**< Use \c target and \c speed. */
  );

  ENUM(CameraControlMode,
    autoCamera,
    lowerCamera,
    upperCamera
  ),

  (Mode)(panTiltMode) mode, /**< The active head motion mode. */
  (CameraControlMode)(lowerCamera) cameraControlMode, /**< The active camera control mode. */
  (bool)(false) watchField, /**< True, if as much as possible of the field should be watched instead of centering the target in the image. */
  (float)(0) pan,           /**< Head pan target angle in radians. */
  (float)(0) tilt,          /**< Head tilt target angle in radians. */
  (float)(1) speed,         /**< Maximum joint speed to reach target angles in radians/s. */
  (Vector3<>) target,       /**< Look at target relative to the robot. */
});

STREAMABLE(TeamHeadControlState,
{,
  (bool)(false) checksBall,
  (bool)(false) usesActiveVision,
});
