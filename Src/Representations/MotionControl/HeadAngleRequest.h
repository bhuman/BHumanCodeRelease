/**
 * @file Representations/MotionControl/HeadAngleRequest.h
 * This file declares a struct that represents a request to set specific angles for the head joint.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct HeadAngleRequest
 * A struct that represents the requested head angles.
 */
STREAMABLE(HeadAngleRequest,
{,
  (Angle)(0) pan,   /**< Head pan target angle. */
  (Angle)(0) tilt,  /**< Head tilt target angle. */
  (Angle)(1) speed, /**< Maximum joint speed to reach target angles in rad/s. */
  (bool)(false) stopAndGoMode, /**< The Head will slow down and stop every HeadMotionEngine.stopAndGoModeFrequenzy milliseconds */
});
