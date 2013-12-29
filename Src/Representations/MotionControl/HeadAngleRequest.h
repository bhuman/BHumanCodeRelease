/**
* @file Representations/MotionControl/HeadAngleRequest.h
* This file declares a class that represents a request to set specific angles for the head joint.
* @author Felix Wenk
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
* @class HeadAngleRequest
* A class that represents the requested head angles.
*/
STREAMABLE(HeadAngleRequest,
{,
  (float)(0) pan,     /**< Head pan target angle in radians. */
  (float)(0) tilt,    /**< Head tilt target angle in radians. */
  (float)(1) speed, /**< Maximum joint speed to reach target angles in radians/s. */
});
