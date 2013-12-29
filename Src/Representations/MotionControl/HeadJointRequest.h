/**
* @file Representations/MotionControl/HeadJointRequest.h
* This file declares a class that represents the requested head joint angles.
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
* @class HeadJointRequest
* A class that represents the requested head joint angles.
*/
STREAMABLE(HeadJointRequest,
{,
  (float)(0) pan, /**< Head pan target angle in radians. */
  (float)(0) tilt, /**< Head tilt target angle in radians. */
  (bool)(true) reachable, /**< Whether the head motion request points on a reachable position. */
  (bool)(false) moving, /**< Whether the head is currently in motion or not. */
});
