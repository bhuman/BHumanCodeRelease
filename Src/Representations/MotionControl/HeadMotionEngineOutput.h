/**
 * @file Representations/MotionControl/HeadMotionEngineOutput.h
 * This file declares a struct that represents the requested head joint angles.
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct HeadJointRequest
 * A struct that represents the requested head joint angles.
 */
STREAMABLE(HeadMotionEngineOutput,
{,
  (Angle)(0) pan, /**< Head pan target angle. */
  (Angle)(0) tilt, /**< Head tilt target angle. */
  (bool)(true) reachable, /**< Whether the head motion request points on a reachable position. */
  (bool)(false) moving, /**< Whether the head is currently in motion or not. */
});
