/**
 * @file GyroOffset.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"
#include "Math/Angle.h"

STREAMABLE(GyroOffset,
{,
  (Vector3a)(Vector3a::Zero()) offset, /**< Current offsets of the gyros. */
  (bool)(false) offsetCheckFinished, /**< Could the robot check for offsets in the gyros? If not, the robot should say so and not do anything. */
  (bool)(false) bodyDisconnect, /**< Head lost connection to the body */
  (bool)(false) isIMUBad,  /**< Do the gyros have such high offsets, that the robot should not do any motion related things? */
});
