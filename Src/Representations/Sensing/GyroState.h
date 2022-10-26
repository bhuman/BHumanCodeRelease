/**
 * @file GyroState.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"
#include "Math/Angle.h"

STREAMABLE(GyroState,
{,
  (Vector3a)(Vector3a::Zero()) mean, /**< Mean of the gyros over the last time intervall. */
  (Vector3a)(Vector3a::Zero()) deviation, /**< Deviation of the gyros over the last time intervall. */
  (unsigned int)(0) timestamp, /**< Timestamp for the last update. Current update rate is once every 324ms */
  (unsigned int)(0) notMovingSinceTimestamp, /**< Timestamp since the robot was not moving much. */
});
