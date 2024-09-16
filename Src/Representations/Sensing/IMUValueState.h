/**
 * @file IMUValueState.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"
#include "Math/Angle.h"

STREAMABLE(IMUValueState,
{
  STREAMABLE(ValueState,
  {,
    (Vector3f)(Vector3f::Zero()) mean, /**< Mean over the last time interval. */
    (Vector3f)(Vector3f::Zero()) deviation, /**< Deviation over the last time interval. */
    (unsigned int)(0) deviationNotChangingSinceTimestamp, /**< Timestamp since the values did not change much. */
    (unsigned int)(0) stableSinceTimestamp, /**< Timestamp since the values did not change much. */
  }),

  (IMUValueState::ValueState) gyroValues, /**< The gyro values */
  (IMUValueState::ValueState) accValues, /**< The acc values */
  (unsigned int)(0) timestamp, /**< Timestamp for the last update. Current update rate is once every 324ms */
  (int)(0) filterTimeWindow, /**< Time span, until all samples are new. */
  (unsigned int)(0) notMovingSinceTimestamp, /**< Timestamp since the robot was not moving much. */
  (float)(Constants::g_1000) accLength, /**< The acc actually measured length. */
});
