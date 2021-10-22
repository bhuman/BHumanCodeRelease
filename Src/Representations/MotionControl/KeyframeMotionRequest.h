/**
 * @file Representations/MotionControl/KeyframeMotionRequest.h
 * This file declares a struct to represent keyframe motion requests.
 * @author Thomas Röfer
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct KeyframeMotionRequest
 * The struct represents keyframe motion requests.
 */
STREAMABLE(KeyframeMotionRequest,
{
  /** ids for all keyframe motions */
  ENUM(KeyframeMotionID,
  {,
    decideAutomatic, //Default. If selected, robot tries to get up
    front,
    back,
    fromSplit,
    recoverFast,
    recoverFromSideBack,
    recoverFromSideFront,
    recoverFromGenu,
    recoverFromSumo,
    recoverArmLeftFrontLyingOn,
    stand,
    sit,
    calibrateHalfSplit,
    calibrateSplit,
    // below are normal motions
    firstNonGetUpAction,
    sitDown = firstNonGetUpAction,
    sitDownKeeper,
    keeperJumpLeft,
    genuflectFromSitting,
    genuflectStand,
    genuflectStandDefender,
    demoBannerWave,
    demoBannerWaveInitial,
  }),

  (KeyframeMotionID)(decideAutomatic) keyframeMotion, /**< The keyframe motion selected. */
  (bool)(false) mirror, /**< Mirror left and right. For get up motions, this field is ignored. */
});
