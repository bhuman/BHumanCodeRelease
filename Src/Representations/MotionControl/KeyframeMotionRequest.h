/**
 * @file Representations/MotionControl/KeyframeMotionRequest.h
 * This file declares a struct to represent keyframe motion requests.
 * @author Thomas Röfer
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

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
    // below are normal motions
    firstNonGetUpAction,
    sitDown = firstNonGetUpAction,
    sitDownKeeper,
    keeperJumpLeft,
    genuflectStand,
    genuflectStandDefender,
    demoBannerWave,
    demoBannerWaveInitial,
  });

  static KeyframeMotionRequest fromDiveRequest(MotionRequest::Dive::Request diveRequest);

  static KeyframeMotionRequest fromSpecialRequest(MotionRequest::Special::Request specialRequest),

  (KeyframeMotionID)(decideAutomatic) keyframeMotion, /**< The keyframe motion selected. */
  (bool)(false) mirror, /**< Mirror left and right. For get up motions, this field is ignored. */
});
