/**
 * @file MotionInfo.h
 *
 * This file declares a struct that represents the status of the motion for the behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/KickInfo.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Math/BHMath.h"
#include "Tools/Motion/MotionPhase.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(MotionInfo,
{
  bool isMotion(MotionPhase::Type motion) const
  {
    return executedPhase == motion;
  }

  bool isMotion(unsigned mask) const
  {
    return (bit(executedPhase) & mask) != 0;
  }

  bool isKicking() const
  {
    return executedPhase == MotionPhase::kick || (executedPhase == MotionPhase::walk && isWalkPhaseInWalkKick);
  }

  bool isKeyframeMotion(KeyframeMotionRequest::KeyframeMotionID motion) const
  {
    return executedPhase == MotionPhase::keyframeMotion && executedKeyframeMotion.keyframeMotion == motion;
  }

  bool isKeyframeMotion(KeyframeMotionRequest::KeyframeMotionID motion, bool mirror) const
  {
    return executedPhase == MotionPhase::keyframeMotion && executedKeyframeMotion.keyframeMotion == motion && executedKeyframeMotion.mirror == mirror;
  },

  (MotionPhase::Type)(MotionPhase::playDead) executedPhase, /**< The type of the phase currently being executed. */
  (bool)(false) isMotionStable, /**< Whether the current motion is considered to be stable .*/

  // walk phase
  (Pose2f) speed, /**< The current speed at which the robot is walking */
  (bool)(false) isWalkPhaseInWalkKick, /**< Whether the current walk phase is an in walk kick. */

  (unsigned int)(0) lastStandTimeStamp, /**< Timestamp when last stand started. */
  (unsigned int)(0) lastMotionPhaseStarted, /**< Timestamp when last motion phase started. */

  (bool)(true) walkPhaseIsLeftPhase, /**< If the current motion phase is a walk phase, then is the current phase a left swing phase? */

  // kick or walk-kick phase
  (unsigned)(0) lastKickTimestamp, /**< The timestamp of the most recently finished kick. */
  (KickInfo::KickType)(KickInfo::numOfKickTypes) lastKickType, /**< The most recently executed kick type. */

  // get up phase
  (unsigned)(0) getUpTryCounter, /**< The number of consecutive get up phases before this one. */

  // keyframe motion phase
  (KeyframeMotionRequest) executedKeyframeMotion, /**< The specific type of keyframe motion. */

  (OdometryData) odometryAtLastPhaseSwitch,
  (OdometryTranslationRequest) odometryRequestAtLastPhaseSwitch,
});
