/**
 * @file Representations/MotionControl/WalkRequest.h
 * This file declares a struct that represents a walk request.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author Colin Graf
 */

#pragma once

#include "Representations/MotionControl/WalkKicks.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct WalkRequest
 * A struct that represents a walk request.
 */
STREAMABLE(WalkRequest,
{
  ENUM(Mode,
  {,
    speedMode, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
    percentageSpeedMode, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
    targetMode, /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as percentage walking speed. */
    pathMode, /**< Just as targetMode, but avoiding obstacles and target is absolute. All three speed percentages must be the same. */
    pathModeAvoidingOwnPenaltyArea, /**< Just as pathMode, but also avoids own penalty area. */
  });

  /**
   * Definition of a kick request with a kick type and the kicking leg, a tolerance for the kick and a target afterwards.
   * In targetMode the kick shall be executed, when the robot reached the target with a tolerance of kickTargetTolerance.
   * In all other modes the kick shall be executed as soon as possible.
   * The tollerance defines how close to the target the requested kick can be excecuted.
   */
  STREAMABLE_WITH_BASE(WalkKickRequest, WalkKickVariant,
  {
    WalkKickRequest& operator=(const WalkKickVariant& other);
    ,
    (Pose2f)(Pose2f(1_deg, 10.f, 10.f)) kickTargetTolerance,
    (Pose2f) targetAfterKick,
  });

  bool isValid() const
  {
    return !std::isnan(static_cast<float>(speed.rotation)) && !std::isnan(speed.translation.x()) && !std::isnan(speed.translation.y()) &&
           (mode != targetMode || (!std::isnan(static_cast<float>(target.rotation)) && !std::isnan(target.translation.x()) && !std::isnan(target.translation.y())));
  },

  (Mode)(speedMode) mode, /**< The walking mode. */
  (Pose2f) speed, /**< Walking speeds, in percentage or mm/s and radian/s. */
  (Pose2f) target, /**< Walking target, in mm and radians, relative to robot for targetMode, in field coordinates for pathModes. */
  (WalkKickRequest) walkKickRequest,
});

inline WalkRequest::WalkKickRequest& WalkRequest::WalkKickRequest::operator=(const WalkKickVariant& other)
{
  WalkKickVariant::operator=(other);
  return *this;
}
