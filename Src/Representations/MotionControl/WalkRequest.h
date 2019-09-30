/**
 * @file Representations/MotionControl/WalkRequest.h
 * This file declares a struct that represents a walk request.
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct WalkRequest
 * A struct that represents a walk request.
 */
STREAMABLE(WalkRequest,
{
  using WalkKickRequest = WalkKickVariant;

  ENUM(Mode,
  {,
    absoluteSpeedMode, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
    relativeSpeedMode, /**< Interpret \c speed as ratio of the maximum walking speed and ignore \c target. */
    targetMode, /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as in relativeSpeedMode. */
  });

  bool isValid() const
  {
    return !std::isnan(static_cast<float>(speed.rotation)) && !std::isnan(speed.translation.x()) && !std::isnan(speed.translation.y()) &&
           (mode != targetMode || (!std::isnan(static_cast<float>(target.rotation)) && !std::isnan(target.translation.x()) && !std::isnan(target.translation.y())));
  },

  (Mode)(absoluteSpeedMode) mode, /**< The walking mode. */
  (Pose2f) speed, /**< Walking speeds as ratios of the maximum speeds or mm/s and radian/s. */
  (Pose2f) target, /**< Walking target, in mm and radians, relative to robot for targetMode, in field coordinates for pathModes. */
  (WalkKickRequest) walkKickRequest,
});
