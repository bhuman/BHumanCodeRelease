/**
 * @file Representations/MotionControl/WalkRequest.h
 * This file declares a struct that represents a walk request.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author Colin Graf
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Enum.h"
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

  ENUM(KickType,
  {,
    none, /**< do not kick */
    left, /**< kick using the left foot */
    right, /**< kick using the right foot */
    sidewardsLeft,
    sidewardsRight,
  });

  bool isValid() const
  {
    return !std::isnan(static_cast<float>(speed.rotation)) && !std::isnan(speed.translation.x()) && !std::isnan(speed.translation.y()) &&
      (mode != targetMode || (!std::isnan(static_cast<float>(target.rotation)) && !std::isnan(target.translation.x()) && !std::isnan(target.translation.y())));
  },

  (Mode)(speedMode) mode, /**< The walking mode. */
  (Pose2f) speed, /**< Walking speeds, in percentage or mm/s and radian/s. */
  (Pose2f) target, /**< Walking target, in mm and radians, relative to robot for targetMode, in field coordinates for pathModes. */
  (KickType)(none) kickType,
});
