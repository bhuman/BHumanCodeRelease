/**
* @file Representations/MotionControl/WalkRequest.h
* This file declares a class that represents a walk request.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class WalkRequest
* A class that represents a walk request.
*/
STREAMABLE(WalkRequest,
{
public:
  ENUM(Mode,
    speedMode, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
    percentageSpeedMode, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
    targetMode /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as percentage walking speed. */
  );

  ENUM(KickType,
    none, /**< do not kick */
    left, /**< kick using the left foot */
    right, /**< kick using the right foot */
    sidewardsLeft,
    sidewardsRight
  );

  bool isValid() const
  {
    return !isnan(speed.rotation) && !isnan(speed.translation.x) && !isnan(speed.translation.y)
           && (mode != targetMode || (!isnan(target.rotation) && !isnan(target.translation.x) && !isnan(target.translation.y)));
  },

  (Mode)(speedMode) mode, /**< The walking mode. */
  (Pose2D) speed, /**< Walking speeds, in percentage or mm/s and radian/s. */
  (Pose2D) target, /**< Walking target, in mm and radians, relative to the robot. Use either a speed or a target. */
  (KickType)(none) kickType,
});
