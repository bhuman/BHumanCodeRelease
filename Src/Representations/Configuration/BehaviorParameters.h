/**
 * @file Representations/Configuration/BehaviorParameters/BehaviorParameters.h
 *
 * The file declares a struct that contains frequently used parameters of the behavior which can be modified.
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Math/Angle.h"

STREAMABLE(BehaviorParameters,
{,
  (Angle) penaltyStrikerAngleToLeftPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (Angle) penaltyStrikerAngleToRightPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (bool) penaltyStrikerUseObstacles, /**< Whether the penalty striker should use the obstacle model to kick in the more free corner */

  (Angle) defaultLookDownAngle, /**< Use this default head tilt */

  (float) ballCatchMaxWalkDistance,
  (float) genuflectRadius, /**< The range that is covered with the penalty keeper genuflect. */
  (float) illegalAreasSmallDistance, /**< To enter illegal state use this smaller value. */
  (float) illegalAreasLargeDistance, /**< To leave illegal state use this higher value. */

  (int) noCommunicationAfterUnpenalizedTime, /**< Prevent communication when freshly unpenalized. */
  (int) noSkillRequestAfterUnpenalizedTime, /**< Prevent behavior calulcation when freshly unpenalized. Should be bigger than noCommunicationAfterUnpenalizedTime. */
  (int) lookAroundAfterUnpenalizedTime, /**< Look around after unpenalized. Should be bigger than lookAroundAfterUnpenalizedTime. */

  (int) ballSeenThreshold, /**< If the ball is older than this [ms], an agent can not become active (it can stay active, though, if it already was). */
  (int) ballLostThreshold, /**< If the ball is older than this [ms], an agent can not be active (even if it was). */
  (int) ballSearchThresholdNoTeam, /**< The time after the ball was last seen to start searching for it, when the robot has no team and is playing alone. */
  (int) ballSearchThresholdTeam, /**< The time after the ball was last seen to start searching for it, when the robot has team and is not playing alone. */
});
