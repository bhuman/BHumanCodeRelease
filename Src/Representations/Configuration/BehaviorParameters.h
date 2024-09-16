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
  (bool) keeperJumpingOn, /**< Use this configuration parameter to prevent keeper from jumping (and maybe hurting itself) - default == true */

  (float) penaltyStrikerWalkSpeed, /**< defines speed in percent (0-100) for penalty striker to walk to the ball */
  (Angle) penaltyStrikerAngleToLeftPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (Angle) penaltyStrikerAngleToRightPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (bool) penaltyStrikerUseObstacles, /**< Whether the penalty striker should use the obstacle model to kick in the more free corner */

  (float) ballCatchMaxWalkDistance,

  (float) standRadius, /**< The range that is covered by just standing. */
  (Rangef) walkRadius, /**< The range that is covered by walking. */
  (Rangef) timeForIntercetionForMaxWalkRadius, /**< The ball needs this time until intercetion to allow for the max walk radius. */
  (float) genuflectRadius, /**< The range that is covered with the penalty keeper genuflect. */
  (float) genuflectStandRadius, /**< The range that is covered with a genuflect. */
  (float) jumpRadius, /**< The range that is covered with a keeper jump. */
  (float) timeForJump, /**< Time of a jump until the robot starts to hold a ball. */
});
