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
  (bool)(true) keeperJumpingOn, /**< Use this configuration parameter to prevent keeper from jumping (and maybe hurting itself) - default == true */

  (float)(0.7f) penaltyStrikerWalkSpeed, /**< defines speed in percent (0-100) for penalty striker to walk to the ball */
  (Angle)(10_deg) penaltyStrikerAngleToLeftPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (Angle)(10_deg) penaltyStrikerAngleToRightPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (bool)(true) penaltyStrikerUseObstacles, /**< Whether the penalty striker should use the obstacle model to kick in the more free corner */

  (float)(1200.f) ballCatchMaxWalkDistance,

  (float)(60.f) standRadius, /**< The range that is covered by just standing. */
  (float)(125.f) walkRadius, /**< The range that is covered by walking. */
  (float)(200.f) genuflectRadius, /**< The range that is covered with the penalty keeper genuflect. */
  (float)(200.f) genuflectStandRadius, /**< The range that is covered with a genuflect. */
  (float)(600.f) jumpRadius, /**< The range that is covered with a keeper jump. */
});
