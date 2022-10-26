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

  // Parameters shared between all free kick cards
  (float)(150.f) strikerWaitDistanceToBall,     /**< Distance to the ball the striker maintains when waiting for supporters to take their positions */
  (float)(50.f) maxDistanceForStanding,         /**< If a robot is closer than this distance in mm to its intended position it may stop */
  (Angle)(4_deg) maxAngleDifferenceForStanding, /**< If a robot is closer than this angle to its intended rotation it may stop */
  (float)(120.f) keepTargetRotationDistance,    /**< Distance to target pose in mm that forces to keep the target rotation */

  // Parameters for TapFreeKickCard and WaitForTapFreeKickCard, moved here because they are shared
  (Angle)(50_deg) maxBallToGoalAngle,             /**< If the angle from ball to goal is better (i.e. smaller) than this, a tap free kick can be performed */
  (float)(4000) maxBallToGoalDistance,            /**< Maximum distance in mm of the ball to the opponents goal for executing a TapFreeKick */
  (int)(22000) ignoreTeammateTime,                /**< If more time in ms is left for executing the free kick, the teammates activity is ignored */
  (int)(3700) timeLeftAfterSupporterIsInPosition, /**< Time in ms that has to be left after the supporter reaches its waiting position */
  (float)(650) yOffsetWaitPosition,               /**< Supporter waiting position offset against the ball in x direction in mm */
  (float)(350) xOffsetWaitPosition,               /**< Supporter waiting position offset against the ball in y direction in mm */
});
