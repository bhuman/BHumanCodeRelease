/**
 * @file PlayBall.h
 *
 * This file declares the default ball playing behavior.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/ActiveRole.h"

class PlayBall : public ActiveRole
{
  STREAMABLE(Parameters,
  {,
    (bool)(false) alwaysShoot,
    (bool)(false) ignoreObstacles, /**< Whether or not to call the executeLegacy algorithm instead of the default smashOrPass algorithm */
    (float)(0.8f) shootThreshold, /**< Threshold for the own goal shot rating above which no pass targets are considered */
    (float)(0.15f) passImprovement, /**< Offset to the own goal shot rating, that a teammate's goal shot rating must be better than to be considered as a pass target (only used in executeLegacy algorithm) */
    (float)(1500.f) minPassDistance, /**< In playing, consider teammates as pass targets that are further away from the current player */
    (float)(4500.f) maxPassDistance, /**< In playing, consider teammates as pass targets that are within this radius from the current player */
    (float)(6000.f) maxFreeKickDistance, /**< During free kicks, increase maximum pass distance to this */
    (float)(9000.f) maxGoalKickDistance, /**< During goal kicks, increase maximum pass distance to this */
    (float)(0.1f) minPenalty, /**< Changing the decision to shoot or pass from the previous frame, results in this penalty to the rating */
    (float)(0.8f) maxPenalty, /**< Minimum decision penalty is scaled up to this maximum penalty with decreasing distance to the ball */
    (float)(300.f) ballDistMinPenalty, /**< Apply only the default penalty if the ball is further away than this */
    (float)(200.f) ballDistMaxPenalty, /**< Maximize the penalty if the ball is closer than this */
    (float)(0.01f) minRating,
    (bool)(true) lookAhead, /**< Whether or not to estimate the teammate's position into the future towards his communicated target pose (not implemented yet) */
    (int)(2000) lookAheadTime, /**< If lookAhead is true, then the position of each teammate will be estimated this many milliseconds into the future */
    (float)(500.f) sampleDistance, /**< Space between sample positions on the line from the teammate's position to the opponent's goal */
    (Rangei)(-1, 2) sampleSteps, /**< Time step interval to sample the positions on the line from the teammate's position to the opponent's goal */
  });

  /**
   * Calculates the role's intention by calling the smashOrPass algorithm (by default) or the executeLegacy algorithm (when the ignoreObstacles parameter is set to true).
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request that the role wants to be executed.
   */
  SkillRequest execute(const Agent& self, const Agents& teammates) override;

  /**
   * SmashOrPass algorithm to decide whether to shoot at the goal or pass the ball to a teammate by using rating functions that estimate the success of these actions.
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request that the role wants to be executed.
   */
  SkillRequest smashOrPass(const Agent& self, const Agents& teammates);

  /**
   * Previous algorithm to decide whether to shoot at the goal or pass the ball to a teammate by using rating functions not taking into account the known obstacles.
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request that the role wants to be executed.
   */
  SkillRequest executeLegacy(const Agent& self, const Agents& teammates);

  void reset() override;
  void preProcess() override;
  void draw(const Vector2f& ballPosition, const Vector2f& agentPosition, const Vector2f& bestPosition, const float value, const float penalty, const bool applyPenalty);

  Parameters p;

  int lastPassTarget = 0; /**< Number of the last passed-to player from the previous frame, -1 for a goal shot, 0 if the role started this frame */
  const Vector2f goalPosition = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
};
