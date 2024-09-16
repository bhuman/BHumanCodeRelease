/**
 * @file PlayBall.h
 *
 * This file declares the default ball playing behavior.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#pragma once

#include "Modules/BehaviorControl/StrategyBehaviorControl/Behavior.h"
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"

class PlayBall : public ActiveRole
{
  STREAMABLE(Parameters,
  {,
    (bool)(false) alwaysShoot, /**< Whether or not passing should be deactivated entirely */
    (float)(0.8f) shootThreshold, /**< Threshold for the own goal shot rating above which no pass targets are considered */
    (float)(1500.f) minPassDistance, /**< In playing, consider teammates as pass targets that are further away from the current player */
    (float)(4500.f) maxPassDistance, /**< In playing, consider teammates as pass targets that are within this radius from the current player */
    (float)(6000.f) maxFreeKickDistance, /**< During free kicks, increase maximum pass distance to this */
    (float)(9000.f) maxGoalKickDistance, /**< During goal kicks, increase maximum pass distance to this */
    (int)(1) maxSearchDepth, /**< The maximum number of passes before a goal shot */
    (float)(0.9f) discountFactor, /**< For each subsequent pass, the rating is made a bit worse to discourage long pass chains */
    (float)(0.1f) minPenalty, /**< Changing the decision to shoot or pass from the previous frame, results in this penalty to the rating */
    (float)(0.8f) maxPenalty, /**< Minimum decision penalty is scaled up to this maximum penalty with decreasing distance to the ball */
    (float)(300.f) ballDistMinPenalty, /**< Apply only the default penalty if the ball is further away than this */
    (float)(200.f) ballDistMaxPenalty, /**< Maximize the penalty if the ball is closer than this */
    (float)(0.01f) minRating,
    (float)(0.05f) clearDribbleHysteresisValue, /* added value to clearRating when clear ball has been added in the last update*/
    (float)(1000.f) minPassDistanceSAC,
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

  void reset() override;
  void preProcess() override;
  void draw(const Vector2f& passBasePosition, const Vector2f& agentPosition, const float value, const float penalty, const bool applyPenalty);

  /**
   * TODO: Add documentation
   */
  float findPassTarget(const Vector2f& ballPosition, const Agents& teammates, const float maxPassDistance, int remainingSearchDepth, const Agent*& passTarget, float& maxValue, const float decisionPenalty, const std::vector<int>& test);

  Parameters p;
  int lastPassTarget = 0; /**< Number of the last passed-to player from the previous frame, -1 for a goal shot, 0 if the role started this frame */
  Eigen::Vector2f dribbleTarget = { -1.f, -1.f };
  const Eigen::Vector2f goalPosition = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);
  float decisionPenalty = 0.f;
  float clearDribbleHysteresis = 0.f;
};
