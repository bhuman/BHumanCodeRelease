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
    (bool) alwaysShoot, /**< Whether or not passing should be deactivated entirely */
    (float) shootThreshold, /**< Threshold for the own goal shot rating above which no pass targets are considered */
    (float) shootGoalKickThreshold, /**< Threshold for the own goal shot rating above which no pass targets are considered during goal kicks */
    (float) passShotRatio, /**< Passing must be this much greater than shoting. */
    (float) riskyShootThreshold, /**< Threshold for a shoot attempt if no pass targets are available */
    (float) passDistanceSameTargetHysteresis, /**< The teammate for which a pass was planned last execution cycle is allowed to stand a bit closer or further away. */
    (float) minPassDistance, /**< In playing, consider teammates as pass targets that are further away from the current player */
    (float) minFreeKickPassDistance, /**< In free kicks, consider teammates as pass targets that are further away from the current player */
    (float) maxPassDistance, /**< In playing, consider teammates as pass targets that are within this radius from the current player */
    (float) maxFreeKickDistance, /**< During free kicks, increase maximum pass distance to this */
    (float) maxGoalKickDistance, /**< During goal kicks, increase maximum pass distance to this */
    (int) maxSearchDepth, /**< The maximum number of passes before a goal shot */
    (float) discountFactor, /**< For each subsequent pass, the rating is made a bit worse to discourage long pass chains */
    (float) minPenalty, /**< Changing the decision to shoot or pass from the previous frame, results in this penalty to the rating */
    (float) maxPenalty, /**< Minimum decision penalty is scaled up to this maximum penalty with decreasing distance to the ball */
    (float) ballDistMinPenalty, /**< Apply only the default penalty if the ball is further away than this */
    (float) ballDistMaxPenalty, /**< Maximize the penalty if the ball is closer than this */
    (float) minRating,
    (float) clearDribbleHysteresisValue, /**< added value to clearRating when clear ball has been added in the last update */
    (float) rememberPassTargetTime, /**< Keep passing for this duration even if no pass is planned anymore. */
    (int) waitForPassTargetEndTime,
  });

  /**
   * Calculates the role's intention by calling the smashOrPass algorithm (by default) or the executeLegacy algorithm (when the ignoreObstacles parameter is set to true).
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request that the role wants to be executed.
   */
  SkillRequest execute(const Agent& self, const Agents& teammates) override;

  /**
   * Decides whether to kick the ball away or return to own base position.
   * Mainly used for the goalkeeper in OneVsOneDemo.
   * @param self The agent which executes the role.
   * @return The skill request that the role wants to be executed.
   */
  SkillRequest kickAwayOrReturn(const Agent& self);

  /**
   * SmashOrPass algorithm to decide whether to shoot at the goal or pass the ball to a teammate by using rating functions that estimate the success of these actions.
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request that the role wants to be executed.
   */
  SkillRequest smashOrPass(const Agent& self, const Agents& teammates);

  struct SmashOrPassInfo
  {
    SkillRequest request;
    int passTarget = -1;
  };

  SmashOrPassInfo getSmashOrPassInfo(const Agent& self, const Agents& teammates);

  void reset() override;
  void preProcess() override;
  void draw(const Vector2f& passBasePosition, const Vector2f& agentPosition, const float value, const float penalty, const bool applyPenalty);

  /**
   * TODO: Add documentation
   */
  float findPassTarget(const Vector2f& ballPosition, const Agents& teammates, const float minPassDistance, const float maxPassDistance, int remainingSearchDepth, const Agent*& passTarget, float& maxValue, const float decisionPenalty, const std::vector<int>& test);

  Parameters p;
  int lastPassTarget = 0; /**< Number of the last passed-to player from the previous frame, -1 for a goal shot, 0 if the role started this frame */
  unsigned lastPassTargetTimestamp = 0; /**< Timestamp when last time a pass was planned. Used to keep passing for a short time. */
  std::optional<Vector2f> dribbleTarget;
  const Vector2f goalPosition = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);
  float decisionPenalty = 0.f;
  float clearDribbleHysteresis = 0.f;

public:
  PlayBall();
};
