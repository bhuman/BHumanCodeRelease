/**
 * @file StrategyBehaviorControlDBHDefender.h
 *
 * This file declares a module that determines the strategy of the team.
 *
 * @author Yannik Meinken
 * @author Lars Bredereke
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"
#include "Math/Angle.h"

MODULE(StrategyBehaviorControlDBHDefender,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(TeamData),
  PROVIDES(SkillRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(10000) ballSearchDelay, /**< if the ball is this long not seen the ball search is activated*/
    (int)(10000) ballSearchDuration, /**< duration of one phase of the search, after tis time next position is observed*/
    (int)(20000) ballStillOnSpot, /**< if we have not jet seen the ball we assume in this time that it is still on the penalty mark*/
    (float)(100.f) ballWeight, /**< shod the first defender go straiter to the ball (higher) or more in the line of a potential pass (lower)*/
    (float)(1000) notFirstAttacker, /**< obstacles with a x coordinate less than this are considered one of the pass receiving attackers*/
    (int)(1000) timeToTrustBall, /**< if the ball was not seen this long we go not strait to it*/
    (float)(300) safetyDistanceToMid,/**< distance the third defender always keeps to the middle line*/
    (float)(300.f) ballWeightOtherHalf, /**< shod the first defender go straiter to the ball (higher) or more in the line of a potential pass (lower)*/
    (int)(3000) verifyOpponentToMarkTime, /**< if the Opponent to mark was this long not seen look if he is still in the same spot*/
    (int)(5000) searchOpponentToMarkTime, /**< if the Opponent to mark was this long not seen look to find him again*/
    (float)(500.f) sameObstacleThreshold, /**< Threshold distance to update the lastObstaclToMark with the new one*/
    (int)(3000) decideMirroredInterval, /**< in the first 3sec the 3 just walks forward and then decide based on the position of the 2 whether or not to play mirrored*/
    (float)(1500.f) ballNearMarkedObstacle, /**< if the Ball was nearer then this to the obstacle that is marked always go directly to the ball*/
  }),
});

class StrategyBehaviorControlDBHDefender : public StrategyBehaviorControlDBHDefenderBase
{
  /**
   * Updates the skill request.
   * @param skillRequest The provided skill request.
   */
  void update(SkillRequest& skillRequest) override;

  SkillRequest behave();

  SkillRequest kickBall();

  SkillRequest searchBall();

  SkillRequest handleBallInOwnHalf();

  Angle angleToBall = 0.f;

  bool secondAttackerFound = false;
  bool ballFound = false;

  bool mirrored = false;

  Vector2f positionToMark = Vector2f(-600.f, mirrored ? 1700.f : -1700.f);
  int timeWhenObstacleToMarkWasSeen = 0;
  bool ballWasNearMarked = false;

  int timeBallSearchStarted = 0; //How long since we startet searching the ball?
};
