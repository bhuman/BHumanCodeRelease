/**
 * @file BallSearch.h
 *
 * This file declares a ball search behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Tools/BehaviorControl/Strategy/BehaviorBase.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Math/Geometry.h"

class BallSearch : public BehaviorBase
{
public:
  bool ballUnknown = false;
  //Own corner kick possible ball positions
  const Vector2f leftOpponentCorner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
  const Vector2f rightOpponentCorner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline);
  //Own goal kick possible ball positions
  const Vector2f leftOwnGoalCorner = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  const Vector2f rightOwnGoalCorner = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);
  //Opponent corner kick possible ball positions
  const Vector2f leftOwnCorner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline);
  const Vector2f rightOwnCorner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline);
  //Opponent goal kick possible ball positions
  const Vector2f leftOpponentGoalCorner = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea);
  const Vector2f rightOpponentGoalCorner = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
  const float cornerPositionOffset = 500.f;
  const int refereeBallPlacementDelay = 5000;
  const float refereeBallPlacementAccuracy = 400.f;
  bool teammatesBallModelInCorner = false;
  bool ballModelIsInOneCorner = false;

  void preProcess();

  SkillRequest execute(const Agent& agent, const Agents& otherAgents);
};
