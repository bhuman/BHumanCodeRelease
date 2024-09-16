/**
 * @file IllegalAreaProvider.cpp
 *
 * This file implements a module that determines which field areas are illegal to enter.
 *
 * @author Arne Hasselbring
 * @author Fynn Böse
 */

#include "IllegalAreaProvider.h"
#include "Tools/BehaviorControl/Strategy/Role.h"
#include "Math/BHMath.h"
#include <algorithm>

MAKE_MODULE(IllegalAreaProvider);

void IllegalAreaProvider::update(IllegalAreas& illegalAreas)
{
  DECLARE_DEBUG_DRAWING("module:IllegalAreaProvider:illegalAreas", "drawingOnField");

  illegalAreas.illegal = 0u;
  illegalAreas.anticipatedIllegal = 0u;
  illegalAreas.anticipatedTimestamp = 0u;

  illegalAreas.getIllegalAreas = [this, &illegalAreas](const Vector2f& positionOnField, float margin)
  {
    return getIllegalAreas(illegalAreas.illegal, positionOnField, margin);
  };

  illegalAreas.isPositionIllegal = [this, &illegalAreas](const Vector2f& positionOnField, float margin)
  {
    return isPositionIllegal(illegalAreas.illegal, positionOnField, margin);
  };

  illegalAreas.willPositionBeIllegal = [this, &illegalAreas](const Vector2f& positionOnField, float margin)
  {
    return isPositionIllegal(illegalAreas.anticipatedIllegal, positionOnField, margin);
  };

  illegalAreas.willPositionBeIllegalIn = [this, &illegalAreas](const Vector2f& positionOnField, float margin, int duration)
  {
    const unsigned areas = illegalAreas.anticipatedTimestamp && -theFrameInfo.getTimeSince(illegalAreas.anticipatedTimestamp) <= duration ?
                           illegalAreas.anticipatedIllegal : illegalAreas.illegal;
    return isPositionIllegal(areas, positionOnField, margin);
  };

  illegalAreas.isSameIllegalArea = [this, &illegalAreas](const Vector2f& positionOnField, const Vector2f& targetOnField, float margin)
  {
    return isSameIllegalArea(illegalAreas.illegal, positionOnField, targetOnField, margin);
  };

  if(theGameState.isInitial() || theGameState.isFinished())
    return;

  if(theGameState.isPenaltyShootout())
  {
    if(theGameState.isForOpponentTeam())
      illegalAreas.illegal |= bit(IllegalAreas::notOwnGoalLine);
    return;
  }

  // The rules don't make a difference between the goalkeeper and field players, but we want the goalkeeper to always be able to enter the own goal area.
  const bool ownGoalAreaIsFull = !theGameState.isGoalkeeper() && nonKeeperTeammatesInOwnGoalArea() >= 2;

  if(theGameState.isReady())
  {
    illegalAreas.anticipatedTimestamp = theGameState.timeWhenStateEnds;
    illegalAreas.anticipatedIllegal |= bit(IllegalAreas::borderStrip);
    if(ownGoalAreaIsFull)
      illegalAreas.anticipatedIllegal |= bit(IllegalAreas::ownGoalArea);
    if(theGameState.isPenaltyKick())
    {
      if(theGameState.isForOwnTeam())
      {
        if(!Role::isActiveRole(theStrategyStatus.role))
          illegalAreas.anticipatedIllegal |= bit(IllegalAreas::opponentPenaltyArea);
      }
      else
        illegalAreas.anticipatedIllegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyArea);
    }
    else
    {
      illegalAreas.anticipatedIllegal |= bit(IllegalAreas::opponentHalf);
      if(theGameState.isForOpponentTeam())
        illegalAreas.anticipatedIllegal |= bit(IllegalAreas::centerCircle);
    }
  }
  else if(theGameState.isSet())
  {
    illegalAreas.illegal |= bit(IllegalAreas::borderStrip);
    if(ownGoalAreaIsFull)
      illegalAreas.illegal |= bit(IllegalAreas::ownGoalArea);
    if(theGameState.isPenaltyKick())
    {
      if(theGameState.isForOwnTeam())
      {
        if(!Role::isActiveRole(theStrategyStatus.role))
          illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyArea);
      }
      else
        illegalAreas.illegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyArea);
    }
    else
    {
      illegalAreas.illegal |= bit(IllegalAreas::opponentHalf);
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::centerCircle);
    }
  }
  else if(theGameState.isPlaying())
  {
    if(ownGoalAreaIsFull)
      illegalAreas.illegal |= bit(IllegalAreas::ownGoalArea);
    if(theGameState.isPenaltyKick())
    {
      // "During the Penalty Kick: [...] 3. All robots must be within the field-of-play. That is, robots may not be outside the field lines, but within the field border."
      // (rule book section 3.8.1)
      illegalAreas.illegal |= bit(IllegalAreas::borderStrip);

      if(theGameState.isForOwnTeam())
      {
        if(!Role::isActiveRole(theStrategyStatus.role))
          illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyArea);
      }
      else
        illegalAreas.illegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyArea);
    }
    else if(theGameState.isFreeKick())
    {
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::ballArea);
    }
    else if(theGameState.isKickOff())
    {
      illegalAreas.illegal |= bit(IllegalAreas::opponentHalf);
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::centerCircle);
    }
  }

  // If any position that is not on the own goal line is illegal, the other flags don't matter and are actually harmful.
  if(illegalAreas.illegal & bit(IllegalAreas::notOwnGoalLine))
    illegalAreas.illegal = bit(IllegalAreas::notOwnGoalLine);
  if(illegalAreas.anticipatedIllegal & bit(IllegalAreas::notOwnGoalLine))
    illegalAreas.anticipatedIllegal = bit(IllegalAreas::notOwnGoalLine);

  draw(illegalAreas.illegal, illegalAreas.anticipatedIllegal);
}

unsigned IllegalAreaProvider::getIllegalAreas(unsigned illegal, const Vector2f& positionOnField, float margin) const
{
  unsigned illegalAreas = 0u;
  const float halfLineWidth = theFieldDimensions.fieldLinesWidth * 0.5f;
  if((illegal & bit(IllegalAreas::ownGoalArea)) &&
     (positionOnField.x() < theFieldDimensions.xPosOwnGoalArea + halfLineWidth + margin &&
      positionOnField.x() > theFieldDimensions.xPosOwnGoalLine - halfLineWidth - margin &&
      positionOnField.y() > theFieldDimensions.yPosRightGoalArea - halfLineWidth - margin &&
      positionOnField.y() < theFieldDimensions.yPosLeftGoalArea + halfLineWidth + margin))
    illegalAreas |= bit(IllegalAreas::ownGoalArea);
  if((illegal & bit(IllegalAreas::ownPenaltyArea)) &&
     (positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth + margin &&
      positionOnField.x() > theFieldDimensions.xPosOwnGoalLine - halfLineWidth - margin &&
      positionOnField.y() > theFieldDimensions.yPosRightPenaltyArea - halfLineWidth - margin &&
      positionOnField.y() < theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth + margin))
    illegalAreas |= bit(IllegalAreas::ownPenaltyArea);
  if((illegal & bit(IllegalAreas::opponentPenaltyArea)) &&
     (positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth - margin &&
      positionOnField.x() < theFieldDimensions.xPosOpponentGoalLine + halfLineWidth + margin &&
      positionOnField.y() > theFieldDimensions.yPosRightPenaltyArea - halfLineWidth - margin &&
      positionOnField.y() < theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth + margin))
    illegalAreas |= bit(IllegalAreas::opponentPenaltyArea);
  if((illegal & bit(IllegalAreas::borderStrip)) &&
     (positionOnField.x() > theFieldDimensions.xPosOpponentGoalLine + halfLineWidth - margin ||
      positionOnField.x() < theFieldDimensions.xPosOwnGoalLine - halfLineWidth + margin ||
      positionOnField.y() < theFieldDimensions.yPosRightTouchline - halfLineWidth + margin ||
      positionOnField.y() > theFieldDimensions.yPosLeftTouchline + halfLineWidth - margin))
    illegalAreas |= bit(IllegalAreas::borderStrip);
  if((illegal & bit(IllegalAreas::opponentHalf)) &&
     (positionOnField.x() > theFieldDimensions.xPosHalfwayLine - halfLineWidth - margin &&
      positionOnField.squaredNorm() >= sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + halfLineWidth - margin))))
    illegalAreas |= bit(IllegalAreas::opponentHalf);
  if((illegal & bit(IllegalAreas::centerCircle)) &&
     (positionOnField.squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + halfLineWidth + margin))))
    illegalAreas |= bit(IllegalAreas::centerCircle);
  if((illegal & bit(IllegalAreas::ballArea)) &&
     (positionOnField - theFieldBall.recentBallPositionOnField()).squaredNorm() < sqr(std::max(0.f, freeKickClearAreaRadius + margin)))
    illegalAreas |= bit(IllegalAreas::ballArea);
  if((illegal & bit(IllegalAreas::notOwnGoalLine)) &&
     (std::abs(positionOnField.x() - theFieldDimensions.xPosOwnGoalLine) > halfLineWidth - std::min(0.f, margin) ||
      positionOnField.y() > theFieldDimensions.yPosLeftGoal - margin ||
      positionOnField.y() < theFieldDimensions.yPosRightGoal + margin))
    illegalAreas |= bit(IllegalAreas::notOwnGoalLine);
  return illegalAreas;
}

bool IllegalAreaProvider::isPositionIllegal(unsigned illegal, const Vector2f& positionOnField, float margin) const
{
  return getIllegalAreas(illegal, positionOnField, margin);
}

bool IllegalAreaProvider::isSameIllegalArea(unsigned illegal, const Vector2f& positionOnField, const Vector2f& targetOnField, float margin) const
{
  return getIllegalAreas(illegal, positionOnField, margin) & getIllegalAreas(illegal, targetOnField, margin);
}

unsigned int IllegalAreaProvider::nonKeeperTeammatesInOwnGoalArea() const
{
  float distanceThreshold = outsideGoalAreaDistanceThreshold;
  bool isNear = false;
  if(theLibPosition.isNearOwnGoalArea(theRobotPose.translation, -theRobotPose.getXAxisStandardDeviation(), -theRobotPose.getYAxisStandardDeviation()))
    distanceThreshold = insideGoalAreaDistanceThreshold;
  else if(theLibPosition.isNearOwnGoalArea(theRobotPose.translation, outsideGoalAreaDistanceThreshold, outsideGoalAreaDistanceThreshold))
    isNear = true;
  unsigned int teammatesInGoalArea = 0;
  for(const auto& agent : theAgentStates.agents)
  {
    if(agent.number == theGameState.playerNumber)
      continue;
    const Vector2f& teammatePosition = agent.currentPosition;
    if(!agent.isGoalkeeper
       && theLibPosition.isNearOwnGoalArea(teammatePosition, distanceThreshold, distanceThreshold))
    {
      if(isNear && !theLibPosition.isNearOwnGoalArea(teammatePosition, insideGoalAreaDistanceThreshold, insideGoalAreaDistanceThreshold))
      {
        /// breaks ties between two robots attempting to enter simultaneously
        if(agent.role == ActiveRole::toRole(ActiveRole::playBall))
          ++teammatesInGoalArea;
      }
      else
        ++teammatesInGoalArea;
    }
  }
  return teammatesInGoalArea;
}

void IllegalAreaProvider::draw(unsigned illegal, unsigned anticipatedIllegal) const
{
  COMPLEX_DRAWING("module:IllegalAreaProvider:illegalAreas")
  {
    const float halfLineWidth = theFieldDimensions.fieldLinesWidth * 0.5f;
    const ColorRGBA brushColor(255, 0, 0, 50);

    ColorRGBA color;
    if(anticipatedIllegal)
    {
      color = ColorRGBA::yellow;
    }
    else
    {
      color = ColorRGBA::red;
    }

    if((illegal | anticipatedIllegal) & bit(IllegalAreas::ownGoalArea))
    {
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosOwnGoalArea + halfLineWidth, theFieldDimensions.yPosRightGoalArea - halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGoalArea + halfLineWidth, theFieldDimensions.yPosLeftGoalArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosLeftGoalArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosRightGoalArea - halfLineWidth)
                                                   };

      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
    }
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::ownPenaltyArea))
    {
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth)
                                                   };
      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
    }
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::opponentPenaltyArea))
    {
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                    Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth)
                                                   };
      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
    }
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::opponentHalf))
    {
      // TODO: Exclude center circle + halfLineWidth
      const std::vector<Vector2f> illegalPolygon = {Vector2f(theFieldDimensions.xPosHalfwayLine - halfLineWidth, theFieldDimensions.yPosRightTouchline),
                                                    Vector2f(theFieldDimensions.xPosHalfwayLine - halfLineWidth, theFieldDimensions.yPosLeftTouchline),
                                                    Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosLeftTouchline),
                                                    Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosRightTouchline)
                                                   };
      POLYGON("module:IllegalAreaProvider:illegalAreas", illegalPolygon.size(), illegalPolygon.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
    }
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::centerCircle))
    {
      CIRCLE("module:IllegalAreaProvider:illegalAreas", 0.f, 0.f, theFieldDimensions.centerCircleRadius + halfLineWidth, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
    }
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::ballArea))
    {
      CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldBall.recentBallPositionOnField().x(), theFieldBall.recentBallPositionOnField().y(), freeKickClearAreaRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
    }
  }
}
