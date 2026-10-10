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
  illegalAreas.illegal |= bit(IllegalAreas::fieldBorder);
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

  const auto handlePenaltyKick = [this, &illegalAreas]()
  {
    if(theGameState.isForOwnTeam())
    {
      if(!Role::isActiveRole(theStrategyStatus.role))
        illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyMarkArea);
    }
    else
      illegalAreas.illegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyMarkArea);
  };

  // The rules don't make a difference between the goalkeeper and field players, but we want the goalkeeper to always be able to enter the own goal area.
  const bool ownGoalAreaIsFull = !theGameState.isGoalkeeper() && (nonKeeperTeammatesInOwnGoalArea() >= 2 || !isGoalAreaAllowed());

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
          illegalAreas.anticipatedIllegal |= bit(IllegalAreas::opponentPenaltyMarkArea);
      }
      else
        illegalAreas.anticipatedIllegal |= bit(theGameState.isGoalkeeper() ? IllegalAreas::notOwnGoalLine : IllegalAreas::ownPenaltyMarkArea);
    }
    else
    {
      illegalAreas.anticipatedIllegal |= bit(IllegalAreas::opponentHalf);
      if(!theGameState.isForOwnTeam())
        illegalAreas.anticipatedIllegal |= bit(IllegalAreas::centerCircle);
    }
  }
  else if(theGameState.isSet())
  {
    illegalAreas.illegal |= bit(IllegalAreas::borderStrip);
    if(ownGoalAreaIsFull)
      illegalAreas.illegal |= bit(IllegalAreas::ownGoalArea);
    if(theGameState.isPenaltyKick())
      handlePenaltyKick();
    else
    {
      illegalAreas.illegal |= bit(IllegalAreas::opponentHalf);
      if(!theGameState.isForOwnTeam())
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

      handlePenaltyKick();
    }
    else if(theGameState.isFreeKick())
    {
      if(theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::ballArea);

      // Rule 16.2: "For a goal kick, the avoidance region is the entire penalty area. All robots of the opposing team
      // must remain outside the penalty area until the ball is in play"
      if(theGameState.isGoalKick() && theGameState.isForOpponentTeam())
        illegalAreas.illegal |= bit(IllegalAreas::opponentPenaltyArea);
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

  bool isInOwnPenaltyArea = (positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth + margin &&
                             positionOnField.x() > theFieldDimensions.xPosOwnGoalLine - halfLineWidth - margin &&
                             positionOnField.y() > theFieldDimensions.yPosRightPenaltyArea - halfLineWidth - margin &&
                             positionOnField.y() < theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth + margin);
  bool isBetweenOwnPenaltyMarkAndGoal = (positionOnField.x() < theFieldDimensions.xPosOwnPenaltyMark + theFieldDimensions.markRadius + margin &&
                                         positionOnField.x() > theFieldDimensions.xPosOwnGoalLine - halfLineWidth - margin &&
                                         positionOnField.y() > theFieldDimensions.yPosRightTouchline - halfLineWidth - margin &&
                                         positionOnField.y() < theFieldDimensions.yPosLeftTouchline + halfLineWidth + margin);
  bool isInOwnPenaltyMarkRadius = (positionOnField - Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0)).squaredNorm() < sqr(std::abs(theFieldDimensions.centerCircleRadius + theFieldDimensions.markRadius + margin));
  bool isInOpponentPenaltyArea = (positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth - margin &&
                                  positionOnField.x() < theFieldDimensions.xPosOpponentGoalLine + halfLineWidth + margin &&
                                  positionOnField.y() > theFieldDimensions.yPosRightPenaltyArea - halfLineWidth - margin &&
                                  positionOnField.y() < theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth + margin);
  bool isBetweenOpponentPenaltyMarkAndGoal = (positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.markRadius - margin &&
                                              positionOnField.x() < theFieldDimensions.xPosOpponentGoalLine + halfLineWidth + margin &&
                                              positionOnField.y() > theFieldDimensions.yPosRightTouchline - halfLineWidth - margin &&
                                              positionOnField.y() < theFieldDimensions.yPosLeftTouchline + halfLineWidth + margin);
  bool isInOpponentPenaltyMarkRadius = (positionOnField - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0)).squaredNorm() < sqr(std::abs(theFieldDimensions.centerCircleRadius + theFieldDimensions.markRadius + margin));

  if((illegal & bit(IllegalAreas::ownPenaltyArea)) && isInOwnPenaltyArea)
    illegalAreas |= bit(IllegalAreas::ownPenaltyArea);
  if((illegal & bit(IllegalAreas::ownPenaltyMarkArea)) && (isInOwnPenaltyArea || isBetweenOwnPenaltyMarkAndGoal || isInOwnPenaltyMarkRadius))
    illegalAreas |= bit(IllegalAreas::ownPenaltyMarkArea);
  if((illegal & bit(IllegalAreas::opponentPenaltyArea)) && isInOpponentPenaltyArea)
    illegalAreas |= bit(IllegalAreas::opponentPenaltyArea);
  if((illegal & bit(IllegalAreas::opponentPenaltyMarkArea)) && (isInOpponentPenaltyArea || isBetweenOpponentPenaltyMarkAndGoal || isInOpponentPenaltyMarkRadius))
    illegalAreas |= bit(IllegalAreas::opponentPenaltyMarkArea);
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
     (positionOnField - theFieldBall.recentBallPositionOnField()).squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + margin)) &&
     (theTeamBallModel.isValid || theFieldBall.timeSinceBallWasSeen < theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) + ballSeenBeforeFreeKickTime))
    illegalAreas |= bit(IllegalAreas::ballArea);
  if((illegal & bit(IllegalAreas::ballArea)) && //Dummy Ball positions for goalkick
     ((positionOnField - Vector2f(theFieldDimensions.xPosOpponentGoalArea,theFieldDimensions.yPosLeftGoalArea)).squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + margin))
     || (positionOnField - Vector2f(theFieldDimensions.xPosOpponentGoalArea,theFieldDimensions.yPosRightGoalArea)).squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + margin))) &&
     (theTeamBallModel.isValid || theFieldBall.timeSinceBallWasSeen < theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) + ballSeenBeforeFreeKickTime) && (theGameState.isGoalKick()) && theGameState.isForOpponentTeam() )
    illegalAreas |= bit(IllegalAreas::ballArea);
  if((illegal & bit(IllegalAreas::ballArea)) && //Dummy Ball positions for cornerkick
     ((positionOnField - Vector2f(theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline)).squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + margin))
     || (positionOnField - Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline)).squaredNorm() < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + margin))) &&
     (theTeamBallModel.isValid || theFieldBall.timeSinceBallWasSeen < theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) + ballSeenBeforeFreeKickTime) && (theGameState.isCornerKick()) && theGameState.isForOpponentTeam() )
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

bool IllegalAreaProvider::isGoalAreaAllowed() const
{
  if(!theGameState.isGoalKick() || theGameState.isForOpponentTeam() || theGameState.isGoalkeeper() || -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds) < ownGoalKickTimeIgnoreGoalKeeper)
    return true;

  for(const auto& agent : theAgentStates.agents)
  {
    if(!agent.isGoalkeeper)
      continue;
    const Vector2f& teammatePosition = agent.currentPosition;
    const Vector2f fieldBallPosition = theFieldBall.recentBallPositionOnField();
    // GoalKeeper is standing on the other side of the goal area
    if(fieldBallPosition.y() * teammatePosition.y() <= 0.f)
      return true;

    return std::abs(teammatePosition.y()) < theFieldDimensions.yPosLeftGoalArea - goalKeeperDistanceToBallGoalKick || // GoalKeeper reached a position far away from the ball
           agent.lastKnownTarget.y() * fieldBallPosition.y() > 0.f || // GoalKeeper does not plan to reach a position far away from the ball
           std::abs(agent.lastKnownTarget.y()) >= theFieldDimensions.yPosLeftGoalArea - goalKeeperDistanceToBallGoalKick;
  }

  return true;
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
        // breaks ties between two robots attempting to enter simultaneously
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
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::ownPenaltyMarkArea))
    {
      const std::vector<Vector2f> penaltyArea = {Vector2f(theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth),
                                                 Vector2f(theFieldDimensions.xPosOwnPenaltyArea + halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                 Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                 Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth)
                                                };
      const std::vector<Vector2f> betweenGoalAndPenaltyMark = {Vector2f(theFieldDimensions.xPosOwnPenaltyMark + theFieldDimensions.markRadius, theFieldDimensions.yPosRightTouchline - halfLineWidth),
                                                               Vector2f(theFieldDimensions.xPosOwnPenaltyMark + theFieldDimensions.markRadius, theFieldDimensions.yPosLeftTouchline + halfLineWidth),
                                                               Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosLeftTouchline + halfLineWidth),
                                                               Vector2f(theFieldDimensions.xPosOwnGoalLine - halfLineWidth, theFieldDimensions.yPosRightTouchline - halfLineWidth)
                                                              };
      POLYGON("module:IllegalAreaProvider:illegalAreas", penaltyArea.size(), penaltyArea.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      POLYGON("module:IllegalAreaProvider:illegalAreas", betweenGoalAndPenaltyMark.size(), betweenGoalAndPenaltyMark.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldDimensions.xPosOwnPenaltyMark, 0, theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
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
    if((illegal | anticipatedIllegal) & bit(IllegalAreas::opponentPenaltyMarkArea))
    {
      const std::vector<Vector2f> penaltyArea = {Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth),
                                                 Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                 Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosLeftPenaltyArea + halfLineWidth),
                                                 Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosRightPenaltyArea - halfLineWidth)
                                                };
      const std::vector<Vector2f> betweenGoalAndPenaltyMark = {Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.markRadius, theFieldDimensions.yPosRightTouchline - halfLineWidth),
                                                               Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.markRadius, theFieldDimensions.yPosLeftTouchline + halfLineWidth),
                                                               Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosLeftTouchline + halfLineWidth),
                                                               Vector2f(theFieldDimensions.xPosOpponentGoalLine + halfLineWidth, theFieldDimensions.yPosRightTouchline - halfLineWidth)
                                                              };
      POLYGON("module:IllegalAreaProvider:illegalAreas", penaltyArea.size(), penaltyArea.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      POLYGON("module:IllegalAreaProvider:illegalAreas", betweenGoalAndPenaltyMark.size(), betweenGoalAndPenaltyMark.data(), 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldDimensions.xPosOpponentPenaltyMark, 0, theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
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
      CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldBall.recentBallPositionOnField().x(), theFieldBall.recentBallPositionOnField().y(), theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      if(theGameState.isGoalKick() && theGameState.isForOpponentTeam())
      {
        CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea, theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
        CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea, theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      }
      if((theGameState.isCornerKick()) && theGameState.isForOpponentTeam())
      {
        CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline, theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
        CIRCLE("module:IllegalAreaProvider:illegalAreas", theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline, theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, color, Drawings::solidBrush, brushColor);
      }
    }
  }
}
