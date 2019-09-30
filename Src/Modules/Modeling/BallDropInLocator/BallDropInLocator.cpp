/**
 * @file BallDropInLocator.cpp
 *
 * This file implements a module that computes the position where the ball is put after it goes out.
 *
 * @author Arne Hasselbring, Nicole Schrader
 */

#include "BallDropInLocator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"

#include <algorithm>

MAKE_MODULE(BallDropInLocator, modeling);

BallDropInLocator::BallDropInLocator()
{
  for(unsigned int i = 0; i < numOfTouchedBys; ++i)
    lastTouchEvents[i].timestamp = 0;
}

void BallDropInLocator::update(BallDropInModel& ballDropInModel)
{
  DECLARE_DEBUG_DRAWING("module:BallDropInLocator:ballTouchEvents", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallDropInLocator:predictedOutPosition", "drawingOnField");

  ballDropInModel.isValid = false;
  ballDropInModel.dropInPositions.clear();

  if(theGameInfo.state != STATE_PLAYING || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    for(unsigned int i = 0; i < numOfTouchedBys; ++i)
      lastTouchEvents[i].timestamp = 0;
    ballWasOnField = true;
    return;
  }

  updateTouchPositions();
  updateBall(ballDropInModel);
  updateGameControllerData(ballDropInModel);

  const bool outLeft = (useOutPosition ? ballDropInModel.outPosition.y() : predictedOutPosition.y()) >= 0.f;

  switch(ballDropInModel.dropInType)
  {
    case BallDropInModel::goalFreeKick:
      ballDropInModel.dropInPositions.emplace_back(
        ownTeamTouchedLast ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark,
        outLeft ? theFieldDimensions.yPosLeftPenaltyArea : theFieldDimensions.yPosRightPenaltyArea);
      ballDropInModel.dropInPositions.emplace_back(
        ownTeamTouchedLast ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark,
        outLeft ? theFieldDimensions.yPosRightPenaltyArea : theFieldDimensions.yPosLeftPenaltyArea);
      ballDropInModel.isValid = useOutPosition || theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK;
      break;
    case BallDropInModel::cornerKick:
      ballDropInModel.dropInPositions.emplace_back(
        ownTeamTouchedLast ? theFieldDimensions.xPosOwnGroundline : theFieldDimensions.xPosOpponentGroundline,
        outLeft ? theFieldDimensions.yPosLeftSideline : theFieldDimensions.yPosRightSideline);
      ballDropInModel.dropInPositions.emplace_back(
        ownTeamTouchedLast ? theFieldDimensions.xPosOwnGroundline : theFieldDimensions.xPosOpponentGroundline,
        outLeft ? theFieldDimensions.yPosRightSideline : theFieldDimensions.yPosLeftSideline);
      ballDropInModel.isValid = useOutPosition || theGameInfo.setPlay == SET_PLAY_CORNER_KICK;
      break;
    case BallDropInModel::kickIn:
      ballDropInModel.dropInPositions.emplace_back(
        clip(useOutPosition ? ballDropInModel.outPosition.x() : predictedOutPosition.x(),
             theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOpponentGroundline),
        outLeft ? theFieldDimensions.yPosLeftSideline : theFieldDimensions.yPosRightSideline);
      ballDropInModel.isValid = useOutPosition || theGameInfo.setPlay == SET_PLAY_KICK_IN;
      break;
    default:
      break;
  }

  draw();
}

void BallDropInLocator::updateTouchPositions()
{
  // When entering a ball replacing free kick, nothing should depend on old events anymore.
  if((theCognitionStateChanges.lastSetPlay != SET_PLAY_GOAL_FREE_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK) ||
     (theCognitionStateChanges.lastSetPlay != SET_PLAY_CORNER_KICK && theGameInfo.setPlay == SET_PLAY_CORNER_KICK) ||
     (theCognitionStateChanges.lastSetPlay != SET_PLAY_KICK_IN && theGameInfo.setPlay == SET_PLAY_KICK_IN))
  {
    for(unsigned int i = 0; i < numOfTouchedBys; ++i)
      lastTouchEvents[i].timestamp = 0;
    return;
  }

  if(!theTeamBallModel.isValid)
    return;

  if((theRobotPose.translation - theTeamBallModel.position).squaredNorm() < ballTouchThresholdSquared)
  {
    lastTouchEvents[ownTeam].timestamp = theFrameInfo.time;
    lastTouchEvents[ownTeam].positionOnField = theRobotPose.translation;
  }

  for(const auto& teammate : theTeamData.teammates)
  {
    if(teammate.status == Teammate::PENALIZED
       && theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET
       && theFrameInfo.getTimeSince(teammate.timeWhenStatusChanged) > timeUntilPenalizedRobotsAreRemoved)
      continue;
    if((teammate.theRobotPose.translation - theTeamBallModel.position).squaredNorm() < ballTouchThresholdSquared)
    {
      lastTouchEvents[ownTeam].timestamp = theFrameInfo.time;
      lastTouchEvents[ownTeam].positionOnField = teammate.theRobotPose.translation;
    }
  }

  for(const auto& obstacle : theTeamPlayersModel.obstacles)
    if(!obstacle.isTeammate() && obstacle.type != Obstacle::goalpost
       && (obstacle.center - theTeamBallModel.position).squaredNorm() < ballTouchThresholdSquared)
    {
      const TouchedBy touchedBy = obstacle.isOpponent() ? opponentTeam : unknown;
      lastTouchEvents[touchedBy].timestamp = theFrameInfo.time;
      lastTouchEvents[touchedBy].positionOnField = obstacle.center;
    }
}

void BallDropInLocator::updateBall(BallDropInModel& ballDropInModel)
{
  if(!theTeamBallModel.isValid)
    return;

  auto isInsideCorrected = [this](const Vector2f& ballOnField, float addOffset) -> bool
  {
    // Yes, this is not exact in the corners :)
    // and assumes that the field is longer and wider than 2 * `offset`...
    const float offset = theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius + addOffset;
    return theFieldDimensions.isInsideField(Vector2f(ballOnField.x() - sgn(ballOnField.x()) * offset, ballOnField.y() - sgn(ballOnField.y()) * offset));
  };

  if(isInsideCorrected(theTeamBallModel.position, ballWasOnField ? safetyMargin : 0.f))
  {
    if(theTeamBallModel.velocity.squaredNorm() < 1.f)
      predictedOutPosition = theTeamBallModel.position;
    else
    {
      const bool movingToLeft = theTeamBallModel.velocity.y() > 0.f;
      const bool movingToOpponent = theTeamBallModel.velocity.x() > 0.f;
      const float offset = theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius;
      const Geometry::Line sideline(Vector2f(0.f, movingToLeft ? (theFieldDimensions.yPosLeftSideline + offset) : (theFieldDimensions.yPosRightSideline - offset)), Vector2f(1.f, 0.f));
      const Geometry::Line groundline(Vector2f(movingToOpponent ? (theFieldDimensions.xPosOpponentGroundline + offset) : (theFieldDimensions.xPosOwnGroundline - offset), 0.f), Vector2f(0.f, 1.f));
      const Geometry::Line ballDirection(theTeamBallModel.position, theTeamBallModel.velocity);
      Vector2f intersection;
      if(Geometry::getIntersectionOfLines(ballDirection, sideline, intersection) &&
         intersection.x() >= theFieldDimensions.xPosOwnGroundline - offset && intersection.x() <= theFieldDimensions.xPosOpponentGroundline + offset)
        predictedOutPosition = intersection;
      else if(Geometry::getIntersectionOfLines(ballDirection, groundline, intersection) &&
              intersection.y() >= theFieldDimensions.yPosRightSideline - offset && intersection.y() <= theFieldDimensions.yPosLeftSideline + offset)
        predictedOutPosition = intersection;
    }
    useOutPosition = false;
    ballWasOnField = true;
    ballDropInModel.lastTimeWhenBallWasOnTheField = theFrameInfo.time;
  }
  else if(ballWasOnField)
  {
    ballDropInModel.outPosition = theTeamBallModel.position;
    ballDropInModel.lastTimeWhenBallOutWasObserved = theFrameInfo.time;
    if(theGameInfo.setPlay == SET_PLAY_NONE)
    {
      ownTeamTouchedLast = lastTouchEvents[ownTeam].timestamp >= std::max(lastTouchEvents[opponentTeam].timestamp, lastTouchEvents[unknown].timestamp);
      if((theTeamBallModel.position.x() > 0.f == ownTeamTouchedLast) &&
         (theTeamBallModel.position.y() < theFieldDimensions.yPosLeftSideline) &&
         (theTeamBallModel.position.y() > theFieldDimensions.yPosRightSideline))
        ballDropInModel.dropInType = BallDropInModel::goalFreeKick;
      else if((theTeamBallModel.position.x() > 0.f != ownTeamTouchedLast) &&
              (theTeamBallModel.position.y() < theFieldDimensions.yPosLeftSideline) &&
              (theTeamBallModel.position.y() > theFieldDimensions.yPosRightSideline))
        ballDropInModel.dropInType = BallDropInModel::cornerKick;
      else
        ballDropInModel.dropInType = BallDropInModel::kickIn;
    }
    useOutPosition = true;
    ballWasOnField = false;
  }
}

void BallDropInLocator::updateGameControllerData(BallDropInModel& ballDropInModel)
{
  if(theCognitionStateChanges.lastSetPlay != SET_PLAY_GOAL_FREE_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK)
  {
    ownTeamTouchedLast = theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
    ballDropInModel.dropInType = BallDropInModel::goalFreeKick;
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
  else if(theCognitionStateChanges.lastSetPlay != SET_PLAY_CORNER_KICK && theGameInfo.setPlay == SET_PLAY_CORNER_KICK)
  {
    ownTeamTouchedLast = theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
    ballDropInModel.dropInType = BallDropInModel::cornerKick;
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
  else if(theCognitionStateChanges.lastSetPlay != SET_PLAY_KICK_IN && theGameInfo.setPlay == SET_PLAY_KICK_IN)
  {
    ownTeamTouchedLast = theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
    ballDropInModel.dropInType = BallDropInModel::kickIn;
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
}

void BallDropInLocator::draw() const
{
  // Draw events as crosses in the color of the team that touched the ball.
  DEBUG_DRAWING("module:BallDropInLocator:ballTouchEvents", "drawingOnField")
  {
    for(unsigned int i = 0; i < numOfTouchedBys; ++i)
    {
      ColorRGBA color = i != unknown
                        ? ColorRGBA::fromTeamColor((i == ownTeam) ? theOwnTeamInfo.teamColor : theOpponentTeamInfo.teamColor)
                        : ColorRGBA::orange;
      const int age = theFrameInfo.getTimeSince(lastTouchEvents[i].timestamp);
      if(age > rememberEventDuration)
        continue;
      color.a = static_cast<unsigned char>(255 - 255 * age / rememberEventDuration);
      CROSS("module:BallDropInLocator:ballTouchEvents", lastTouchEvents[i].positionOnField.x(), lastTouchEvents[i].positionOnField.y(), 75, 30, Drawings::solidPen, color);
    }
  }

  DEBUG_DRAWING("module:BallDropInLocator:predictedOutPosition", "drawingOnField")
  {
    CROSS("module:BallDropInLocator:predictedOutPosition", predictedOutPosition.x(), predictedOutPosition.y(), 75, 30, Drawings::solidPen, ColorRGBA::black);
  }
}
