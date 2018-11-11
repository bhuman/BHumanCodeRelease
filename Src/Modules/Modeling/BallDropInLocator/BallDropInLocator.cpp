/**
 * @file BallDropInLocator.cpp
 *
 * This file implements a module that computes the position where the ball is put after it goes out.
 *
 * @author Arne Hasselbring
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

  ballDropInModel.isValid = false;

  if(theGameInfo.state != STATE_PLAYING || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    for(unsigned int i = 0; i < numOfTouchedBys; ++i)
      lastTouchEvents[i].timestamp = 0;
    return;
  }

  updateTouchPositions();
  updateBall(ballDropInModel);
  updateGameControllerData(ballDropInModel);

  // TODO: Check in which direction the ball was rolling.
  const bool outLeft = (useOutPosition ? ballDropInModel.outPosition.y() : lastBallPositionLyingInsideField.y()) >= 0.f;

  if(ballDropInModel.isGoalFreeKickOut)
  {
    ballDropInModel.dropInPosition.x() = ownTeamTouchedLast ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark;
    ballDropInModel.dropInPosition.y() = outLeft ? theFieldDimensions.yPosLeftPenaltyArea : theFieldDimensions.yPosRightPenaltyArea;

    ballDropInModel.isValid = true;
  }
  else
  {
    Vector2f lastTouchPosition;
    if(determineLastTouchPosition(lastTouchPosition))
    {
      ballDropInModel.dropInPosition.x() = lastTouchPosition.x() + (ownTeamTouchedLast ? -dropInPenaltyDistance : dropInPenaltyDistance);
      ballDropInModel.dropInPosition.x() = clip(ballDropInModel.dropInPosition.x(), theFieldDimensions.xPosOwnDropInLine, theFieldDimensions.xPosOpponentDropInLine);
      ballDropInModel.dropInPosition.y() = outLeft ? theFieldDimensions.yPosLeftDropInLine : theFieldDimensions.yPosRightDropInLine;

      ballDropInModel.isValid = true;
    }
  }

  draw();
}

void BallDropInLocator::updateTouchPositions()
{
  // When entering a goal free kick, nothing should depend on old events anymore.
  if(theCognitionStateChanges.lastSetPlay != SET_PLAY_GOAL_FREE_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK)
  {
    for(unsigned int i = 0; i < numOfTouchedBys; ++i)
      lastTouchEvents[i].timestamp = 0;
    return;
  }

  if(theTeamBallModel.isValid)
  {
    // TODO: This can be improved, e.g. by considering the trajectory of the ball and whether it has crossed the circle of some robot.

    // TODO: Do something sensible when multiple robots are in the ball radius at once.
    // It probably does not matter anyway because their positions can't differ too much.

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
}

void BallDropInLocator::updateBall(BallDropInModel& ballDropInModel)
{
  if(!theTeamBallModel.isValid)
    return;

  auto isInsideCorrected = [this](const Vector2f& ballOnField) -> bool
  {
    // Yes, this is not exact in the corners :)
    // and assumes that the field is longer and wider than 2 * `offset`...
    const float offset = theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius + safetyMargin;
    return theFieldDimensions.isInsideField(Vector2f(ballOnField.x() - sgn(ballOnField.x()) * offset, ballOnField.y() - sgn(ballOnField.y()) * offset));
  };

  if(isInsideCorrected(theTeamBallModel.position))
  {
    if(theTeamBallModel.velocity.squaredNorm() < 1.f)
    {
      lastBallPositionLyingInsideField = theTeamBallModel.position;
      useOutPosition = false;
    }
    ballDropInModel.lastTimeWhenBallWasOnTheField = theFrameInfo.time;
  }
  else if(isInsideCorrected(lastBallPosition))
  {
    ballDropInModel.outPosition = theTeamBallModel.position;
    ballDropInModel.lastTimeWhenBallOutWasObserved = theFrameInfo.time;
    if(theFrameInfo.getTimeSince(ballDropInModel.lastTimeWhenBallWentOut) > 3000)
    {
      // TODO: add "don't know" state
      ownTeamTouchedLast = lastTouchEvents[ownTeam].timestamp >= std::max(lastTouchEvents[opponentTeam].timestamp, lastTouchEvents[unknown].timestamp);
      if(theTeamBallModel.position.x() > 0.f == ownTeamTouchedLast)
      {
        const float offset = theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius;
        const Geometry::Line groundLine(Vector2f(ownTeamTouchedLast ? theFieldDimensions.xPosOpponentGroundline + offset : theFieldDimensions.xPosOwnGroundline - offset, 0.f), Vector2f(1.f, 0.f));
        const Geometry::Line ballDirection(lastBallPosition, theTeamBallModel.position - lastBallPosition);
        Vector2f intersection;
        ballDropInModel.isGoalFreeKickOut = Geometry::getIntersectionOfLines(groundLine, ballDirection, intersection)
                                            && std::abs(intersection.y()) < theFieldDimensions.yPosLeftSideline;
      }
      else
        ballDropInModel.isGoalFreeKickOut = false;
    }
    useOutPosition = true;
  }
  lastBallPosition = theTeamBallModel.position;
}

void BallDropInLocator::updateGameControllerData(BallDropInModel& ballDropInModel)
{
  if(theGameInfo.dropInTime == 0 && theGameInfo.dropInTime != lastDropInTime)
  {
    ownTeamTouchedLast = theGameInfo.dropInTeam == theOwnTeamInfo.teamNumber;
    ballDropInModel.isGoalFreeKickOut = false;
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
  else if(theCognitionStateChanges.lastSetPlay != SET_PLAY_GOAL_FREE_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK)
  {
    ownTeamTouchedLast = theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
    ballDropInModel.isGoalFreeKickOut = true;
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
  lastDropInTime = theGameInfo.dropInTime;
}

bool BallDropInLocator::determineLastTouchPosition(Vector2f& lastTouchPosition) const
{
  const BallTouchEvent* usedEvent = &lastTouchEvents[ownTeamTouchedLast ? ownTeam : opponentTeam];
  // If no event has been observed with a certain affiliation, the unknown event may be used.
  if(theFrameInfo.getTimeSince(usedEvent->timestamp) > rememberEventDuration)
    usedEvent = &lastTouchEvents[unknown];
  // TODO: Maybe the unknown event should be used if not ownTeamKicked and it is newer than the newest opponentTeam event.

  lastTouchPosition = usedEvent->positionOnField;
  return theFrameInfo.getTimeSince(usedEvent->timestamp) <= rememberEventDuration;
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
}
