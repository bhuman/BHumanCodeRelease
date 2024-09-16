/**
 * @file BallDropInLocator.cpp
 *
 * This file implements a module that computes the position where the ball is put after it goes out.
 *
 * @author Arne Hasselbring, Nicole Schrader
 */

#include "BallDropInLocator.h"
#include "Debugging/DebugDrawings.h"
#include "Math/BHMath.h"

#include <algorithm>

MAKE_MODULE(BallDropInLocator);

void BallDropInLocator::update(BallDropInModel& ballDropInModel)
{
  DECLARE_DEBUG_DRAWING("module:BallDropInLocator:ballTouchEvents", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallDropInLocator:predictedOutPosition", "drawingOnField");

  ballDropInModel.isValid = false;
  ballDropInModel.dropInPositions.clear();

  if(!theGameState.isPlaying() || theGameState.isPenaltyShootout())
  {
    ballWasOnField = true;
    return;
  }

  updateBall(ballDropInModel);
  updateGameControllerData(ballDropInModel);

  if(useOutPosition && !theGameState.isFreeKick() && !theGameState.isPenaltyKick() && theFrameInfo.getTimeSince(ballDropInModel.lastTimeWhenBallOutWasObserved) > useOutPositionTimeout)
    useOutPosition = false;

  const bool outLeft = (useOutPosition ? ballDropInModel.outPosition.y() : predictedOutPosition.y()) >= 0.f;

  if(ballDropInModel.dropInTypes & bit(BallDropInModel::ownGoalKick))
  {
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOwnGoalArea,
      outLeft ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOwnGoalArea,
      outLeft ? theFieldDimensions.yPosRightGoalArea : theFieldDimensions.yPosLeftGoalArea);
  }
  if(ballDropInModel.dropInTypes & bit(BallDropInModel::ownCornerKick))
  {
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOpponentGoalLine,
      outLeft ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline);
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOpponentGoalLine,
      outLeft ? theFieldDimensions.yPosRightTouchline : theFieldDimensions.yPosLeftTouchline);
  }
  if(ballDropInModel.dropInTypes & bit(BallDropInModel::opponentGoalKick))
  {
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOpponentGoalArea,
      outLeft ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOpponentGoalArea,
      outLeft ? theFieldDimensions.yPosRightGoalArea : theFieldDimensions.yPosLeftGoalArea);
  }
  if(ballDropInModel.dropInTypes & bit(BallDropInModel::opponentCornerKick))
  {
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOwnGoalLine,
      outLeft ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline);
    ballDropInModel.dropInPositions.emplace_back(theFieldDimensions.xPosOwnGoalLine,
      outLeft ? theFieldDimensions.yPosRightTouchline : theFieldDimensions.yPosLeftTouchline);
  }
  if(ballDropInModel.dropInTypes & bit(BallDropInModel::kickIn))
  {
    ballDropInModel.dropInPositions.emplace_back(
      clip(useOutPosition ? ballDropInModel.outPosition.x() : predictedOutPosition.x(),
           theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.xPosOpponentGoalLine),
      outLeft ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline);
  }
  ballDropInModel.isValid = useOutPosition || theGameState.isGoalKick() || theGameState.isCornerKick() || theGameState.isKickIn();

  draw();
}

void BallDropInLocator::updateBall(BallDropInModel& ballDropInModel)
{
  Vector2f ballPositionOnField;
  Vector2f ballVelocityOnField;

  if(theTeammatesBallModel.isValid && theTeammatesBallModel.newerThanOwnBall)
  {
    ballPositionOnField = theTeammatesBallModel.position;
    ballVelocityOnField = theTeammatesBallModel.velocity;
  }
  else if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 8000)
  {
    ballPositionOnField = theRobotPose * theBallModel.estimate.position;
    ballVelocityOnField = theBallModel.estimate.velocity;
    ballVelocityOnField.rotate(theRobotPose.rotation);
  }
  else
  {
    return;
  }

  auto isInsideCorrected = [this](const Vector2f& ballOnField, float addOffset) -> bool
  {
    // Yes, this is not exact in the corners :)
    // and assumes that the field is longer and wider than 2 * `offset`...
    const float offset = theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius + addOffset;
    return theFieldDimensions.isInsideField(Vector2f(ballOnField.x() - sgn(ballOnField.x()) * offset, ballOnField.y() - sgn(ballOnField.y()) * offset));
  };

  if(isInsideCorrected(ballPositionOnField, ballWasOnField ? safetyMargin : 0.f))
  {
    if(ballVelocityOnField.squaredNorm() < 1.f)
      predictedOutPosition = ballPositionOnField;
    else
    {
      const bool movingToLeft = ballVelocityOnField.y() > 0.f;
      const bool movingToOpponent = ballVelocityOnField.x() > 0.f;
      const float offset = theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius;
      const Geometry::Line touchline(Vector2f(0.f, movingToLeft ? (theFieldDimensions.yPosLeftTouchline + offset) : (theFieldDimensions.yPosRightTouchline - offset)), Vector2f(1.f, 0.f));
      const Geometry::Line goalLine(Vector2f(movingToOpponent ? (theFieldDimensions.xPosOpponentGoalLine + offset) : (theFieldDimensions.xPosOwnGoalLine - offset), 0.f), Vector2f(0.f, 1.f));
      const Geometry::Line ballDirection(ballPositionOnField, ballVelocityOnField);
      ballDropInModel.ballDirection = ballDirection.direction;
      Vector2f intersection;
      if(Geometry::getIntersectionOfLines(ballDirection, touchline, intersection) &&
         intersection.x() >= theFieldDimensions.xPosOwnGoalLine - offset && intersection.x() <= theFieldDimensions.xPosOpponentGoalLine + offset)
        predictedOutPosition = intersection;
      else if(Geometry::getIntersectionOfLines(ballDirection, goalLine, intersection) &&
              intersection.y() >= theFieldDimensions.yPosRightTouchline - offset && intersection.y() <= theFieldDimensions.yPosLeftTouchline + offset)
        predictedOutPosition = intersection;
    }
    useOutPosition = false;
    ballWasOnField = true;
    ballDropInModel.lastTimeWhenBallWasOnTheField = theFrameInfo.time;
  }
  else if(ballWasOnField)
  {
    ballDropInModel.outPosition = ballPositionOnField;
    ballDropInModel.lastTimeWhenBallOutWasObserved = theFrameInfo.time;
    if(!theGameState.isFreeKick() && !theGameState.isPenaltyKick())
    {
      if(ballPositionOnField.y() < theFieldDimensions.yPosLeftTouchline &&
         ballPositionOnField.y() > theFieldDimensions.yPosRightTouchline)
      {
        if(ballPositionOnField.x() > 0.f)
          ballDropInModel.dropInTypes = bit(BallDropInModel::ownCornerKick) | bit(BallDropInModel::opponentGoalKick);
        else
          ballDropInModel.dropInTypes = bit(BallDropInModel::ownGoalKick) | bit(BallDropInModel::opponentCornerKick);
      }
      else
        ballDropInModel.dropInTypes = bit(BallDropInModel::kickIn);
    }
    useOutPosition = true;
    ballWasOnField = false;
  }
}

void BallDropInLocator::updateGameControllerData(BallDropInModel& ballDropInModel)
{
  if(theGameState.isGoalKick() && theGameState.state != theExtendedGameState.stateLastFrame)
  {
    ballDropInModel.dropInTypes = bit(theGameState.isForOwnTeam() ? BallDropInModel::ownGoalKick : BallDropInModel::opponentGoalKick);
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
  else if(theGameState.isCornerKick() && theGameState.state != theExtendedGameState.stateLastFrame)
  {
    ballDropInModel.dropInTypes = bit(theGameState.isForOwnTeam() ? BallDropInModel::ownCornerKick : BallDropInModel::opponentCornerKick);
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
  else if(theGameState.isKickIn() && theGameState.state != theExtendedGameState.stateLastFrame)
  {
    ballDropInModel.dropInTypes = bit(BallDropInModel::kickIn);
    ballDropInModel.lastTimeWhenBallWentOut = theFrameInfo.time;
  }
}

void BallDropInLocator::draw() const
{
  DEBUG_DRAWING("module:BallDropInLocator:predictedOutPosition", "drawingOnField")
  {
    CROSS("module:BallDropInLocator:predictedOutPosition", predictedOutPosition.x(), predictedOutPosition.y(), 75, 30, Drawings::solidPen, ColorRGBA::black);
  }
}
