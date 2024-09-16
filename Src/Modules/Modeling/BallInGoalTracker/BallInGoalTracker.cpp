/**
 * @file BallInGoalTracker.cpp
 *
 * This file implements a module that checks if the ball is in the goal.
 *
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#include "BallInGoalTracker.h"
#include <cmath>

MAKE_MODULE(BallInGoalTracker);

void BallInGoalTracker::update(BallInGoal& theBallInGoal)
{
  // write if the ball is rolling towards the goal or sufficiently near a goal
  const bool goalShot = goalShotNow();
  const bool nearGoal = nearGoalNow();
  ballBuffer.push_front(goalShot || nearGoal);

  // returns true if more than 1 entry in the buffer is true
  const bool bufferedGuess = ballBuffer.sum() > 1;

  // if the ball is considered as outside the goal save the time
  if(!bufferedGuess)
    lastTimeOutGoal = theFrameInfo.time;
  // if the ball was long enough considered as in goal save the current time
  else if(theFrameInfo.getTimeSince(lastTimeOutGoal) > changeStatusTriggerTime)
  {
    theBallInGoal.lastTimeInGoal = theFrameInfo.time;
    // if the ball is in a goal look up if it is the own
    theBallInGoal.inOwnGoal = theFieldBall.recentBallPositionOnField().x() < theFieldDimensions.xPosHalfwayLine;
  }
}

bool BallInGoalTracker::nearGoalNow() const
{
  //the closer the robot is to the ball, the closer the ball must be to the goal
  const float distanceToBall = theFieldBall.recentBallPositionRelative().norm();
  const float ballDistanceToGoal = distanceBallToGoal();
  const float distanceThreshold = (1.f - std::exp2(-(distanceToBall - minDistance) * convergingRate)) * maxDistanceToGoal;
  return ballDistanceToGoal < distanceThreshold;
}

bool BallInGoalTracker::goalShotNow() const
{
  return (theFieldBall.isRollingTowardsOpponentGoal
          && theFieldBall.recentBallEndPositionOnField().x() > theFieldDimensions.xPosOpponentGoalLine)
         || (theFieldBall.isRollingTowardsOwnGoal && theFieldBall.recentBallEndPositionOnField().x() < theFieldDimensions.xPosOwnGoalLine);
}

float BallInGoalTracker::distanceBallToGoal() const
{
  const Vector2f& ballPosition = theFieldBall.recentBallPositionOnField();
  const bool ballInOwnHalf = ballPosition.x() < theFieldDimensions.xPosHalfwayLine;
  const float goalLineX = ballInOwnHalf ? theFieldDimensions.xPosOwnGoalLine : theFieldDimensions.xPosOpponentGoalLine;
  //the goal is considered wider because the function is not continuous behind the goal line it jumps from negative (in goal) to positive (outside)
  if(ballPosition.y() < theFieldDimensions.yPosRightGoal - extraGoalWidth)
  {
    return (ballPosition - Vector2f(goalLineX, theFieldDimensions.yPosRightGoal - extraGoalWidth)).norm();
  }
  else if(ballPosition.y() > theFieldDimensions.yPosLeftGoal + extraGoalWidth)
  {
    return (ballPosition - Vector2f(goalLineX, theFieldDimensions.yPosLeftGoal + extraGoalWidth)).norm();
  }
  else
  {
    return (ballInOwnHalf ? 1.f : -1.f) * (ballPosition.x() - goalLineX);
  }
}
