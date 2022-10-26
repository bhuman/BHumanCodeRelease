/**
 * @file BallInGoalTracker.cpp
 *
 * This file implements a module that checks if the ball is in the goal.
 *
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#include "BallInGoalTracker.h"

MAKE_MODULE(BallInGoalTracker, modeling);

void BallInGoalTracker::update(BallInGoal& theBallInGoal)
{
  // if the ball is rolling towards the goal
  if(goalShotNow())
    ballBuffer.push_front(true);
  else
    // write whether or not the ball is near enough to a goal
    ballBuffer.push_front(nearGoalNow());

  // returns true if more than 1 entry in the buffer is true
  const bool bufferedGuess = (ballBuffer.sum() - 1) > 0;

  // if the ball is considered as outside the goal save the time
  if(!bufferedGuess)
    lastTimeOutGoal = theFrameInfo.time;
  // if the ball was long enough considered as in goal save the current time
  else if(theFrameInfo.getTimeSince(lastTimeOutGoal) > changeStatusTriggerTime)
    stableLastTimeInGoal = theFrameInfo.time;

  theBallInGoal.timeSinceLastInGoal = theFrameInfo.time - stableLastTimeInGoal;

  // if the ball is in a goal look up if it is the own
  if(theBallInGoal.timeSinceLastInGoal == 0)
    theBallInGoal.inOwnGoal = theFieldBall.recentBallPositionOnField().x() < theFieldDimensions.xPosHalfWayLine;
}

bool BallInGoalTracker::nearGoalNow() const
{
  //the closer the robot is to the ball, the closer the ball must be to the goal
  float distanceToBall = theFieldBall.recentBallPositionRelative().norm();
  return distanceBallToGoal() < (exp2(-(distanceToBall - minDistance) * convergingRate) + 1) * maxDistanceToGoal;
}

bool BallInGoalTracker::goalShotNow() const
{
  return (theFieldBall.isRollingTowardsOpponentGoal
          && theFieldBall.recentBallEndPositionOnField().x() > theFieldDimensions.xPosOpponentGroundLine)
         || (theFieldBall.isRollingTowardsOwnGoal && theFieldBall.recentBallEndPositionOnField().x() < theFieldDimensions.xPosOwnGroundLine);
}

float BallInGoalTracker::distanceBallToGoal() const
{
  const float x = theFieldBall.recentBallPositionOnField().x() < theFieldDimensions.xPosHalfWayLine ? theFieldDimensions.xPosOwnGroundLine : theFieldDimensions.xPosOpponentGroundLine;
  //the goal is considered wider because the function is not continuous behind the ground line it jumps from negativ (in goal) to positiv (outside)
  if(theFieldBall.recentBallPositionOnField().y() < theFieldDimensions.yPosRightGoal - extraGoalWidth)
  {
    return Geometry::distance(Vector2f(x, theFieldDimensions.yPosRightGoal - extraGoalWidth), theFieldBall.recentBallPositionOnField());
  }
  else if(theFieldBall.recentBallPositionOnField().y() > theFieldDimensions.yPosLeftGoal + extraGoalWidth)
  {
    return Geometry::distance(Vector2f(x, theFieldDimensions.yPosLeftGoal + extraGoalWidth), theFieldBall.recentBallPositionOnField());
  }
  else
  {
    Geometry::Line line(Vector2f(x, theFieldDimensions.yPosRightGoal), Vector2f(0, theFieldDimensions.yPosLeftGoal - theFieldDimensions.yPosRightGoal));
    return (x < theFieldDimensions.xPosHalfWayLine ? 1 : -1) * Geometry::getDistanceToLineSigned(line, theFieldBall.recentBallPositionOnField());
  }
}
