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
  //if the ball is in the goal we save it to the buffer
  if(checkInGoalNow())
    ballBuffer.push_front(true);
  //if the ball is going to land in the goal and is already near also save it as in goal
  else if(goalShotNow())
    ballBuffer.push_front(nearGoalNow());
  //write false to the Buffer if none of the previous cases match
  else
    ballBuffer.push_front(false);

  //returns true if more than 1 entry in the buffer is true
  const bool bufferedGuess = (ballBuffer.sum() - 1) > 0;

  //if the ball is considered as outside of the goal save the time
  if(!bufferedGuess)
    lastTimeOutGoal = theFrameInfo.time;
  //if the ball was long enough considered as in goal save the current time
  else if(theFrameInfo.getTimeSince(lastTimeOutGoal) > changeStatusTriggerTime)
    stableLastTimeInGoal = theFrameInfo.time;

  theBallInGoal.timeSinceLastInGoal = theFrameInfo.time - stableLastTimeInGoal;

  //if the ball is in a goal look up if it is the own
  if(theBallInGoal.timeSinceLastInGoal == 0)
    theBallInGoal.inOwnGoal = theFieldBall.teamPositionOnField.x() < theFieldDimensions.xPosOwnGoalArea;
}

bool BallInGoalTracker::checkInGoalNow() const
{
  return (theFieldBall.teamPositionOnField.x() > theFieldDimensions.xPosOpponentGroundLine
    || theFieldBall.teamPositionOnField.x() < theFieldDimensions.xPosOwnGroundLine)
    && theFieldBall.teamPositionOnField.y() < theFieldDimensions.yPosLeftGoal
    && theFieldBall.teamPositionOnField.y() > theFieldDimensions.yPosRightGoal;
}

bool BallInGoalTracker::goalShotNow() const
{
  return (theFieldBall.isRollingTowardsOpponentGoal
    && theFieldBall.teamEndPositionOnField.x() > theFieldDimensions.xPosOpponentGroundLine)
    || (theFieldBall.isRollingTowardsOwnGoal
    && theFieldBall.teamEndPositionOnField.x() < theFieldDimensions.xPosOwnGroundLine);
}

bool BallInGoalTracker::nearGoalNow() const
{
  return theFieldBall.teamPositionOnField.x() > theFieldDimensions.xPosOpponentGroundLine
    || theFieldBall.teamPositionOnField.x() < theFieldDimensions.xPosOwnGroundLine
    || (theFieldBall.teamPositionOnField.y() < theFieldDimensions.yPosLeftGoalArea
      && theFieldBall.teamPositionOnField.y() > theFieldDimensions.yPosRightGoalArea
      && (theFieldBall.teamPositionOnField.x() > theFieldDimensions.xPosOpponentGoalArea
        || theFieldBall.teamPositionOnField.x() < theFieldDimensions.xPosOwnGoalArea));
}
