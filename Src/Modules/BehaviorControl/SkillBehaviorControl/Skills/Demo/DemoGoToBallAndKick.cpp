/**
 * @file DemoGoToBallAndKick.cpp
 *
 * This file implements the DemoGoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoGoToBallAndKick,
       defs((Vector2f)({-200.f, 0.f}) waitPosition, /**< We can not trust the detected ball, therefor wait behind the known ball location. */
            (int)(1500) ballSeenDuration, /**< The ball was seen in this time frame, therefore we can trust the detection. */
            (int)(1000) lastKickDuration, /**< Ignore ball detections when we recently kicked. */
            (float)(20.f) positionHysteresis, /**< hysteresis to avoid foot change when y is 0.0 */
            (float)(350.f) ballDistanceClose, /**< Distance to consider the ball being close. */
            (int)(1500) timeForBallBeingCloseToKick), /**< Ball must be close for this time, to start kick. */
       vars((unsigned)(theFrameInfo.time) lastTimeBallWasClose, /**< Timestamp ball was last time close. */
            (bool)(false) kickLeft)) /**< Which foot to use for a kick. */
{
  if(theBallModel.estimate.position.squaredNorm() > sqr(ballDistanceClose))
    lastTimeBallWasClose = theFrameInfo.time;

  const bool shouldKick = (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < ballSeenDuration  // We recently saw the ball
                           || theFrameInfo.getTimeSince(theMotionInfo.lastKickTimestamp) > lastKickDuration) // The last kick was long ago
                          && theBallModel.timeWhenLastSeen > theMotionInfo.lastKickTimestamp + lastKickDuration // The detected ball timestamp must be AFTER the
                          && theFrameInfo.getTimeSince(lastTimeBallWasClose) > timeForBallBeingCloseToKick; // Walk close to the ball and stop before kicking

  kickLeft = theFieldBall.positionRelative.y() > (kickLeft ? -positionHysteresis : positionHysteresis);

  initial_state(goToLastKnownPosition)
  {
    transition
    {
      if(shouldKick && state_time > 2000)
      {
        if(kickLeft)
          goto kickBallLeft;
        else
          goto kickBallRight;
      }
    }
    action
    {
      LookAtBall();
      const float yOffset = kickLeft ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset;
      WalkToPoint({.target = {theBallModel.estimate.position + waitPosition + Vector2f(0.f, yOffset)},
                  .reduceWalkingSpeed = ReduceWalkSpeedType::slow,
                  .rough = true,
                  .disableObstacleAvoidance = true,
                  .disableStanding = true});
    }
  }

  state(kickBallLeft)
  {
    transition
    {
      if(!shouldKick)
        goto goToLastKnownPosition;
      else if(!kickLeft)
        goto kickBallRight;
    }
    action
    {
      LookAtBall();
      WalkToBallAndKick({.targetDirection = -theKickInfo[KickInfo::walkForwardsLeftAlternative].rotationOffset,
                         .kickType = KickInfo::walkForwardsLeftAlternative});
    }
  }

  state(kickBallRight)
  {
    transition
    {
      if(!shouldKick)
        goto goToLastKnownPosition;
      else if(kickLeft)
        goto kickBallLeft;
    }
    action
    {
      LookAtBall();
      WalkToBallAndKick({.targetDirection = -theKickInfo[KickInfo::walkForwardsRightAlternative].rotationOffset,
                         .kickType = KickInfo::walkForwardsRightAlternative});
    }
  }
}
