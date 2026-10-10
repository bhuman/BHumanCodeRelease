/**
 * @file DemoGoToBallAndKick.cpp
 *
 * This file implements the DemoGoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoGoToBallAndKick,
       defs((Vector2f)({-200.f, 0.f}) waitPosition, /**< We can not trust the detected ball, therefore wait behind the known ball location. */
            (int)(1500) ballSeenDuration, /**< The ball was seen in this time frame, therefore we can trust the detection. */
            (float)(20.f) positionHysteresis, /**< hysteresis to avoid foot change when y is 0.0 */
            (float)(350.f) ballDistanceClose), /**< Distance to consider the ball being close. */
       vars((unsigned)(theFrameInfo.time) lastTimeBallWasClose, /**< Timestamp ball was last time close. */
            (bool)(false) kickLeft, /**< Which foot to use for a kick. */
            (Angle)(theOdometryData.rotation) startRotation)) /** Ref kick angle. */
{
  if(theBallModel.estimate.position.squaredNorm() > sqr(ballDistanceClose))
    lastTimeBallWasClose = theFrameInfo.time;

  const bool shouldKick = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < ballSeenDuration;

  kickLeft = theFieldBall.positionRelative.y() > (kickLeft ? -positionHysteresis : positionHysteresis);

  initial_state(goToLastKnownPosition)
  {
    transition
    {
      if(shouldKick)
      {
        goto kickBall;
      }
    }
    action
    {
      LookAtBall();
      const float yOffset = kickLeft ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset;
      WalkToPoint({.target = {theBallModel.estimate.position + waitPosition + Vector2f(0.f, yOffset)},
                   .reduceWalkSpeedType = ReduceWalkSpeedType::slow,
                   .rough = true,
                   .disableObstacleAvoidance = true,
                   .disableStanding = true});
    }
  }

  state(kickBall)
  {
    transition
    {
      if(!shouldKick)
        goto goToLastKnownPosition;
    }
    action
    {
      LookAtBall();
      WalkToBallAndKick({.targetDirection = Angle::normalize(startRotation - theOdometryData.rotation - theKickInfo[KickInfo::rlKickStrong].rotationOffset),
                         .kickType = KickInfo::rlKickDynamic });
    }
  }
}
