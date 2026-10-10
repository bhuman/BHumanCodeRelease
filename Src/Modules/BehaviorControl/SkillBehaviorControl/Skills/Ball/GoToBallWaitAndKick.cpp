/**
 * @file GoToBallWaitAndKick.cpp
 *
 * This file implements an implementation for the GoToBallWaitAndKick skill.
 *
 * @author Lasse Keller
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) GoToBallWaitAndKick,
       load((int) waitingTime,
            (Vector2f) defaultOffset,
            (float) distanceForNormalWalk,
            (float) ignoreObstaclesThreshold,
            (float) ignoreDynamicObstaclesThreshold,
            (float) kickDistanceThreshold,
            (Angle) kickAngleThreshold))
{
  const auto calculateWaitingPose = [&]() -> Pose2f
  {
    const Angle angleBallToGoal = (Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - theFieldBall.recentBallPositionOnField()).angle();
    // Use parameters from the kick info to find the preferred waiting position behind the ball.
    const Vector2f kickBallOffset = defaultOffset.rotated(angleBallToGoal);
    return Pose2f(angleBallToGoal - theRobotPose.rotation, theRobotPose.inverse() * (theFieldBall.recentBallPositionOnField() - kickBallOffset));
  };

  initial_state(notAtBall)
  {
    transition
    {
      const Pose2f targetPose = calculateWaitingPose();
      if(targetPose.translation.squaredNorm() < sqr(kickDistanceThreshold) &&
         std::abs(targetPose.rotation) < kickAngleThreshold &&
         theSkillRequest.passTarget == -1)
      {
        goto kick;
      }
      if(action_done)
        goto wait;
    }
    action
    {
      LookAtBall();
      const Pose2f targetPose = calculateWaitingPose();
      WalkToPoint({.target = calculateWaitingPose(),
                   .reduceWalkSpeedType = targetPose.translation.squaredNorm() > sqr(distanceForNormalWalk) ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow,
                   .rough = targetPose.translation.squaredNorm() <= sqr(ignoreObstaclesThreshold),
                   .disableObstacleAvoidance = targetPose.translation.squaredNorm() <= sqr(ignoreDynamicObstaclesThreshold),
                   .disableAvoidFieldBorder = true,
                   .targetOfInterest = theFieldBall.recentBallPositionRelative() });
    }
  }

  state(wait)
  {
    transition
    {
      if(state_time > waitingTime && theSkillRequest.passTarget != -1)
        goto pass;
      if(state_time > waitingTime && theSkillRequest.passTarget == -1)
        goto kick;
    }
    action
    {
      Stand();
      LookLeftAndRight();
    }
  }

  state(pass)
  {
    action
    {
      PassToTeammate({.playerNumber = theSkillRequest.passTarget,
                      .passedWaitingTime = waitingTime});
    }
  }

  state(kick)
  {
    action
    {
      KickAtGoal();
    }
  }
}
