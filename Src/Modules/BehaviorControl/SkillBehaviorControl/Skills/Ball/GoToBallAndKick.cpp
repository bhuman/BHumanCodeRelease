/**
 * @file GoToBallAndKick.cpp
 *
 * This file implements the GoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/WalkSpeedConversion.h"

option((SkillBehaviorControl) GoToBallAndKick,
       args((Angle) targetDirection,
            (KickInfo::KickType) kickType,
            (bool) lookActiveWithBall,
            (KickPrecision) alignPrecisely,
            (float) length,
            (PreStepType) preStepType,
            (bool) turnKickAllowed,
            (bool) shiftTurnKickPose,
            (const Pose2f&) speed,
            (ReduceWalkSpeedType::ReduceWalkSpeedType) reduceWalkSpeedType,
            (const Rangea&) directionPrecision),
       defs((float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
            (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
            (float)(600.f) goalkeeperDoNotIgnoreObstaclesDistance, /**< If the target is further away than this distance, the goalkeeper will not ignore obstacles. */
            (float)(500.f) goalkeeperIgnoreObstaclesDistance), /**< If the target is closer than this distance, the goalkeeper will ignore obstacles. */
       vars((Angle)(0_deg) startRotation)) /**< The value of \c theOdometryData.rotation when the \c turnAfterKick state is entered. */
{
  const Pose2f kickPose = Pose2f(targetDirection, theFieldInterceptBall.interceptedEndPositionRelative).rotate(theKickInfo[kickType].rotationOffset).translate(theKickInfo[kickType].ballOffset);

  ASSERT(reduceWalkSpeedType != ReduceWalkSpeedType::distanceBased);
  const Pose2f walkingSpeedRatio = WalkSpeedConversion::convertSpeedRatio(reduceWalkSpeedType, speed, Pose2f(1.f, 1.f, 1.f), theFrameInfo, theGameState, theFieldBall, theWalkingEngineOutput);

  initial_state(walkToBallFarRange)
  {
    transition
    {
      if(action_done)
      {
        if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
        {
          startRotation = theOdometryData.rotation;
          goto turnAfterKick;
        }
        else
          goto done;
      }
      else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
        goto walkToBallIgnoreObstacles;
      else if(kickPose.translation.norm() < switchToLibWalkDistance)
        goto walkToBallCloseRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * kickPose, speed);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = true,
                             .kickTargetRelative = theFieldInterceptBall.interceptedEndPositionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance,
                         .preStepType = preStepType,
                         .turnKickAllowed = turnKickAllowed,
                         .shiftTurnKickPose = shiftTurnKickPose,
                         .directionPrecision = directionPrecision});
    }
  }

  state(walkToBallCloseRange)
  {
    transition
    {
      if(action_done)
      {
        if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
        {
          startRotation = theOdometryData.rotation;
          goto turnAfterKick;
        }
        else
          goto done;
      }
      else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
        goto walkToBallIgnoreObstacles;
      else if(kickPose.translation.norm() > switchToPathPlannerDistance)
        goto walkToBallFarRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ false, /* toBall: */ true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = false,
                             .kickTargetRelative = theFieldInterceptBall.interceptedEndPositionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance,
                         .preStepType = preStepType,
                         .turnKickAllowed = turnKickAllowed,
                         .shiftTurnKickPose = shiftTurnKickPose,
                         .directionPrecision = directionPrecision});
    }
  }

  state(walkToBallIgnoreObstacles)
  {
    transition
    {
      if(action_done)
      {
        if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
        {
          startRotation = theOdometryData.rotation;
          goto turnAfterKick;
        }
        else
          goto done;
      }
      else if(kickPose.translation.norm() > switchToPathPlannerDistance)
        goto walkToBallFarRange;
      else if(kickPose.translation.norm() > goalkeeperDoNotIgnoreObstaclesDistance || !theGameState.isGoalkeeper())
        goto walkToBallCloseRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* toBall: */ true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = false,
                             .kickTargetRelative = theFieldInterceptBall.interceptedEndPositionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance,
                         .preStepType = preStepType,
                         .turnKickAllowed = turnKickAllowed,
                         .shiftTurnKickPose = shiftTurnKickPose,
                         .directionPrecision = directionPrecision});
    }
  }

  target_state(turnAfterKick)
  {
    transition
    {
      if(action_done || theFieldBall.ballWasSeen(state_time - 300))
      {
        if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
          goto walkToBallFarRange;
        else if(kickPose.translation.norm() < switchToLibWalkDistance)
          goto walkToBallCloseRange;
        else
          goto walkToBallFarRange;
      }
    }
    action
    {
      PublishMotion({.target = kickPose.translation,
                     .speed = {0.f, 0.f, 0.f}});
      LookAtAngles({.pan = Angle::normalize(theKickInfo[theMotionInfo.lastKickType].postRotationOffset - (theOdometryData.rotation - startRotation)),
                    .tilt = 23_deg});
      TurnAngle({.angle = theKickInfo[theMotionInfo.lastKickType].postRotationOffset});
    }
  }

  target_state(done)
  {
    transition
    {
      if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
        goto walkToBallFarRange;
      else if(kickPose.translation.norm() < switchToLibWalkDistance)
        goto walkToBallCloseRange;
      else
        goto walkToBallFarRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* toBall: */ true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = true,
                             .kickTargetRelative = theFieldInterceptBall.interceptedEndPositionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance,
                         .preStepType = preStepType,
                         .turnKickAllowed = turnKickAllowed,
                         .shiftTurnKickPose = shiftTurnKickPose,
                         .directionPrecision = directionPrecision});
    }
  }
}
