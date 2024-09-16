/**
 * @file GoToBallAndKick.cpp
 *
 * This file implements the GoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

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
            (ReduceWalkSpeedType) reduceWalkSpeedType,
            (const Rangea&) directionPrecision),
       defs((float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
            (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
            (float)(600.f) goalkeeperDoNotIgnoreObstaclesDistance, /**< If the target is further away than this distance, the goalkeeper will not ignore obstacles. */
            (float)(500.f) goalkeeperIgnoreObstaclesDistance, /**< If the target is closer than this distance, the goalkeeper will ignore obstacles. */
            (float)(175.f) targetForwardWalkingSpeedSlow, /**< Reduce walking speed to reach this forward speed (in mm/s). */
            (float)(250.f) targetForwardWalkingSpeedNormal), /**< Normal walking speed forward (in mm/s). */
       vars((Angle)(0_deg) startRotation)) /**< The value of \c theOdometryData.rotation when the \c turnAfterKick state is entered. */
{
  const Pose2f kickPose = Pose2f(targetDirection, theFieldInterceptBall.interceptedEndPositionRelative).rotate(theKickInfo[kickType].rotationOffset).translate(theKickInfo[kickType].ballOffset);

  Pose2f walkingSpeedRatio = speed;
  switch(reduceWalkSpeedType)
  {
    case ReduceWalkSpeedType::slow:
    {
      const float minimalSpeed = std::min(speed.translation.x(), Rangef::ZeroOneRange().limit(targetForwardWalkingSpeedSlow / theWalkingEngineOutput.maxSpeed.translation.x()));
      walkingSpeedRatio = Pose2f(std::min(speed.rotation, Angle(minimalSpeed)), minimalSpeed, std::min(speed.translation.y(), minimalSpeed));
      break;
    }
    case ReduceWalkSpeedType::normal:
    {
      const float minimalSpeed = std::min(speed.translation.x(), Rangef::ZeroOneRange().limit(targetForwardWalkingSpeedNormal / theWalkingEngineOutput.maxSpeed.translation.x()));
      walkingSpeedRatio = Pose2f(std::min(speed.rotation, Angle(minimalSpeed)), minimalSpeed, std::min(speed.translation.y(), minimalSpeed));
      break;
    }
    case ReduceWalkSpeedType::noChange:
    default:
      break;
  }

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
