/**
 * @file WalkToPoint.cpp
 *
 * This file implements an implementation for the WalkToPoint skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/WalkSpeedConversion.h"
#include <cmath>

// TODO: Move the numbers from the code to meaningful parameters.
option((SkillBehaviorControl) WalkToPoint,
       args((const Pose2f&) target,
            (const Pose2f&) speed,
            (ReduceWalkSpeedType::ReduceWalkSpeedType) reduceWalkSpeedType,
            (bool) disableEnergySavingWalk,
            (bool) rough,
            (bool) disableObstacleAvoidance,
            (bool) disableAligning,
            (bool) disableStanding,
            (bool) disableAvoidFieldBorder,
            (const std::optional<Vector2f>&) targetOfInterest,
            (SideWalkingRequest::SideWalkingRequest) sideWalkingRequest),
       defs((float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
            (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
            (float)(100.f) translationStopThreshold, /**< Threshold for the distance to the target regarding both axis (x, y) to stop walking. */
            (float)(150.f) translationStartThreshold, /**< Threshold for the distance to the target regarding any axis (x, y) to start walking. */
            (float)(10_deg) rotationStopThreshold, /**< Threshold for the angle to the target pose to stop walking. */
            (float)(20_deg) rotationStartThreshold)) /**< Threshold for the angle to the target pose to start walking. */
{
  const auto startWalking = [&](const Pose2f& targetPoseRelative) -> bool
  {
    return std::abs(targetPoseRelative.rotation) > rotationStartThreshold ||
    std::abs(targetPoseRelative.translation.x()) > translationStartThreshold ||
    std::abs(targetPoseRelative.translation.y()) > translationStartThreshold;
  };

  const auto stopWalking = [&](const Pose2f& targetPoseRelative) -> bool
  {
    return std::abs(targetPoseRelative.rotation) < rotationStopThreshold &&
    std::abs(targetPoseRelative.translation.x()) < translationStopThreshold &&
    std::abs(targetPoseRelative.translation.y()) < translationStopThreshold;
  };

  ASSERT(theWalkingEngineOutput.maxSpeed.translation.x() > 0.f);

  const float targetDistance = target.translation.norm();
  disableObstacleAvoidance |= theGameState.isPenaltyShootout();

  const Pose2f walkingSpeedRatio = WalkSpeedConversion::convertSpeedRatio(reduceWalkSpeedType, speed, target, theFrameInfo, theGameState, theFieldBall, theWalkingEngineOutput);
  const bool energySavingWalk = !disableEnergySavingWalk && reduceWalkSpeedType == ReduceWalkSpeedType::slow;

  common_transition
  {
    if(!rough && (theFrameInfo.getTimeSince(theFootBumperState.status[Legs::left].lastContact) < 400 || theFrameInfo.getTimeSince(theFootBumperState.status[Legs::right].lastContact) < 400))
      goto footContactWalkBackwards;
    if(!rough && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::right].timeOfLastContact) < 800))
      goto armContactWalkLeft;
    if(!rough && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::left].timeOfLastContact) < 800))
      goto armContactWalkRight;
  }

  initial_state(walkClose)
  {
    transition
    {
      if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
        goto walkFar;

      if(stopWalking(target))
      {
        if(theMotionInfo.executedPhase == MotionPhase::stand && !disableStanding)
          goto destinationReachedStand;
        else
          goto destinationReached;
      }
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = walkingSpeedRatio });
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(target, rough, disableObstacleAvoidance, /* toBall: */ disableAvoidFieldBorder);
      WalkToPose({.target = target,
                  .speed = walkingSpeedRatio,
                  .obstacleAvoidance = obstacleAvoidance,
                  .keepTargetRotation = disableAligning,
                  .targetOfInterest = targetOfInterest,
                  .sideWalkingRequest = sideWalkingRequest,
                  .energySavingWalk = energySavingWalk});
    }
  }

  state(walkFar)
  {
    transition
    {
      if(targetDistance < switchToLibWalkDistance || disableObstacleAvoidance)
        goto walkClose;
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = walkingSpeedRatio});
      auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * target, walkingSpeedRatio);
      WalkToPose({.target = target,
                  .speed = walkingSpeedRatio,
                  .obstacleAvoidance = obstacleAvoidance,
                  .keepTargetRotation = disableAligning,
                  .targetOfInterest = targetOfInterest,
                  .sideWalkingRequest = sideWalkingRequest,
                  .energySavingWalk = energySavingWalk});
    }
  }

  target_state(destinationReached)
  {
    transition
    {
      if(startWalking(target))
      {
        if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
          goto walkFar;
        else
          goto walkClose;
      }
      if(state_time > 1000 && !disableStanding)
        goto destinationReachedStand;
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = walkingSpeedRatio});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(target, rough, disableObstacleAvoidance, /* toBall: */ disableAvoidFieldBorder);
      WalkToPose({.target = target,
                  .speed = walkingSpeedRatio,
                  .obstacleAvoidance = obstacleAvoidance,
                  .keepTargetRotation = disableAligning,
                  .targetOfInterest = targetOfInterest,
                  .sideWalkingRequest = sideWalkingRequest,
                  .energySavingWalk = energySavingWalk});
    }
  }

  target_state(destinationReachedStand)
  {
    transition
    {
      if(startWalking(target) || disableStanding)
      {
        if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
          goto walkFar;
        else
          goto walkClose;
      }
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = {0.f, 0.f, 0.f}});
      Stand({.energySavingWalk = energySavingWalk});
    }
  }

  state(footContactWalkBackwards)
  {
    transition
    {
      if(state_time > (rough ? 1000 : 3000))
      {
        if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
          goto walkFar;
        else
          goto walkClose;
      }
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = {0.f, 0.f, 0.f}});
      WalkAtRelativeSpeed({.speed = {0.f, -walkingSpeedRatio.translation.x(), 0.f},
                           .energySavingWalk = energySavingWalk});
    }
  }

  state(armContactWalkLeft)
  {
    transition
    {
      if(state_time > (rough ? 1000 : 3000))
      {
        if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
          goto walkFar;
        else
          goto walkClose;
      }
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = {0.f, 0.f, walkingSpeedRatio.translation.y()}});
      WalkAtRelativeSpeed({.speed = {0.f, 0.f, walkingSpeedRatio.translation.y()},
                           .energySavingWalk = energySavingWalk});
    }
  }

  state(armContactWalkRight)
  {
    transition
    {
      if(state_time > (rough ? 1000 : 3000))
      {
        if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
          goto walkFar;
        else
          goto walkClose;
      }
    }
    action
    {
      PublishMotion({.target = target.translation,
                     .speed = {0.f, 0.f, 0.f}});
      WalkAtRelativeSpeed({.speed = {0.f, 0.f, -walkingSpeedRatio.translation.y()},
                           .energySavingWalk = energySavingWalk});
    }
  }
}
