/**
 * @file WalkToPoint.cpp
 *
 * This file implements an implementation for the WalkToPoint skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */

#include "SkillBehaviorControl.h"
#include <cmath>

// TODO: Move the numbers from the code to meaningful parameters.
option((SkillBehaviorControl) WalkToPoint,
       args((const Pose2f&) target,
            (const Pose2f&) speed,
            (ReduceWalkSpeedType) reduceWalkingSpeed,
            (bool) rough,
            (bool) disableObstacleAvoidance,
            (bool) disableAligning,
            (bool) disableStanding,
            (bool) disableAvoidFieldBorder,
            (const std::optional<Vector2f>&) targetOfInterest,
            (bool) forceSideWalking),
       defs((float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
            (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
            (float)(175.f) targetForwardWalkingSpeedSlow, /**< Reduce walking speed to reach this forward speed (in mm/s). */
            (float)(250.f) targetForwardWalkingSpeedNormal, /**< Normal walking speed forward (in mm/s). */
            (float)(100.f) translationStopThreshold, /**< Threshold for the distance to the target regarding both axis (x, y) to stop walking. */
            (float)(150.f) translationStartThreshold, /**< Threshold for the distance to the target regarding any axis (x, y) to start walking. */
            (float)(10_deg) rotationStopThreshold, /**< Threshold for the angle to the target pose to stop walking. */
            (float)(20_deg) rotationStartThreshold, /**< Threshold for the angle to the target pose to start walking. */
            (Rangef)({500.f, 1000.f}) ballDistanceSpeedReductionRange)) /**< Reduce walking speed based on distance to ball. */
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

  Pose2f walkingSpeedRatio = speed;
  switch(reduceWalkingSpeed)
  {
    case ReduceWalkSpeedType::slow:
    {
      const float ballDistance = theFieldBall.recentBallEndPositionRelative().norm();
      const float maxWalkSpeed = mapToRange(ballDistance, ballDistanceSpeedReductionRange.min, ballDistanceSpeedReductionRange.max, 1.f, targetForwardWalkingSpeedSlow / theWalkingEngineOutput.maxSpeed.translation.x());
      const float minimalSpeed = std::min(speed.translation.x(), Rangef::ZeroOneRange().limit(maxWalkSpeed));
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

  const float targetDistance = target.translation.norm();

  disableObstacleAvoidance |= theGameState.isPenaltyShootout();

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
                  .forceSideWalking = forceSideWalking });
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
                  .forceSideWalking = forceSideWalking});
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
                  .forceSideWalking = forceSideWalking});
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
      Stand();
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
      WalkAtRelativeSpeed({.speed = {0.f, -walkingSpeedRatio.translation.x(), 0.f}});
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
      WalkAtRelativeSpeed({.speed = {0.f, 0.f, walkingSpeedRatio.translation.y()}});
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
      WalkAtRelativeSpeed({.speed = {0.f, 0.f, -walkingSpeedRatio.translation.y()}});
    }
  }
}
