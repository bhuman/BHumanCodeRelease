/**
 * @file WalkToPoint.cpp
 *
 * This file implements an implementation for the WalkToPoint skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootBumperState.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(WalkToPointImpl,
{,
  IMPLEMENTS(WalkToPoint),
  REQUIRES(ArmContactModel),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(PathPlanner),
  REQUIRES(RobotPose),
  REQUIRES(WalkingEngineOutput),
  MODIFIES(BehaviorStatus),
  CALLS(Stand),
  CALLS(PublishMotion),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToPose),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
    (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
    (float)(175.f) targetForwardWalkingSpeed, /**< Reduce walking speed to reach this forward speed (in mm/s). */
    (float)(100.f) translationStopThreshold, /**< Threshold for the distance to the target regarding both axis (x, y) to stop walking. */
    (float)(150.f) translationStartThreshold, /**< Threshold for the distance to the target regarding any axis (x, y) to start walking. */
    (float)(10_deg) rotationStopThreshold, /**< Threshold for the angle to the target pose to stop walking. */
    (float)(20_deg) rotationStartThreshold, /**< Threshold for the angle to the target pose to start walking. */
  }),
});

class WalkToPointImpl : public WalkToPointImplBase
{
  // TODO: Move the numbers from the code to meaningful parameters.
  option(WalkToPoint)
  {
    ASSERT(theWalkingEngineOutput.maxSpeed.translation.x() > 0.f);

    Pose2f walkingSpeedRatio = p.speed;
    if(p.reduceWalkingSpeed)
    {
      const float minimalSpeed = std::min(p.speed.translation.x(), Rangef::ZeroOneRange().limit(targetForwardWalkingSpeed / theWalkingEngineOutput.maxSpeed.translation.x()));
      walkingSpeedRatio = Pose2f(std::min(p.speed.rotation, Angle(minimalSpeed)), minimalSpeed, std::min(p.speed.translation.y(), minimalSpeed));
    }
    const float targetDistance = p.target.translation.norm();

    const bool disableObstacleAvoidance = p.disableObstacleAvoidance || theGameState.isPenaltyShootout();

    common_transition
    {
      if(!p.rough && (theFrameInfo.getTimeSince(theFootBumperState.status[Legs::left].lastContact) < 400 || theFrameInfo.getTimeSince(theFootBumperState.status[Legs::right].lastContact) < 400))
        goto footContactWalkBackwards;
      if(!p.rough && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::right].timeOfLastContact) < 800))
        goto armContactWalkLeft;
      if(!p.rough && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::left].timeOfLastContact) < 800))
        goto armContactWalkRight;
    }

    initial_state(walkClose)
    {
      transition
      {
        if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
          goto walkFar;

        if(stopWalking(p.target))
        {
          if(theMotionInfo.executedPhase == MotionPhase::stand && !p.disableStanding)
            goto destinationReachedStand;
          else
            goto destinationReached;
        }
      }
      action
      {
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(p.target, p.rough, disableObstacleAvoidance, /* toBall: */ p.disableAvoidFieldBorder);
        theWalkToPoseSkill({.target = p.target,
                            .speed = walkingSpeedRatio,
                            .obstacleAvoidance = obstacleAvoidance,
                            .keepTargetRotation = p.disableAligning,
                            .targetOfInterest = p.targetOfInterest,
                            .forceSideWalking = p.forceSideWalking });
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = walkingSpeedRatio });
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
        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * p.target, walkingSpeedRatio);
        theWalkToPoseSkill({.target = p.target,
                            .speed = walkingSpeedRatio,
                            .obstacleAvoidance = obstacleAvoidance,
                            .keepTargetRotation = p.disableAligning,
                            .targetOfInterest = p.targetOfInterest,
                            .forceSideWalking = p.forceSideWalking });
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = walkingSpeedRatio });
      }
    }

    target_state(destinationReached)
    {
      transition
      {
        if(startWalking(p.target))
        {
          if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
            goto walkFar;
          else
            goto walkClose;
        }
        if(state_time > 1000 && !p.disableStanding)
          goto destinationReachedStand;
      }
      action
      {
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(p.target, p.rough, disableObstacleAvoidance, /* toBall: */ p.disableAvoidFieldBorder);
        theWalkToPoseSkill({.target = p.target,
                            .speed = walkingSpeedRatio,
                            .obstacleAvoidance = obstacleAvoidance,
                            .keepTargetRotation = p.disableAligning,
                            .targetOfInterest = p.targetOfInterest,
                            .forceSideWalking = p.forceSideWalking });
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = walkingSpeedRatio });
      }
    }

    target_state(destinationReachedStand)
    {
      transition
      {
        if(startWalking(p.target) || p.disableStanding)
        {
          if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
            goto walkFar;
          else
            goto walkClose;
        }
      }
      action
      {
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = {0.f, 0.f, 0.f}});
        theStandSkill();
      }
    }

    state(footContactWalkBackwards)
    {
      transition
      {
        if(state_time > (p.rough ? 1000 : 3000))
        {
          if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
            goto walkFar;
          else
            goto walkClose;
        }
      }
      action
      {
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = {0.f, 0.f, 0.f}});
        theWalkAtRelativeSpeedSkill({.speed = {0.f, -walkingSpeedRatio.translation.x(), 0.f}});
      }
    }
    state(armContactWalkLeft)
    {
      transition
      {
        if(state_time > (p.rough ? 1000 : 3000))
        {
          if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
            goto walkFar;
          else
            goto walkClose;
        }
      }
      action
      {
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = {0.f, 0.f, walkingSpeedRatio.translation.y()}});
        theWalkAtRelativeSpeedSkill({.speed = {0.f, 0.f, walkingSpeedRatio.translation.y()}});
      }
    }
    state(armContactWalkRight)
    {
      transition
      {
        if(state_time > (p.rough ? 1000 : 3000))
        {
          if(targetDistance > switchToPathPlannerDistance && !disableObstacleAvoidance)
            goto walkFar;
          else
            goto walkClose;
        }
      }
      action
      {
        thePublishMotionSkill({.target = p.target.translation,
                               .speed = {0.f, 0.f, 0.f}});
        theWalkAtRelativeSpeedSkill({.speed = {0.f, 0.f, -walkingSpeedRatio.translation.y()}});
      }
    }
  }

  bool startWalking(const Pose2f& targetPoseRelative)
  {
    return std::abs(targetPoseRelative.rotation) > rotationStartThreshold ||
           std::abs(targetPoseRelative.translation.x()) > translationStartThreshold ||
           std::abs(targetPoseRelative.translation.y()) > translationStartThreshold;
  }

  bool stopWalking(const Pose2f& targetPoseRelative)
  {
    return std::abs(targetPoseRelative.rotation) < rotationStopThreshold &&
           std::abs(targetPoseRelative.translation.x()) < translationStopThreshold &&
           std::abs(targetPoseRelative.translation.y()) < translationStopThreshold;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToPointImpl);
