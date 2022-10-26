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
  }),
});

class WalkToPointImpl : public WalkToPointImplBase
{
  option(WalkToPoint)
  {
    ASSERT(theWalkingEngineOutput.maxSpeed.translation.x() > 0.f);
    const float walkingSpeedRatio = !p.reduceWalkingSpeed ? p.speed : std::min(p.speed, Rangef::ZeroOneRange().limit(targetForwardWalkingSpeed / theWalkingEngineOutput.maxSpeed.translation.x()));
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

        if(targetDistance < 15.f && std::abs(p.target.rotation) < 3_deg)
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
                            .speed = {walkingSpeedRatio, walkingSpeedRatio, walkingSpeedRatio},
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
        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * p.target, Pose2f(walkingSpeedRatio, walkingSpeedRatio, walkingSpeedRatio));
        theWalkToPoseSkill({.target = p.target,
                            .speed = {walkingSpeedRatio, walkingSpeedRatio, walkingSpeedRatio},
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
        if(targetDistance > 100.f || std::abs(p.target.rotation) > 10_deg)
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
                            .speed = {walkingSpeedRatio, walkingSpeedRatio, walkingSpeedRatio},
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
        if(targetDistance > 100.f || std::abs(p.target.rotation) > 10_deg || p.disableStanding)
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
                               .speed = 0.f});
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
                               .speed = 0.f});
        theWalkAtRelativeSpeedSkill({.speed = {0.f, -walkingSpeedRatio, 0.f}});
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
                               .speed = walkingSpeedRatio});
        theWalkAtRelativeSpeedSkill({.speed = {0.f, 0.f, walkingSpeedRatio}});
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
                               .speed = walkingSpeedRatio});
        theWalkAtRelativeSpeedSkill({.speed = {0.f, 0.f, -walkingSpeedRatio}});
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToPointImpl);
