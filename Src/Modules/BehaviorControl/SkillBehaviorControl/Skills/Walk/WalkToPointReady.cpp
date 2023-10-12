/**
 * @file WalkToPointReady.cpp
 *
 * This file implements an implementation for the WalkToPointReady skill.
 *
 * @author Arne Hasselbring
 * @author Fynn Böse
 */

#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"

SKILL_IMPLEMENTATION(WalkToPointReadyImpl,
{,
  IMPLEMENTS(WalkToPointReady),
  CALLS(LookActive),
  CALLS(WalkToPoint),
  CALLS(WalkToPointObstacle),
  CALLS(WalkPotentialField),
  CALLS(Stand),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(IllegalAreas),
  REQUIRES(RobotPose),
  REQUIRES(StrategyStatus),
  DEFINES_PARAMETERS(
  {,
    (int)(5000) timeToEmergencyMode, /**< If only this time is left in the ready state, the robot will just turn to its final rotation. */
  }),
});

class WalkToPointReadyImpl : public WalkToPointReadyImplBase
{
  option(WalkToPointReady)
  {
    const Pose2f targetRelative = theRobotPose.inverse() * p.target;
    const int timeUntilSetStarts = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);
    const bool opponentPenaltyKickGoalkeeper = theGameState.state == GameState::setupOpponentPenaltyKick && theGameState.isGoalkeeper();
    const float durationUntilAnticipatedIllegal = (theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) * 0.5f;

    initial_state(walkToPointReady)
    {
      transition
      {
        if (theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal)) {
          goto leaveIllegalAreas;
        }
        if(targetRelative.translation.squaredNorm() < sqr(200.f))
        {
          if(theRobotPose.translation.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius) || opponentPenaltyKickGoalkeeper)
            goto adjustToFinalPosePrecisely;
          else
            goto adjustToFinalPose;
        }
        else if(timeUntilSetStarts < timeToEmergencyMode &&
                !theIllegalAreas.willPositionBeIllegal(theRobotPose.translation, 175.f))
          goto emergencyAdjustment;
      }
      action
      {
        theLookActiveSkill({.ignoreBall = true});
        if(theGameState.isKickOff())
          theWalkToPointSkill({.target = targetRelative,
                               .speed = theGameState.kickOffSetupFromSidelines ? Pose2f(0.7f, 0.7f, 0.7f) : Pose2f(1.f, 1.f, 0.7f),
                               .reduceWalkingSpeed = false,
                               .rough = targetRelative.translation.norm() < 500.f,
                               .disableStanding = true,
                               .targetOfInterest = p.targetOfInterest});
        else
          theWalkToPointObstacleSkill({.target = targetRelative,
                                       .speed = theGameState.kickOffSetupFromSidelines ? Pose2f(0.7f, 0.7f, 0.7f) : Pose2f(1.f, 1.f, 1.f),
                                       .reduceWalkingSpeed = false,
                                       .rough = targetRelative.translation.norm() < 500.f,
                                       .disableStanding = true,
                                       .targetOfInterest = p.targetOfInterest});
      }
    }

    state(adjustToFinalPose)
    {
      transition
      {
        if(state_time > 1000 && targetRelative.translation.squaredNorm() < sqr(40.f) && std::abs(targetRelative.rotation) < 5_deg)
          goto stand;
        else if(timeUntilSetStarts < timeToEmergencyMode &&
                !theIllegalAreas.isPositionIllegal(theRobotPose.translation, 175.f) &&
                targetRelative.translation.squaredNorm() > sqr(300.f) && std::abs(targetRelative.rotation) > 20_deg)
          goto emergencyAdjustment;
      }
      action
      {
        theLookActiveSkill({.ignoreBall = true});
        if(theGameState.isKickOff())
          theWalkToPointSkill({.target = targetRelative,
                               .speed = {0.7f, 0.7f, 0.7f},
                               .reduceWalkingSpeed = false,
                               .rough = true,
                               .disableObstacleAvoidance = theStrategyStatus.role == PositionRole::toRole(PositionRole::goalkeeper),
                               .disableStanding = true,
                               .targetOfInterest = p.targetOfInterest});
        else
          theWalkToPointObstacleSkill({.target = targetRelative,
                                       .speed = {0.7f, 0.7f, 0.7f},
                                       .reduceWalkingSpeed = false,
                                       .rough = true,
                                       .disableObstacleAvoidance = theStrategyStatus.role == PositionRole::toRole(PositionRole::goalkeeper),
                                       .disableStanding = true,
                                       .targetOfInterest = p.targetOfInterest });
      }
    }

    state(adjustToFinalPosePrecisely)
    {
      transition
      {
        if(state_time > 1000 && targetRelative.translation.squaredNorm() < sqr(25.f) && std::abs(targetRelative.rotation) < 3_deg)
          goto stand;
        else if(timeUntilSetStarts < timeToEmergencyMode &&
                !theIllegalAreas.isPositionIllegal(theRobotPose.translation, 175.f) &&
                targetRelative.translation.squaredNorm() > sqr(300.f) && std::abs(targetRelative.rotation) > 20_deg)
          goto emergencyAdjustment;
      }
      action
      {
        theLookActiveSkill({.ignoreBall = true});
        if(theGameState.isKickOff())
          theWalkToPointSkill({.target = targetRelative,
                               .speed = {0.7f, 0.7f, 0.7f},
                               .reduceWalkingSpeed = false,
                               .rough = true,
                               .disableStanding = true,
                               .targetOfInterest = p.targetOfInterest});
        else
          theWalkToPointObstacleSkill({.target = targetRelative,
                                       .speed = {0.7f, 0.7f, 0.7f},
                                       .reduceWalkingSpeed = false,
                                       .rough = true,
                                       .disableStanding = true,
                                       .targetOfInterest = p.targetOfInterest });
      }
    }

    state(stand)
    {
      transition
      {
        if(targetRelative.translation.squaredNorm() > sqr(opponentPenaltyKickGoalkeeper ? 50.f : 300.f) || std::abs(targetRelative.rotation) > (opponentPenaltyKickGoalkeeper ? 5_deg : 15_deg) ||
           theIllegalAreas.isPositionIllegal(theRobotPose.translation, 145.f))
          goto walkToPointReady;
        if(theStandSkill.isDone() && state_time > 1000)
          goto standHigh;
      }
      action
      {
        theLookActiveSkill({.ignoreBall = true});
        theStandSkill();
      }
    }

    state(standHigh)
    {
      transition
      {
        if(targetRelative.translation.squaredNorm() > sqr(opponentPenaltyKickGoalkeeper ? 50.f : 300.f) || std::abs(targetRelative.rotation) > (opponentPenaltyKickGoalkeeper ? 5_deg : 15_deg) ||
           theIllegalAreas.isPositionIllegal(theRobotPose.translation, 145.f))
          goto walkToPointReady;
      }
      action
      {
        theLookActiveSkill({.ignoreBall = true});
        theStandSkill({.high = true});
      }
    }

    state(emergencyAdjustment)
    {
      action
      {
        theLookActiveSkill({.ignoreBall = true});
        theWalkToPointObstacleSkill({.target = {targetRelative.rotation},
                                     .reduceWalkingSpeed = false,
                                     .rough = true,
                                     .disableObstacleAvoidance = true,
                                     .disableAligning = true,
                                     .targetOfInterest = p.targetOfInterest });
      }
    }

    state(leaveIllegalAreas)
    {
      transition
      {
        if (!theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal))
          goto walkToPointReady;
      }
      action
      {
        theWalkPotentialFieldSkill({.target = theRobotPose.translation,
                                      .playerNumber = -1,
                                      .straight = true });
        theLookActiveSkill({.withBall = false });
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToPointReadyImpl);
