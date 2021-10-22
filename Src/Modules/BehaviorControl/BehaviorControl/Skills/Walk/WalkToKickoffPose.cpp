/**
 * @file WalkToKickoffPose.cpp
 *
 * This file implements an implementation for the WalkToKickoffPose skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(WalkToKickoffPoseImpl,
{,
  IMPLEMENTS(WalkToKickoffPose),
  CALLS(LookActive),
  CALLS(WalkToPoint),
  CALLS(Stand),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(RobotPose),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(TeamBehaviorStatus),
  DEFINES_PARAMETERS(
  {,
    (int)(5000) timeToEmergencyMode, /**< If only this time is left in the ready state, the robot will just turn to its final rotation. */
  }),
});

class WalkToKickoffPoseImpl : public WalkToKickoffPoseImplBase
{
  option(WalkToKickoffPose)
  {
    const Pose2f targetRelative = theRobotPose.inversePose * p.target;
    // secondaryTime is used because it can happen that the robot receives its first GC packet when the ready state is already running for some time.
    // On the other hand, the secondaryTime must not be used when the raw game state is not yet ready (during the first 15s after a goal).
    const int timeUntilSetState = std::min<int>(theRawGameInfo.state == STATE_READY ? theGameInfo.secondaryTime * 1000 : 45000, 45000 - theExtendedGameInfo.timeSinceReadyStarted);

    initial_state(walkToKickoffPose)
    {
      transition
      {
        if(targetRelative.translation.squaredNorm() < sqr(200.f))
        {
          if(theRobotPose.translation.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius))
            goto adjustToFinalPosePrecisely;
          else
            goto adjustToFinalPose;
        }
        else if(timeUntilSetState < timeToEmergencyMode && theRobotPose.translation.x() < -200.f &&
                (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber || theRobotPose.translation.squaredNorm() > sqr(theFieldDimensions.centerCircleRadius + 200.f)))
          goto emergencyAdjustment;
      }
      action
      {
        theLookActiveSkill(false, /* ignoreBall: */ true);
        theWalkToPointSkill(targetRelative, theExtendedGameInfo.walkingInFromSidelines ? 0.7f : 1.f, /* rough: */ targetRelative.translation.norm() < 500.f, false, false, true);
      }
    }

    state(adjustToFinalPose)
    {
      transition
      {
        if(state_time > 1000 && targetRelative.translation.squaredNorm() < sqr(40.f) && std::abs(targetRelative.rotation) < 5_deg)
          goto stand;
        else if(timeUntilSetState < timeToEmergencyMode && theRobotPose.translation.x() < -200.f &&
                (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber || theRobotPose.translation.squaredNorm() > sqr(theFieldDimensions.centerCircleRadius + 200.f)) &&
                targetRelative.translation.squaredNorm() > sqr(300.f) && std::abs(targetRelative.rotation) > 20_deg)
          goto emergencyAdjustment;
      }
      action
      {
        theLookActiveSkill(false, /* ignoreBall: */ true);
        theWalkToPointSkill(targetRelative, 0.7f, /* rough: */ true, /* disableObstacleAvoidance: */ theTeamBehaviorStatus.role.isGoalkeeper(), false, true);
      }
    }

    state(adjustToFinalPosePrecisely)
    {
      transition
      {
        if(state_time > 1000 && targetRelative.translation.squaredNorm() < sqr(25.f) && std::abs(targetRelative.rotation) < 3_deg)
          goto stand;
        else if(timeUntilSetState < timeToEmergencyMode && theRobotPose.translation.x() < -200.f &&
                (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber || theRobotPose.translation.squaredNorm() > sqr(theFieldDimensions.centerCircleRadius + 200.f)) &&
                targetRelative.translation.squaredNorm() > sqr(300.f) && std::abs(targetRelative.rotation) > 20_deg)
          goto emergencyAdjustment;
      }
      action
      {
        theLookActiveSkill(false, /* ignoreBall: */ true);
        theWalkToPointSkill(targetRelative, 0.7f, /* rough: */ true, false, false, true);
      }
    }

    state(stand)
    {
      transition
      {
        if(targetRelative.translation.squaredNorm() > sqr(300.f) || std::abs(targetRelative.rotation) > 15_deg ||
           theRobotPose.translation.x() > -170.f ||
           (theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber && theRobotPose.translation.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius + 170.f)))
          goto walkToKickoffPose;
        if(theStandSkill.isDone() && state_time > 1000)
          goto standHigh;
      }
      action
      {
        theLookActiveSkill(false, /* ignoreBall: */ true);
        theStandSkill();
      }
    }

    state(standHigh)
    {
      transition
      {
        if(targetRelative.translation.squaredNorm() > sqr(300.f) || std::abs(targetRelative.rotation) > 15_deg ||
           theRobotPose.translation.x() > -170.f ||
           (theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber && theRobotPose.translation.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius + 170.f)))
          goto walkToKickoffPose;
      }
      action
      {
        theLookActiveSkill(false, /* ignoreBall: */ true);
        theStandSkill(/* high: */ true);
      }
    }

    state(emergencyAdjustment)
    {
      action
      {
        theLookActiveSkill(false, /* ignoreBall: */ true);
        theWalkToPointSkill(Pose2f(targetRelative.rotation), 1.f, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* disableAligning: */ true);
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToKickoffPoseImpl);
