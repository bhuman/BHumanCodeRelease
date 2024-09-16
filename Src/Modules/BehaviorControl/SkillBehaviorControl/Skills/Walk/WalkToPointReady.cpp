/**
 * @file WalkToPointReady.cpp
 *
 * This file implements an implementation for the WalkToPointReady skill.
 *
 * @author Arne Hasselbring
 * @author Fynn Böse
 */

#include "SkillBehaviorControl.h"
#include <cmath>

option((SkillBehaviorControl) WalkToPointReady,
       args((const Pose2f&) target,
            (const Pose2f&) speed,
            (ReduceWalkSpeedType) reduceWalkingSpeed),
       defs((int)(2000) timeToEmergencyMode, /**< If only this time is left in the ready state, the robot will just turn to its final rotation. */
            (int)(5000) timeToTOI)) /**< If only this time is left in the ready state, the robot will be forced to keep the opponent goal (set as the targetOfInterest, TOI) in front of it. */
{
  const Pose2f targetRelative = theRobotPose.inverse() * target;
  const int timeUntilSetStarts = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);
  const bool opponentPenaltyKickGoalkeeper = theGameState.state == GameState::setupOpponentPenaltyKick && theGameState.isGoalkeeper();
  const int durationUntilAnticipatedIllegal = (theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) / 2;
  const bool shouldEmergencyTurn = timeUntilSetStarts < timeToEmergencyMode &&
                                   !theIllegalAreas.willPositionBeIllegal(theRobotPose.translation, 175.f);

  common_transition
  {
    if(theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal))
      goto leaveIllegalAreas;
  }

  initial_state(walkToPointReady)
  {
    transition
    {
      if(targetRelative.translation.squaredNorm() < sqr(200.f))
      {
        if(theRobotPose.translation.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius) || opponentPenaltyKickGoalkeeper)
          goto adjustToFinalPosePrecisely;
        else
          goto adjustToFinalPose;
      }
      else if(shouldEmergencyTurn)
        goto emergencyAdjustment;
    }
    action
    {
      const bool useTargetOfInterest = timeUntilSetStarts < timeToTOI;
      // Always use the opponent goal as target, except for penalty kicks
      Vector2f targetOfInterest = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);
      if(theGameState.isPenaltyKick())
        targetOfInterest = theGameState.isForOpponentTeam() ? Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f) : Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);
      targetOfInterest = theRobotPose.inverse() * targetOfInterest;

      LookActive({.ignoreBall = true});
      if(theGameState.isKickOff())
        WalkToPoint({.target = targetRelative,
                     .speed = theGameState.kickOffSetupFromTouchlines ? speed : Pose2f(1.f, 1.f, 1.f),
                     .reduceWalkingSpeed = reduceWalkingSpeed,
                     .rough = targetRelative.translation.norm() < 500.f,
                     .targetOfInterest = useTargetOfInterest ? targetOfInterest : std::optional<Vector2f>()});
      else
        WalkToPointObstacle({.target = targetRelative,
                             .speed = theGameState.kickOffSetupFromTouchlines ? speed : Pose2f(1.f, 1.f, 1.f),
                             .reduceWalkingSpeed = reduceWalkingSpeed,
                             .rough = targetRelative.translation.norm() < 500.f,
                             .targetOfInterest = useTargetOfInterest ? targetOfInterest : std::optional<Vector2f>() });
    }
  }

  state(adjustToFinalPose)
  {
    transition
    {
      if(state_time > 1000 && targetRelative.translation.squaredNorm() < sqr(40.f) && std::abs(targetRelative.rotation) < 5_deg)
        goto stand;
      else if(shouldEmergencyTurn &&
              targetRelative.translation.squaredNorm() > sqr(300.f) && std::abs(targetRelative.rotation) > 20_deg)
        goto emergencyAdjustment;
    }
    action
    {
      LookActive({.ignoreBall = true});
      if(theGameState.isKickOff())
        WalkToPoint({.target = targetRelative,
                     .speed = speed,
                     .reduceWalkingSpeed = reduceWalkingSpeed,
                     .rough = true,
                     .disableObstacleAvoidance = theStrategyStatus.role == PositionRole::toRole(PositionRole::goalkeeper)});
      else
        WalkToPointObstacle({.target = targetRelative,
                             .speed = speed,
                             .reduceWalkingSpeed = reduceWalkingSpeed,
                             .rough = true,
                             .disableObstacleAvoidance = theStrategyStatus.role == PositionRole::toRole(PositionRole::goalkeeper)});
    }
  }

  state(adjustToFinalPosePrecisely)
  {
    transition
    {
      if(state_time > 1000 && targetRelative.translation.squaredNorm() < sqr(25.f) && std::abs(targetRelative.rotation) < 3_deg)
        goto stand;
      else if(shouldEmergencyTurn &&
              targetRelative.translation.squaredNorm() > sqr(300.f) && std::abs(targetRelative.rotation) > 20_deg)
        goto emergencyAdjustment;
    }
    action
    {
      LookActive({.ignoreBall = true});
      if(theGameState.isKickOff())
        WalkToPoint({.target = targetRelative,
                     .speed = speed,
                     .reduceWalkingSpeed = reduceWalkingSpeed,
                     .rough = true});
      else
        WalkToPointObstacle({.target = targetRelative,
                             .speed = speed,
                             .reduceWalkingSpeed = reduceWalkingSpeed,
                             .rough = true});
    }
  }

  state(stand)
  {
    transition
    {
      if(targetRelative.translation.squaredNorm() > sqr(opponentPenaltyKickGoalkeeper ? 50.f : 300.f) || std::abs(targetRelative.rotation) > (opponentPenaltyKickGoalkeeper ? 5_deg : 15_deg) ||
         theIllegalAreas.isPositionIllegal(theRobotPose.translation, 145.f))
        goto walkToPointReady;
      if(action_done && state_time > 1000)
        goto standHigh;
    }
    action
    {
      LookActive({.ignoreBall = true});
      Stand();
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
      LookActive({.ignoreBall = true});
      Stand({.high = true});
    }
  }

  state(emergencyAdjustment)
  {
    action
    {
      LookActive({.ignoreBall = true});
      ASSERT(theGameState.isKickOff() || theGameState.isPenaltyKick());
      Angle ballDirection = theGameState.isKickOff() ? (theRobotPose.inverse()).translation.angle() :
                            (theGameState.isForOwnTeam() ?
                             (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f)).angle() :
                             (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f)).angle());
      WalkToPointObstacle({.target = {ballDirection},
                           .reduceWalkingSpeed = reduceWalkingSpeed,
                           .rough = true,
                           .disableObstacleAvoidance = true,
                           .disableAligning = true});
    }
  }

  state(leaveIllegalAreas)
  {
    transition
    {
      if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal))
        goto walkToPointReady;
    }
    action
    {
      WalkPotentialField({.target = theRobotPose.translation,
                          .straight = true});
      LookActive({.withBall = false});
    }
  }
}
