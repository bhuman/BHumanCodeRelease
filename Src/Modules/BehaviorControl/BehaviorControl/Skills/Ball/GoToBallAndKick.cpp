/**
 * @file GoToBallAndKick.cpp
 *
 * This file implements an implementation of the GoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(GoToBallAndKickImpl,
{,
  IMPLEMENTS(GoToBallAndKick),
  CALLS(GoToBallHeadControl),
  CALLS(LookAtAngles),
  CALLS(RecordTargetAndSpeed),
  CALLS(TurnAngle),
  CALLS(WalkToBallAndKick),
  REQUIRES(FieldBall),
  REQUIRES(KickInfo),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(OdometryData),
  REQUIRES(PathPlanner),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  MODIFIES(BehaviorStatus),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
    (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
    (float)(600.f) goalkeeperDoNotIgnoreObstaclesDistance, /**< If the target is further away than this distance, the goalkeeper will not ignore obstacles. */
    (float)(500.f) goalkeeperIgnoreObstaclesDistance, /**< If the target is closer than this distance, the goalkeeper will ignore obstacles. */
  }),
});

class GoToBallAndKickImpl : public GoToBallAndKickImplBase
{
  option(GoToBallAndKick)
  {
    const Pose2f kickPose = Pose2f(p.targetDirection, theFieldBall.endPositionRelative).rotate(theKickInfo[p.kickType].rotationOffset).translate(theKickInfo[p.kickType].ballOffset);

    theRecordTargetAndSpeedSkill(kickPose.translation, 1.f);

    initial_state(walkToBallFarRange)
    {
      transition
      {
        if(theWalkToBallAndKickSkill.isDone())
        {
          if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
          {
            startRotation = theOdometryData.rotation;
            goto turnAfterKick;
          }
          else
            goto done;
        }
        else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theRobotInfo.isGoalkeeper())
          goto walkToBallIgnoreObstacles;
        else if(kickPose.translation.norm() < switchToLibWalkDistance)
          goto walkToBallCloseRange;
      }

      action
      {
        const float kickPower = kickLengthToPower(p.kickType, p.length, p.targetDirection);
        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * kickPose, p.speed);
        theGoToBallHeadControlSkill(kickPose.translation.norm(), /* lookAtKickTarget: */ true,
                                    /* kickTargetRelative: */ theFieldBall.endPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection));
        theWalkToBallAndKickSkill(p.targetDirection, p.kickType, p.alignPrecisely, kickPower, p.speed, obstacleAvoidance, p.preStepAllowed, p.turnKickAllowed, p.directionPrecision);
      }
    }

    state(walkToBallCloseRange)
    {
      transition
      {
        if(theWalkToBallAndKickSkill.isDone())
        {
          if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
          {
            startRotation = theOdometryData.rotation;
            goto turnAfterKick;
          }
          else
            goto done;
        }
        else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theRobotInfo.isGoalkeeper())
          goto walkToBallIgnoreObstacles;
        else if(kickPose.translation.norm() > switchToPathPlannerDistance)
          goto walkToBallFarRange;
      }

      action
      {
        const float kickPower = kickLengthToPower(p.kickType, p.length, p.targetDirection);
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ false);
        theGoToBallHeadControlSkill(kickPose.translation.norm(), /* lookAtKickTarget: */ true,
                                    /* kickTargetRelative: */ theFieldBall.endPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection));
        theWalkToBallAndKickSkill(p.targetDirection, p.kickType, p.alignPrecisely, kickPower, p.speed, obstacleAvoidance, p.preStepAllowed, p.turnKickAllowed, p.directionPrecision);
      }
    }

    state(walkToBallIgnoreObstacles)
    {
      transition
      {
        if(theWalkToBallAndKickSkill.isDone())
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
        else if(kickPose.translation.norm() > goalkeeperDoNotIgnoreObstaclesDistance || !theRobotInfo.isGoalkeeper())
          goto walkToBallCloseRange;
      }

      action
      {
        const float kickPower = kickLengthToPower(p.kickType, p.length, p.targetDirection);
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true);
        theGoToBallHeadControlSkill(kickPose.translation.norm(), /* lookAtKickTarget: */ true,
                                    /* kickTargetRelative: */ theFieldBall.endPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection));
        theWalkToBallAndKickSkill(p.targetDirection, p.kickType, p.alignPrecisely, kickPower, p.speed, obstacleAvoidance, p.preStepAllowed, p.turnKickAllowed, p.directionPrecision);
      }
    }

    target_state(turnAfterKick)
    {
      transition
      {
        if(theTurnAngleSkill.isDone() || theFieldBall.ballWasSeen(state_time - 300))
        {
          if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theRobotInfo.isGoalkeeper())
            goto walkToBallFarRange;
          else if(kickPose.translation.norm() < switchToLibWalkDistance)
            goto walkToBallCloseRange;
          else
            goto walkToBallFarRange;
        }
      }
      action
      {
        theRecordTargetAndSpeedSkill(kickPose.translation, 0.f);
        theLookAtAnglesSkill(Angle::normalize(theKickInfo[theMotionInfo.lastKickType].postRotationOffset - (theOdometryData.rotation - startRotation)), 23_deg);
        theTurnAngleSkill(theKickInfo[theMotionInfo.lastKickType].postRotationOffset);
      }
    }

    target_state(done)
    {
      transition
      {
        if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theRobotInfo.isGoalkeeper())
          goto walkToBallFarRange;
        else if(kickPose.translation.norm() < switchToLibWalkDistance)
          goto walkToBallCloseRange;
        else
          goto walkToBallFarRange;
      }

      action
      {
        const float kickPower = kickLengthToPower(p.kickType, p.length, p.targetDirection);
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true);
        theGoToBallHeadControlSkill(kickPose.translation.norm(), /* lookAtKickTarget: */ true,
                                    /* kickTargetRelative: */ theFieldBall.endPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection));
        theWalkToBallAndKickSkill(p.targetDirection, p.kickType, p.alignPrecisely, kickPower, p.speed, obstacleAvoidance, p.preStepAllowed, p.turnKickAllowed, p.directionPrecision);
      }
    }
  }

  /**
   * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
   * @param kickType The requested kick type.
   * @param length The requested length.
   * @return The resulting kick power (1 for kicks with constant length).
   */
  float kickLengthToPower(KickInfo::KickType kickType, float length, Angle direction) const
  {
    if(kickType == KickInfo::forwardFastRight || kickType == KickInfo::forwardFastLeft)
    {
      // This is only dummy code.
      if(length < 1000.f)
        return 0.f;
      else if(length > 6000.f)
        return 1.f;
      else
        return (length - 1000.f) / 5000.f;
    }
    else
    {
      Rangef useKickRange = theKickInfo[kickType].range;
      if(kickType == KickInfo::walkForwardsLeft || kickType == KickInfo::walkForwardsRight)
      {
        const bool isLeft = kickType == KickInfo::KickType::walkForwardsLeft;
        const Angle useMaxKickAngle = -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset;
        const Rangea angleClip(!isLeft ? 0_deg : useMaxKickAngle, !isLeft ? useMaxKickAngle : 0_deg);
        const float interpolation = Rangef::ZeroOneRange().limit(angleClip.limit(direction) / -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset);

        useKickRange.min = (1 - interpolation) * theKickInfo[kickType].range.min + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].range.min;
        useKickRange.max = (1 - interpolation) * theKickInfo[kickType].range.max + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].range.max;
      }
      return std::min(1.f, std::max(0.f, length - useKickRange.min) / (useKickRange.max - useKickRange.min));
    }
  }

  Angle startRotation; /**< The value of \c theOdometryData.rotation when the \c turnAfterKick state is entered. */
};

MAKE_SKILL_IMPLEMENTATION(GoToBallAndKickImpl);
