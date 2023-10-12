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
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(GoToBallAndKickImpl,
{,
  IMPLEMENTS(GoToBallAndKick),
  CALLS(GoToBallHeadControl),
  CALLS(LookActive),
  CALLS(LookAtAngles),
  CALLS(PublishMotion),
  CALLS(TurnAngle),
  CALLS(WalkToBallAndKick),
  REQUIRES(FieldBall),
  REQUIRES(GameState),
  REQUIRES(KickInfo),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(OdometryData),
  REQUIRES(PathPlanner),
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
    const Pose2f kickPose = Pose2f(p.targetDirection, theFieldBall.interceptedEndPositionRelative).rotate(theKickInfo[p.kickType].rotationOffset).translate(theKickInfo[p.kickType].ballOffset);

    thePublishMotionSkill({.target = kickPose.translation});

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
        else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
          goto walkToBallIgnoreObstacles;
        else if(kickPose.translation.norm() < switchToLibWalkDistance)
          goto walkToBallCloseRange;
      }

      action
      {
        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * kickPose, p.speed);
        if(p.lookActiveWithBall)
          theLookActiveSkill({.withBall = true, .onlyOwnBall = true});
        else
          theGoToBallHeadControlSkill({.distanceToTarget = kickPose.translation.norm(),
                                       .lookAtKickTarget = true,
                                       .kickTargetRelative = theFieldBall.interceptedEndPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection)});
        theWalkToBallAndKickSkill({.targetDirection = p.targetDirection,
                                   .kickType = p.kickType,
                                   .alignPrecisely = p.alignPrecisely,
                                   .kickLength = p.length,
                                   .speed = p.speed,
                                   .obstacleAvoidance = obstacleAvoidance,
                                   .preStepAllowed = p.preStepAllowed,
                                   .turnKickAllowed = p.turnKickAllowed,
                                   .shiftTurnKickPose = p.shiftTurnKickPose,
                                   .directionPrecision = p.directionPrecision});
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
        else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
          goto walkToBallIgnoreObstacles;
        else if(kickPose.translation.norm() > switchToPathPlannerDistance)
          goto walkToBallFarRange;
      }

      action
      {
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ false, /* toBall: */ true);
        if(p.lookActiveWithBall)
          theLookActiveSkill({.withBall = true, .onlyOwnBall = true});
        else
          theGoToBallHeadControlSkill({.distanceToTarget = kickPose.translation.norm(),
                                       .lookAtKickTarget = false,
                                       .kickTargetRelative = theFieldBall.interceptedEndPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection)});
        theWalkToBallAndKickSkill({.targetDirection = p.targetDirection,
                                   .kickType = p.kickType,
                                   .alignPrecisely = p.alignPrecisely,
                                   .kickLength = p.length,
                                   .speed = p.speed,
                                   .obstacleAvoidance = obstacleAvoidance,
                                   .preStepAllowed = p.preStepAllowed,
                                   .turnKickAllowed = p.turnKickAllowed,
                                   .shiftTurnKickPose = p.shiftTurnKickPose,
                                   .directionPrecision = p.directionPrecision});
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
        else if(kickPose.translation.norm() > goalkeeperDoNotIgnoreObstaclesDistance || !theGameState.isGoalkeeper())
          goto walkToBallCloseRange;
      }

      action
      {
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* toBall: */ true);
        if(p.lookActiveWithBall)
          theLookActiveSkill({.withBall = true, .onlyOwnBall = true});
        else
          theGoToBallHeadControlSkill({.distanceToTarget = kickPose.translation.norm(),
                                       .lookAtKickTarget = false,
                                       .kickTargetRelative = theFieldBall.interceptedEndPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection)});
        theWalkToBallAndKickSkill({.targetDirection = p.targetDirection,
                                   .kickType = p.kickType,
                                   .alignPrecisely = p.alignPrecisely,
                                   .kickLength = p.length,
                                   .speed = p.speed,
                                   .obstacleAvoidance = obstacleAvoidance,
                                   .preStepAllowed = p.preStepAllowed,
                                   .turnKickAllowed = p.turnKickAllowed,
                                   .shiftTurnKickPose = p.shiftTurnKickPose,
                                   .directionPrecision = p.directionPrecision});
      }
    }

    target_state(turnAfterKick)
    {
      transition
      {
        if(theTurnAngleSkill.isDone() || theFieldBall.ballWasSeen(state_time - 300))
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
        thePublishMotionSkill({.target = kickPose.translation,
                               .speed = {0.f, 0.f, 0.f}});
        theLookAtAnglesSkill({.pan = Angle::normalize(theKickInfo[theMotionInfo.lastKickType].postRotationOffset - (theOdometryData.rotation - startRotation)),
                              .tilt = 23_deg});
        theTurnAngleSkill({.angle = theKickInfo[theMotionInfo.lastKickType].postRotationOffset});
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
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* toBall: */ true);
        if(p.lookActiveWithBall)
          theLookActiveSkill({.withBall = true, .onlyOwnBall = true});
        else
          theGoToBallHeadControlSkill({.distanceToTarget = kickPose.translation.norm(),
                                       .lookAtKickTarget = true,
                                       .kickTargetRelative = theFieldBall.interceptedEndPositionRelative + Vector2f(theKickInfo[p.kickType].range.max, 0.f).rotated(p.targetDirection)});
        theWalkToBallAndKickSkill({.targetDirection = p.targetDirection,
                                   .kickType = p.kickType,
                                   .alignPrecisely = p.alignPrecisely,
                                   .kickLength = p.length,
                                   .speed = p.speed,
                                   .obstacleAvoidance = obstacleAvoidance,
                                   .preStepAllowed = p.preStepAllowed,
                                   .turnKickAllowed = p.turnKickAllowed,
                                   .shiftTurnKickPose = p.shiftTurnKickPose,
                                   .directionPrecision = p.directionPrecision});
      }
    }
  }

  Angle startRotation; /**< The value of \c theOdometryData.rotation when the \c turnAfterKick state is entered. */
};

MAKE_SKILL_IMPLEMENTATION(GoToBallAndKickImpl);
