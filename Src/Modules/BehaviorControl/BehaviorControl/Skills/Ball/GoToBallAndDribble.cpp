/**
 * @file GoToBallAndDribble.cpp
 *
 * This file implements an implementation of the GoToBallAndDribble skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(GoToBallAndDribbleImpl,
{,
  IMPLEMENTS(GoToBallAndDribble),
  REQUIRES(FieldBall),
  REQUIRES(LibWalk),
  REQUIRES(PathPlanner),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
  MODIFIES(BehaviorStatus),
  CALLS(GoToBallHeadControl),
  CALLS(Dribble),
  CALLS(RecordTargetAndSpeed),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
    (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
  }),
});

class GoToBallAndDribbleImpl : public GoToBallAndDribbleImplBase
{
  option(GoToBallAndDribble)
  {
    Pose2f dribblePose(p.targetDirection, theFieldBall.endPositionRelative);

    theRecordTargetAndSpeedSkill(dribblePose.translation, 1.f);
    theGoToBallHeadControlSkill(dribblePose.translation.norm());

    initial_state(dribbleFarRange)
    {
      transition
      {
        if(dribblePose.translation.squaredNorm() < sqr(switchToLibWalkDistance))
          goto dribbleCloseRange;
      }

      action
      {
        auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * dribblePose, Pose2f(1.f, 1.f, 1.f));
        theDribbleSkill(p.targetDirection, Pose2f(1.f, 1.f, 1.f), obstacleAvoidance, p.alignPrecisely, p.kickPower, p.preStepAllowed, p.turnKickAllowed, p.directionPrecision);
      }
    }

    state(dribbleCloseRange)
    {
      transition
      {
        if(dribblePose.translation.squaredNorm() > sqr(switchToPathPlannerDistance))
          goto dribbleFarRange;
      }

      action
      {
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(dribblePose, /* rough: */ true, /* disableObstacleAvoidance: */ false);
        theDribbleSkill(p.targetDirection, Pose2f(1.f, 1.f, 1.f), obstacleAvoidance, p.alignPrecisely, p.kickPower, p.preStepAllowed, p.turnKickAllowed, p.directionPrecision);
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(GoToBallAndDribbleImpl);
