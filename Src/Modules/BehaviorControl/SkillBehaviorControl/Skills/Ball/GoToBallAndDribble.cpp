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
  MODIFIES(BehaviorStatus),
  CALLS(GoToBallHeadControl),
  CALLS(Dribble),
  CALLS(PublishMotion),
  CALLS(LookActive),
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
    Pose2f dribblePose(p.targetDirection, theFieldBall.interceptedEndPositionRelative);

    thePublishMotionSkill({.target = dribblePose.translation});
    if(p.lookActiveWithBall)
      theLookActiveSkill({.withBall = true, .onlyOwnBall = true});
    else
      theGoToBallHeadControlSkill({.distanceToTarget = dribblePose.translation.norm()});

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
        theDribbleSkill({.targetDirection = p.targetDirection,
                         .obstacleAvoidance = obstacleAvoidance,
                         .alignPrecisely = p.alignPrecisely,
                         .kickLength = p.kickLength,
                         .preStepAllowed = p.preStepAllowed,
                         .turnKickAllowed = p.turnKickAllowed,
                         .directionPrecision = p.directionPrecision});
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
        auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(dribblePose, /* rough: */ true, /* disableObstacleAvoidance: */ false, /* toBall: */ true);
        theDribbleSkill({.targetDirection = p.targetDirection,
                         .obstacleAvoidance = obstacleAvoidance,
                         .alignPrecisely = p.alignPrecisely,
                         .kickLength = p.kickLength,
                         .preStepAllowed = p.preStepAllowed,
                         .turnKickAllowed = p.turnKickAllowed,
                         .directionPrecision = p.directionPrecision});
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(GoToBallAndDribbleImpl);
