/**
 * @file PathToTarget.cpp
 *
 * This file implements an implementation of the PathToTarget skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(PathToTargetImpl,
{,
  IMPLEMENTS(PathToTarget),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(PathPlanner),
  REQUIRES(TeamBehaviorStatus),
  MODIFIES(MotionRequest),
});

class PathToTargetImpl : public PathToTargetImplBase
{
  void execute(const PathToTarget& p) override
  {
    const bool avoidOwnPenaltyArea = !theTeamBehaviorStatus.role.isGoalkeeper;
    theMotionRequest = thePathPlanner.plan(p.target, Pose2f(p.speed, p.speed, p.speed), avoidOwnPenaltyArea);
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const PathToTarget&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(PathToTargetImpl);
