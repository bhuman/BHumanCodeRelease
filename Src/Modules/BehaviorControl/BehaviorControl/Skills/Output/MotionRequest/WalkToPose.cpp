/**
 * @file WalkToPose.cpp
 *
 * This file implements the implementation of the WalkToPose skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(WalkToPoseImpl,
{,
  IMPLEMENTS(WalkToPose),
  CALLS(RecordTargetAndSpeed),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkToPoseImpl : public WalkToPoseImplBase
{
  void execute(const WalkToPose& p) override
  {
    theRecordTargetAndSpeedSkill(p.target.translation, std::max(p.speed.translation.x(), p.speed.translation.y()));

    theMotionRequest.motion = MotionRequest::walkToPose;
    theMotionRequest.walkTarget = p.target;
    theMotionRequest.walkSpeed = p.speed;
    theMotionRequest.keepTargetRotation = p.keepTargetRotation;
    theMotionRequest.obstacleAvoidance = p.obstacleAvoidance;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToPose&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToPoseImpl);
