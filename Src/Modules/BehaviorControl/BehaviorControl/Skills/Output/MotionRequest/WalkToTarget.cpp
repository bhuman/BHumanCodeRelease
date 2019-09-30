/**
 * @file WalkToTarget.cpp
 *
 * This file implements the implementation of the WalkToTarget skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(WalkToTargetImpl,
{,
  IMPLEMENTS(WalkToTarget),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkToTargetImpl : public WalkToTargetImplBase
{
  void execute(const WalkToTarget& p) override
  {
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target = p.target;
    theMotionRequest.walkRequest.speed = p.speed;
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToTarget&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToTargetImpl);
