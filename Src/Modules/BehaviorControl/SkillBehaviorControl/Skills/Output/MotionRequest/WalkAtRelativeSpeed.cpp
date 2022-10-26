/**
 * @file WalkAtRelativeSpeed.cpp
 *
 * This file implements the implementation of the WalkAtRelativeSpeed skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(WalkAtRelativeSpeedImpl,
{,
  IMPLEMENTS(WalkAtRelativeSpeed),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkAtRelativeSpeedImpl : public WalkAtRelativeSpeedImplBase
{
  void execute(const WalkAtRelativeSpeed& p) override
  {
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
    theMotionRequest.walkSpeed = p.speed;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkAtRelativeSpeed&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkAtRelativeSpeedImpl);
