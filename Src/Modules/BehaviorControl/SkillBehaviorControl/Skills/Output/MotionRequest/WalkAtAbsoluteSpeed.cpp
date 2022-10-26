/**
 * @file WalkAtAbsoluteSpeed.cpp
 *
 * This file implements the implementation of the WalkAtAbsoluteSpeed skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(WalkAtAbsoluteSpeedImpl,
{,
  IMPLEMENTS(WalkAtAbsoluteSpeed),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkAtAbsoluteSpeedImpl : public WalkAtAbsoluteSpeedImplBase
{
  void execute(const WalkAtAbsoluteSpeed& p) override
  {
    theMotionRequest.motion = MotionRequest::walkAtAbsoluteSpeed;
    theMotionRequest.walkSpeed = p.speed;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkAtAbsoluteSpeed&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkAtAbsoluteSpeedImpl);
