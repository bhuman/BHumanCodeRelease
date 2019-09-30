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
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::absoluteSpeedMode;
    theMotionRequest.walkRequest.speed = p.speed;
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkAtAbsoluteSpeed&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkAtAbsoluteSpeedImpl);
