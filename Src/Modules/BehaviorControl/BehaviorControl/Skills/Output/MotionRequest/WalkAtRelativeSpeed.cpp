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
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
    theMotionRequest.walkRequest.speed = p.speed;
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkAtRelativeSpeed&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkAtRelativeSpeedImpl);
