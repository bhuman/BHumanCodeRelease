/**
 * @file Special.cpp
 *
 * This file implements the implementation of the Special skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(SpecialImpl,
{,
  IMPLEMENTS(Special),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class SpecialImpl : public SpecialImplBase
{
  void execute(const Special& p) override
  {
    theMotionRequest.motion = MotionRequest::special;
    theMotionRequest.specialRequest = p.request;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const Special& p) const override
  {
    const KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromSpecialRequest(p.request);
    return theMotionInfo.isKeyframeMotion(keyframeMotionRequest.keyframeMotion, keyframeMotionRequest.mirror);
  }
};

MAKE_SKILL_IMPLEMENTATION(SpecialImpl);
