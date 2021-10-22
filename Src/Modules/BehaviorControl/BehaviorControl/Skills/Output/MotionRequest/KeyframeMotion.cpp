/**
 * @file KeyframeMotion.cpp
 *
 * This file implements the implementation of the KeyframeMotion skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(KeyframeMotionImpl,
{,
  IMPLEMENTS(KeyframeMotion),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class KeyframeMotionImpl : public KeyframeMotionImplBase
{
  void execute(const KeyframeMotion& p) override
  {
    theMotionRequest.motion = MotionRequest::keyframeMotion;
    theMotionRequest.keyframeMotionRequest.keyframeMotion = p.id;
    theMotionRequest.keyframeMotionRequest.mirror = p.mirror;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const KeyframeMotion& p) const override
  {
    return theMotionInfo.isKeyframeMotion(p.id, p.mirror);
  }
};

MAKE_SKILL_IMPLEMENTATION(KeyframeMotionImpl);
