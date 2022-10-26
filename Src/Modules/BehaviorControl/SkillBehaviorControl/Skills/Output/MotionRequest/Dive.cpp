/**
 * @file Dive.cpp
 *
 * This file implements the implementation of the Dive skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(DiveImpl,
{,
  IMPLEMENTS(Dive),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class DiveImpl : public DiveImplBase
{
  void execute(const Dive& p) override
  {
    theMotionRequest.motion = MotionRequest::dive;
    theMotionRequest.diveRequest = p.request;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const Dive& p) const override
  {
    const KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromDiveRequest(p.request);
    return theMotionInfo.isKeyframeMotion(keyframeMotionRequest.keyframeMotion, keyframeMotionRequest.mirror);
  }
};

MAKE_SKILL_IMPLEMENTATION(DiveImpl);
