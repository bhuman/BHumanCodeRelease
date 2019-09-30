/**
 * @file SpecialAction.cpp
 *
 * This file implements the implementation of the SpecialAction skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(SpecialActionImpl,
{,
  IMPLEMENTS(SpecialAction),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class SpecialActionImpl : public SpecialActionImplBase
{
  void execute(const SpecialAction& p) override
  {
    theMotionRequest.motion = MotionRequest::specialAction;
    theMotionRequest.specialActionRequest.specialAction = p.id;
    theMotionRequest.specialActionRequest.mirror = p.mirror;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const SpecialAction& p) const override
  {
    return theMotionInfo.motion == MotionRequest::specialAction &&
           theMotionInfo.specialActionRequest.specialAction == p.id &&
           theMotionInfo.specialActionRequest.mirror == p.mirror;
  }
};

MAKE_SKILL_IMPLEMENTATION(SpecialActionImpl);
