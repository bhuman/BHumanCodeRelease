/**
 * @file Stand.cpp
 *
 * This file implements the implementation of the Stand skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(StandImpl,
{,
  IMPLEMENTS(Stand),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class StandImpl : public StandImplBase
{
  void execute(const Stand& p) override
  {
    theMotionRequest.motion = MotionRequest::stand;
    theMotionRequest.standHigh = p.high;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const Stand&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::stand;
  }
};

MAKE_SKILL_IMPLEMENTATION(StandImpl);
