/**
 * @file GetUpEngine.cpp
 *
 * This file implements the implementation of the GetUpEngine skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

SKILL_IMPLEMENTATION(GetUpEngineImpl,
{,
  IMPLEMENTS(GetUpEngine),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class GetUpEngineImpl : public GetUpEngineImplBase
{
  void execute(const GetUpEngine& p) override
  {
    theMotionRequest.motion = MotionRequest::getUp;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const GetUpEngine& p) const override
  {
    return theMotionInfo.motion == MotionRequest::getUp;
  }
};

MAKE_SKILL_IMPLEMENTATION(GetUpEngineImpl);
