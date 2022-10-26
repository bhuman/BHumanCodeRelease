/**
 * @file CalibrateFootSole.cpp
 *
 * This file implements the implementation of the CalibrateFootSole skill.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"

SKILL_IMPLEMENTATION(CalibrateFootSoleImpl,
{,
  IMPLEMENTS(CalibrateFootSole),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class CalibrateFootSoleImpl : public CalibrateFootSoleImplBase
{
  void execute(const CalibrateFootSole&) override
  {
    theMotionRequest.motion = MotionRequest::calibration;
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const CalibrateFootSole&) const override
  {
    return theMotionInfo.executedPhase == MotionPhase::calibration;
  }
};

MAKE_SKILL_IMPLEMENTATION(CalibrateFootSoleImpl);
