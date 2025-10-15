/**
 * @file FreeBallHoldingEngine.h
 *
 * This file declares a module that
 *
 * @author Harm Thordsen
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/MotionControl/FreeBallHoldingGenerator.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkGenerator.h"

MODULE(FreeBallHoldingEngine,
{,
  REQUIRES(MotionRequest),
  REQUIRES(WalkGenerator),
  PROVIDES(FreeBallHoldingGenerator),
});

class FreeBallHoldingEngine : public FreeBallHoldingEngineBase
{
  ENUM(FreeBallState,
  {,
    none,
    align,
    rotate,
    sideStep,
    zeroStep,
  });

  FreeBallState state = none;
  void update(FreeBallHoldingGenerator& theFreeBallHoldingGenerator) override;
};
