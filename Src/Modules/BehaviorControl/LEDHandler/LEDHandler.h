/**
 * @file LEDHandler.h
 * This file implements a module that generates the LEDRequest from certain representations.
 * @author Thijs Jeffry de Haas
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStateInformation.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Framework/Module.h"

MODULE(LEDHandler,
{,
  REQUIRES(BehaviorStateInformation),
  REQUIRES(GameState),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  PROVIDES(LEDRequest),
});

class LEDHandler : public LEDHandlerBase
{
  void update(LEDRequest& ledRequest) override;
};
