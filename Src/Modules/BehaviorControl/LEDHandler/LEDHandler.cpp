/**
 * @file LEDHandler.cpp
 * This file implements a module that generates the LEDRequest from certain representations.
 * @author Thijs Jeffry de Haas
 * @author Jesse Richter-Klug
 */

#include "LEDHandler.h"

MAKE_MODULE(LEDHandler);

void LEDHandler::update(LEDRequest& ledRequest)
{
  //reset
  FOREACH_ENUM(LEDRequest::LED, led)
    ledRequest.ledStates[led] = LEDRequest::off;

  // Prepare overwrite LED
  if((theMotionInfo.isMotion(MotionPhase::prepare) || theMotionInfo.isMotion(MotionPhase::playDead)) && theMotionRequest.motion == MotionRequest::prepare)
  {
    ledRequest.ledStates[LEDRequest::red] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::blue] = LEDRequest::on;
  }
  else if(theGameState.playerState == GameState::unstiff)
    ledRequest.ledStates[LEDRequest::blue] = LEDRequest::blinking;
  else if(theGameState.playerState == GameState::calibration)
  {
    ledRequest.ledStates[LEDRequest::red] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::green] = LEDRequest::half;
  }
  else if(theGameState.isPenalized())
    ledRequest.ledStates[LEDRequest::red] = LEDRequest::on;
  else if(theGameState.isReady())
  {
    ledRequest.ledStates[LEDRequest::blue] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::green] = LEDRequest::half;
  }
  else if(theGameState.isSet())
  {
    ledRequest.ledStates[LEDRequest::red] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::green] = LEDRequest::on;
  }
  else if(theGameState.isPlaying())
    ledRequest.ledStates[LEDRequest::green] = LEDRequest::on;
  else
    FOREACH_ENUM(LEDRequest::LED, led)
      ledRequest.ledStates[led] = LEDRequest::on;

  // If something else than prepare is requested, but prepare cannot be left yet, the LEDs blink quickly
  if(theBehaviorStateInformation.transitionFailed || theBehaviorStateInformation.remoteControlActive ||
     (theMotionInfo.isMotion(MotionPhase::prepare) && theMotionRequest.motion != MotionRequest::prepare)
     || (theMotionInfo.isMotion(MotionPhase::playDead) && theMotionRequest.motion != MotionRequest::playDead && theMotionRequest.motion != MotionRequest::prepare))
    FOREACH_ENUM(LEDRequest::LED, led)
    {
      LEDRequest::State& state = ledRequest.ledStates[led];
      switch(state)
      {
        case LEDRequest::on:
        case LEDRequest::blinking:
          state = theBehaviorStateInformation.remoteControlActive ? LEDRequest::blinking : LEDRequest::fastBlinking;
          break;
        case LEDRequest::half:
        case LEDRequest::halfBlinking:
          state = theBehaviorStateInformation.remoteControlActive ? LEDRequest::halfBlinking : LEDRequest::halfFastBlinking;
          break;
      }
    }
}
