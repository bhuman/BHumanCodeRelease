/**
 * @file USControl.cpp
 * Implementation of a module that controls the firing strategy
 * of the ultrasound sensors.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "USControl.h"
#include "Platform/SystemCall.h"

USControl::USControl() :
  lastSendTime(0),
  lastSwitchTime(0),
  currentMode(0),
  commandSent(false)
{}

void USControl::update(USRequest& usRequest)
{
  if(!enable || (stopOnPlayDead &&
     theMotionRequest.motion == MotionRequest::specialAction &&
     theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead))
  {
    usRequest.sendMode = -1;
    usRequest.receiveMode = -1;
    commandSent = false;
  }
  else if(commandSent)
  {
    usRequest.sendMode = -1;
    if(SystemCall::getTimeSince(lastSendTime) >= timeBetweenSendAndReceive)
    {
      usRequest.receiveMode = modes[currentMode];
      commandSent = false;
    }
    // wait a while longer
  }
  else
  {
    usRequest.receiveMode = -1; // do not read anything

    // if a command has been sent last frame: check if we should send one this frame
    if(SystemCall::getTimeSince(lastSendTime) >= sendInterval)
    {
      if(SystemCall::getTimeSince(lastSwitchTime) >= switchInterval)
      {
        currentMode = (currentMode + 1) % modes.size();
        lastSwitchTime = SystemCall::getCurrentSystemTime();
      }
      usRequest.sendMode = modes[currentMode];
      lastSendTime = SystemCall::getCurrentSystemTime();
      commandSent = true;
    }
    else
      usRequest.sendMode = -1; // do not send anything
  }
}

MAKE_MODULE(USControl, sensing)
