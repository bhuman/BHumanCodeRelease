/**
 * @file RollingBallStateProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "RollingBallStateProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(RollingBallStateProvider);

void RollingBallStateProvider::update(RollingBallState& theRollingBallState)
{
  theRollingBallState.isActive = true;

  if(theGameState.isInitial() && theFrameInfo.getTimeSince(headFrontTimestamp) > sideSwitchTimeout && theKeyStates.pressed[KeyStates::headFront])
  {
    headFrontTimestamp = theFrameInfo.time;
    theRollingBallState.isRampLeftSide = !theRollingBallState.isRampLeftSide;
    if(theRollingBallState.isRampLeftSide)
      SystemCall::say("Ramp Left");
    else
      SystemCall::say("Ramp Right");
  }

  if(theGameState.isInitial() && theFrameInfo.getTimeSince(headRearTimestamp) > rampDistanceTimeout && theKeyStates.pressed[KeyStates::headRear])
  {
    headRearTimestamp = theFrameInfo.time;
    theRollingBallState.approxRampDistance += rampDistanceStepsize;
    if(!rampDistanceRange.isInside(theRollingBallState.approxRampDistance))
      theRollingBallState.approxRampDistance = rampDistanceRange.min;

    SystemCall::say((std::to_string(static_cast<int>(theRollingBallState.approxRampDistance))).c_str());
  }

  theRollingBallState.maxRampDistance = rampDistanceRange.max;
}
