/**
 * FootBumperStateProvider.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: arne
 *              simont@tzi.de
 */

#include "FootBumperStateProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(FootBumperStateProvider, sensing);

void FootBumperStateProvider::update(FootBumperState& footBumperState)
{
  // Check, if any bumper is pressed
  const bool leftFootLeft = !theDamageConfigurationBody.leftFootBumperDefect && checkContact(KeyStates::leftFootLeft, leftFootLeftDuration);
  const bool leftFootRight = !theDamageConfigurationBody.leftFootBumperDefect && checkContact(KeyStates::leftFootRight, leftFootRightDuration);
  const bool rightFootLeft = !theDamageConfigurationBody.rightFootBumperDefect &&  checkContact(KeyStates::rightFootLeft, rightFootLeftDuration);
  const bool rightFootRight = !theDamageConfigurationBody.rightFootBumperDefect && checkContact(KeyStates::rightFootRight, rightFootRightDuration);
  const bool contactLeftFoot = leftFootLeft || leftFootRight;
  const bool contactRightFoot = rightFootLeft || rightFootRight;
  // Update statistics
  if(contactLeftFoot)
  {
    contactBufferLeft.push_front(1);
    contactDurationLeft++;
  }
  else
  {
    contactBufferLeft.push_front(0);
    contactDurationLeft = 0;
  }
  if(contactRightFoot)
  {
    contactBufferRight.push_front(1);
    contactDurationRight++;
  }
  else
  {
    contactBufferRight.push_front(0);
    contactDurationRight = 0;
  }

  // Generate model
  if((theMotionInfo.motion == MotionInfo::stand || theMotionInfo.motion == MotionInfo::walk) &&
     (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING) && //The bumper is used for configuration in initial
     (theFallDownState.state == FallDownState::upright))
  {
    if(contactBufferLeft.sum() > contactThreshold)
    {
      footBumperState.contactLeft = true;
      footBumperState.contactDurationLeft = contactDurationLeft;
      if(contactLeftFoot)
        footBumperState.lastContactLeft = theFrameInfo.time;
    }
    else
    {
      footBumperState.contactLeft = false;
      footBumperState.contactDurationLeft = 0;
    }
    if(contactBufferRight.sum() > contactThreshold)
    {
      footBumperState.contactRight = true;
      footBumperState.contactDurationRight = contactDurationRight;
      if(contactRightFoot)
        footBumperState.lastContactRight = theFrameInfo.time;
    }
    else
    {
      footBumperState.contactRight = false;
      footBumperState.contactDurationRight = 0;
    }
  }
  else
  {
    footBumperState.contactLeft = false;
    footBumperState.contactRight = false;
    footBumperState.contactDurationLeft = 0;
    footBumperState.contactDurationRight = 0;
  }

  // Debugging stuff:

  if(debug && theFrameInfo.getTimeSince(lastSoundTime) > (int)soundDelay && (footBumperState.contactLeft || footBumperState.contactRight))
  {
    lastSoundTime = theFrameInfo.time;
    SystemCall::playSound("doh.wav");
  }

  PLOT("module:FootBumperStateProvider:sumLeft", contactBufferLeft.sum());
  PLOT("module:FootBumperStateProvider:durationLeft", contactDurationLeft);
  PLOT("module:FootBumperStateProvider:sumRight", contactBufferRight.sum());
  PLOT("module:FootBumperStateProvider:durationRight", contactDurationRight);
  PLOT("module:FootBumperStateProvider:contactLeft", footBumperState.contactLeft ? 10 : 0);
  PLOT("module:FootBumperStateProvider:contactRight", footBumperState.contactRight ? 10 : 0);
  PLOT("module:FootBumperStateProvider:leftFootLeft", leftFootLeft ? 10 : 0);
  PLOT("module:FootBumperStateProvider:leftFootRight", leftFootRight ? 10 : 0);
  PLOT("module:FootBumperStateProvider:rightFootLeft", rightFootLeft ? 10 : 0);
  PLOT("module:FootBumperStateProvider:rightFootRight", rightFootRight ? 10 : 0);
}

bool FootBumperStateProvider::checkContact(KeyStates::Key key, int& duration)
{
  bool pressed = theKeyStates.pressed[key];
  duration = pressed ? duration + 1 : 0;
  // if key is pressed longer than the malfunction threshold, it is ignored
  return pressed && duration < malfunctionThreshold;
}
