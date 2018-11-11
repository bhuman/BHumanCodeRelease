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
  const bool leftFootLeft = !theDamageConfigurationBody.sides[Legs::left].footBumperDefect && checkContact(KeyStates::lFootLeft, leftFootLeftDuration);
  const bool leftFootRight = !theDamageConfigurationBody.sides[Legs::left].footBumperDefect && checkContact(KeyStates::lFootRight, leftFootRightDuration);
  const bool rightFootLeft = !theDamageConfigurationBody.sides[Legs::right].footBumperDefect &&  checkContact(KeyStates::rFootLeft, rightFootLeftDuration);
  const bool rightFootRight = !theDamageConfigurationBody.sides[Legs::right].footBumperDefect && checkContact(KeyStates::rFootRight, rightFootRightDuration);
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
      footBumperState.status[Legs::left].contact = true;
      footBumperState.status[Legs::left].contactDuration = contactDurationLeft;
      if(contactLeftFoot)
        footBumperState.status[Legs::left].lastContact = theFrameInfo.time;
    }
    else
    {
      footBumperState.status[Legs::left].contact = false;
      footBumperState.status[Legs::left].contactDuration = 0;
    }
    if(contactBufferRight.sum() > contactThreshold)
    {
      footBumperState.status[Legs::right].contact = true;
      footBumperState.status[Legs::right].contactDuration = contactDurationRight;
      if(contactRightFoot)
        footBumperState.status[Legs::right].lastContact = theFrameInfo.time;
    }
    else
    {
      footBumperState.status[Legs::right].contact = false;
      footBumperState.status[Legs::right].contactDuration = 0;
    }
  }
  else
  {
    footBumperState.status[Legs::left].contact = false;
    footBumperState.status[Legs::right].contact = false;
    footBumperState.status[Legs::left].contactDuration = 0;
    footBumperState.status[Legs::right].contactDuration = 0;
  }

  // Debugging stuff:

  if(debug && theFrameInfo.getTimeSince(lastSoundTime) > (int)soundDelay && (footBumperState.status[Legs::left].contact || footBumperState.status[Legs::right].contact))
  {
    lastSoundTime = theFrameInfo.time;
    SystemCall::playSound("doh.wav");
  }

  PLOT("module:FootBumperStateProvider:sumLeft", contactBufferLeft.sum());
  PLOT("module:FootBumperStateProvider:durationLeft", contactDurationLeft);
  PLOT("module:FootBumperStateProvider:sumRight", contactBufferRight.sum());
  PLOT("module:FootBumperStateProvider:durationRight", contactDurationRight);
  PLOT("module:FootBumperStateProvider:contactLeft", footBumperState.status[Legs::left].contact ? 10 : 0);
  PLOT("module:FootBumperStateProvider:contactRight", footBumperState.status[Legs::right].contact ? 10 : 0);
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
