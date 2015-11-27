/*
 * FootContactModelProvider.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: arne
 *              simont@tzi.de
 */

#include "FootContactModelProvider.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(FootContactModelProvider, sensing);

FootContactModelProvider::FootContactModelProvider() :
  contactDurationLeft(0), contactDurationRight(0), leftFootLeftDuration(0), leftFootRightDuration(0), rightFootLeftDuration(0), rightFootRightDuration(0)
{}

void FootContactModelProvider::update(FootContactModel& model)
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
      model.contactLeft = true;
      model.contactDurationLeft = contactDurationLeft;
      if(contactLeftFoot)
        model.lastContactLeft = theFrameInfo.time;
    }
    else
    {
      model.contactLeft = false;
      model.contactDurationLeft = 0;
    }
    if(contactBufferRight.sum() > contactThreshold)
    {
      model.contactRight = true;
      model.contactDurationRight = contactDurationRight;
      if(contactRightFoot)
        model.lastContactRight = theFrameInfo.time;
    }
    else
    {
      model.contactRight = false;
      model.contactDurationRight = 0;
    }
  }
  else
  {
    model.contactLeft = false;
    model.contactRight = false;
    model.contactDurationLeft = 0;
    model.contactDurationRight = 0;
  }

  // Debugging stuff:

  if(debug && theFrameInfo.getTimeSince(lastSoundTime) > (int) soundDelay && (model.contactLeft || model.contactRight))
  {
    lastSoundTime = theFrameInfo.time;
    SystemCall::playSound("doh.wav");
  }

  DECLARE_PLOT("module:FootContactModelProvider:sumLeft");
  DECLARE_PLOT("module:FootContactModelProvider:durationLeft");
  DECLARE_PLOT("module:FootContactModelProvider:contactLeft");
  DECLARE_PLOT("module:FootContactModelProvider:sumRight");
  DECLARE_PLOT("module:FootContactModelProvider:durationRight");
  DECLARE_PLOT("module:FootContactModelProvider:contactRight");
  DECLARE_PLOT("module:FootContactModelProvider:leftFootLeft");
  DECLARE_PLOT("module:FootContactModelProvider:leftFootRight");
  DECLARE_PLOT("module:FootContactModelProvider:rightFootLeft");
  DECLARE_PLOT("module:FootContactModelProvider:rightFootRight");
  PLOT("module:FootContactModelProvider:sumLeft", contactBufferLeft.sum());
  PLOT("module:FootContactModelProvider:durationLeft", contactDurationLeft);
  PLOT("module:FootContactModelProvider:sumRight", contactBufferRight.sum());
  PLOT("module:FootContactModelProvider:durationRight", contactDurationRight);
  PLOT("module:FootContactModelProvider:contactLeft", model.contactLeft ? 10 : 0);
  PLOT("module:FootContactModelProvider:contactRight", model.contactRight ? 10 : 0);
  PLOT("module:FootContactModelProvider:leftFootLeft", leftFootLeft ? 10 : 0);
  PLOT("module:FootContactModelProvider:leftFootRight", leftFootRight ? 10 : 0);
  PLOT("module:FootContactModelProvider:rightFootLeft", rightFootLeft ? 10 : 0);
  PLOT("module:FootContactModelProvider:rightFootRight", rightFootRight ? 10 : 0);
}

bool FootContactModelProvider::checkContact(KeyStates::Key key, int& duration)
{
  bool pressed = theKeyStates.pressed[key];
  duration = pressed ? duration + 1 : 0;
  // if key is pressed longer than the malfunction threshold, it is ignored
  return pressed && duration < malfunctionThreshold;
}
