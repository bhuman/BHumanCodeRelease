/**
 * @file DemoPose.cpp
 *
 * This file implements a skill that <#...#>
 *
 * @author Ayleen Lührsen
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoPose)
{
  using enum RefereePercept::Gesture;
  using enum ArmKeyFrameRequest::ArmKeyFrameId;

  // Look at referee, stand upright, and announce gesture if known.
  const auto gesture = [this](const ArmKeyFrameRequest::ArmKeyFrameId left,
                        const ArmKeyFrameRequest::ArmKeyFrameId right)
  {
    Stand({.high = true});
    KeyFrameLeftArm({.motion = left});
    KeyFrameRightArm({.motion = right});
    LookAtAngles({.pan = 0.f,
                  .tilt = -15_deg});
  };

  common_transition
  {
    if(theEnhancedKeyStates.isPressedFor(KeyStates::Key::headFront, 1000))
      goto posing;
  }

  initial_state(initial)
  {
    action
    {
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
    }
  }

  state(posing)
  {
    transition
    {
      if(state_time > 1000) // wait for a second until detections are accepted
        switch(theRefereePercept.gesture)
        {
          case RefereePercept::goalKickBlue:
            SystemCall::say("Goal Kick");
            goto goalKickBlue;
          case RefereePercept::goalKickRed:
            SystemCall::say("Goal Kick");
            goto goalKickRed;
          case RefereePercept::kickInBlue:
            SystemCall::say("Kick In");
            goto kickInBlue;
          case RefereePercept::kickInRed:
            SystemCall::say("Kick In");
            goto kickInRed;
          case RefereePercept::cornerKickBlue:
            SystemCall::say("Corner Kick");
            goto cornerKickBlue;
          case RefereePercept::cornerKickRed:
            SystemCall::say("Corner Kick");
            goto cornerKickRed;
          case RefereePercept::goalBlue:
            SystemCall::say("Goal");
            goto goalBlue;
          case RefereePercept::goalRed:
            SystemCall::say("Goal");
            goto goalRed;
          case RefereePercept::pushingFreeKickBlue:
            SystemCall::say("Free Kick");
            goto pushingFreeKickBlue;
          case RefereePercept::pushingFreeKickRed:
            SystemCall::say("Free Kick");
            goto pushingFreeKickRed;
          case RefereePercept::fullTime:
            SystemCall::say("Full time");
            goto fullTime;
        }
    }
    action
    {
      gesture(useDefault, useDefault);
    }
  }

  state(kickInBlue) {action {gesture(armHorizontalSideways, useDefault);}}
  state(kickInRed) {action {gesture(useDefault, armHorizontalSideways);}}
  state(goalKickBlue) {action {gesture(arm45degreeUpSideways, useDefault);}}
  state(goalKickRed) {action {gesture(useDefault, arm45degreeUpSideways);}}
  state(cornerKickBlue) {action {gesture(arm45degreeDownSideways, useDefault);}}
  state(cornerKickRed) {action {gesture(useDefault, arm45degreeDownSideways);}}
  state(goalBlue) {action {gesture(armHorizontalSideways, arm45degreeUpFront);}}
  state(goalRed) {action {gesture(arm45degreeUpFront, armHorizontalSideways);}}
  state(pushingFreeKickBlue) {action {gesture(armHorizontalSideways, armHandToChest);}}
  state(pushingFreeKickRed) {action {gesture(armHandToChest, armHorizontalSideways);}}
  state(fullTime)
  {
    action
    {
      if(state_time / 1500 & 1)
        gesture(armHandToChest, armHandToChest);
      else
        gesture(armHorizontalSideways, armHorizontalSideways);
    }
  }
}
