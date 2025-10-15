/**
 * @file DemoPose.cpp
 *
 * This file implements a skill that <#...#>
 *
 * @author Ayleen Lührsen
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoPose,
       vars((unsigned)(theFrameInfo.time) timeWhenSignalDetected))
{
  using enum RefereeGesture::Gesture; // Some versions of clang++ do not accept RefereeSignal::Signal here.
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
    if(theEnhancedKeyStates.isPressedFor(KeyStates::headFront, 1000))
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
      if(theRefereeSignal.timeWhenDetected > timeWhenSignalDetected)
      {
        timeWhenSignalDetected = theRefereeSignal.timeWhenDetected;
        switch(theRefereeSignal.signal)
        {
          case kickInLeft:
            SystemCall::say("Kick In");
            goto kickInLeft;
          case kickInRight:
            SystemCall::say("Kick In");
            goto kickInRight;
        }
      }
    }
    action
    {
      gesture(useDefault, useDefault);
    }
  }

  state(kickInLeft) {action {gesture(armHorizontalSideways, useDefault);}}
  state(kickInRight) {action {gesture(useDefault, armHorizontalSideways);}}
}
