/**
 * @file JoystickControl.cpp
 *
 * This file implements a module that controls the robot through a joystick.
 * It provides a motion request and a head motion request.
 *
 * @author Thomas Röfer
 */

#include "JoystickControl.h"

MAKE_MODULE(JoystickControl);

void JoystickControl::update(MotionRequest& theMotionRequest)
{
  theMotionRequest.odometryData = theOdometryData;

  if(theJoystickState.valid)
  {
    joystickState = theJoystickState;

    for(const CombineAxes& combine : combineAxes)
      combine.apply(joystickState);
    for(const ButtonToAxis& buttonToAxis : buttonsToAxes)
      buttonToAxis.apply(joystickState);
    for(const AxisToButton& axisToButton : axesToButtons)
      axisToButton.apply(joystickState);

    if(joystickState.pressed(sitButton))
    {
      theMotionRequest.motion = MotionRequest::playDead;
      theMotionRequest.standHigh = true;
    }
    else if(joystickState.pressed(standButton))
    {
      if((lastButtons & 1 << standButton) == 0)
      {
        theMotionRequest.motion = MotionRequest::stand;
        theMotionRequest.standHigh ^= true;
      }
    }
    else
    {
      for(const ButtonToKick& buttonToKick : buttonsToKicks)
        if(joystickState.pressed(buttonToKick.button))
        {
          theMotionRequest.motion = MotionRequest::walkToBallAndKick;
          theMotionRequest.kickType = buttonToKick.kick;
          const float speed = std::max(std::abs(forwardAxis.value(joystickState)), std::abs(sidewaysAxis.value(joystickState)));
          theMotionRequest.walkSpeed = {speed, speed, speed};
          theMotionRequest.ballEstimate = theBallModel.estimate;
          theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
          theMotionRequest.directionPrecision = {-5_deg, 5_deg};
          theMotionRequest.standHigh = true;
          goto skipWalk;
        }
      theMotionRequest.walkSpeed.translation.x() = theMotionRequest.motion == MotionRequest::stand && deadZone.isInside(forwardAxis.value(joystickState)) ? 0.f : forwardAxis.value(joystickState);
      theMotionRequest.walkSpeed.translation.y() = theMotionRequest.motion == MotionRequest::stand && deadZone.isInside(sidewaysAxis.value(joystickState)) ? 0.f : sidewaysAxis.value(joystickState);
      theMotionRequest.walkSpeed.rotation = theMotionRequest.motion == MotionRequest::stand && deadZone.isInside(turnAxis.value(joystickState)) ? 0.f : turnAxis.value(joystickState);
      if(theMotionRequest.walkSpeed != Pose2f())
      {
        theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
        theMotionRequest.standHigh = true;
      }
    skipWalk:;
    }
  }
  else
  {
    joystickState = JoystickState();
    theMotionRequest.motion = MotionRequest::playDead;
    theMotionRequest.standHigh = true;
  }

  headMotionRequest.mode = HeadMotionRequest::panTiltMode;
  headMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  headMotionRequest.pan = panAxis.value(joystickState);
  headMotionRequest.tilt = tiltAxis.value(joystickState);
  headMotionRequest.speed = panTiltSpeed;
  headMotionRequest.stopAndGoMode = false;

  lastButtons = joystickState.buttons;
}
