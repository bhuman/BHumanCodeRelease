/**
 * @file JoystickControl.cpp
 *
 * This file implements a module that controls the robot through a joystick.
 * It can either provide a motion request and a head motion request or
 * a shared autonomy request (not both at the same time).
 *
 * @author Thomas Röfer
 */

#include "JoystickControl.h"

MAKE_MODULE(JoystickControl);

void JoystickControl::update(MotionRequest& theMotionRequest)
{
  theMotionRequest.odometryData = theOdometryData;

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
    theMotionRequest.walkSpeed.translation.x() = forwardAxis.value(joystickState);
    theMotionRequest.walkSpeed.translation.y() = sidewaysAxis.value(joystickState);
    theMotionRequest.walkSpeed.rotation = turnAxis.value(joystickState);
    if(theMotionRequest.walkSpeed != Pose2f())
    {
      theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
      theMotionRequest.standHigh = true;
    }
  }

  headMotionRequest.mode = HeadMotionRequest::panTiltMode;
  headMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  headMotionRequest.pan = panAxis.value(joystickState);
  headMotionRequest.tilt = tiltAxis.value(joystickState);
  headMotionRequest.speed = panTiltSpeed;
  headMotionRequest.stopAndGoMode = false;

  lastButtons = joystickState.buttons;
}

void JoystickControl::update(SharedAutonomyRequest& theSharedAutonomyRequest)
{
  // This is not the remote controlled robot.
  if(theGameState.playerNumber & 1)
  {
    theSharedAutonomyRequest.isValid = false;
    return;
  }

  unsigned lastButtons = this->lastButtons;

  if(theGameState.isSet() && !theGameState.isPenaltyKick() && !theExtendedGameState.wasSet())
    passThrough = passThroughSharedAutonomyRequest;

  passThrough &= !theJoystickState.pressed(kickButton);
  if(passThrough)
  {
    theSharedAutonomyRequest = theSharedAutonomyRequest2;
    return;
  }

  const Pose2f oldSpeed = motionRequest.walkSpeed;
  update(motionRequest);
  const bool force = joystickState.pressed(forceButton);
  if(joystickState.pressed(passButton))
    theSharedAutonomyRequest.controlOperations = force ? SharedAutonomyRequest::passToMate : SharedAutonomyRequest::playBallPassToMate;
  else if(joystickState.pressed(dribbleButton) && motionRequest.walkSpeed.translation != Vector2f::Zero())
  {
    theSharedAutonomyRequest.controlOperations = force ? SharedAutonomyRequest::dribbleInDirection : SharedAutonomyRequest::playBallDribbleInDirection;
    if((oldSpeed.translation - motionRequest.walkSpeed.translation).norm() > directionThreshold
       || (lastButtons & 1 << dribbleButton) == 0)
      direction = (Pose2f(theRobotPose.rotation) * motionRequest.walkSpeed.translation).angle();
    theSharedAutonomyRequest.targetPose.rotation = direction;
  }
  else if(joystickState.pressed(kickButton) && motionRequest.walkSpeed.translation != Vector2f::Zero())
  {
    theSharedAutonomyRequest.controlOperations = force ? SharedAutonomyRequest::passToPoint : SharedAutonomyRequest::playBallKickToPoint;
    if((oldSpeed.translation - motionRequest.walkSpeed.translation).norm() > directionThreshold
       || (lastButtons & 1 << kickButton) == 0)
      direction = (Pose2f(theRobotPose.rotation) * motionRequest.walkSpeed.translation).angle();
    theSharedAutonomyRequest.targetPose.translation = theRobotPose * theBallModel.estimate.position + Pose2f(direction)
                                                      * Vector2f(5000.f * oldSpeed.translation.squaredNorm(), 0.f);
  }
  else if(motionRequest.motion == MotionRequest::stand)
    theSharedAutonomyRequest.controlOperations = motionRequest.standHigh ? SharedAutonomyRequest::standHigh : SharedAutonomyRequest::stand;
  else if(motionRequest.motion == MotionRequest::playDead)
    theSharedAutonomyRequest.controlOperations = SharedAutonomyRequest::sit;
  else
  {
    theSharedAutonomyRequest.controlOperations = SharedAutonomyRequest::walkAtRelativeSpeed;
    theSharedAutonomyRequest.targetPose = motionRequest.walkSpeed;
  }

  if(joystickState.pressed(scoreButton) && (lastButtons & 1 << scoreButton) == 0)
    theSharedAutonomyRequest.allowGoalKicks ^= true;

  theSharedAutonomyRequest.headRequest.manual = headMotionRequest.pan != 0_deg || headMotionRequest.tilt != tiltAxis.offset;
  theSharedAutonomyRequest.headRequest.angle = Vector2a(headMotionRequest.pan, headMotionRequest.tilt);
  theSharedAutonomyRequest.isValid = true;
}
