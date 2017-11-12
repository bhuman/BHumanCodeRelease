/*
 * @file FallEngine.cpp
 * A minimized motion engine for falling.
 * @author Bernd Poppinga
 */

#include "FallEngine.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Platform/SystemCall.h"

void FallEngine::update(FallEngineOutput& output)
{
  // when it wasn't active check if it should be activated
  if(!output.active)
  {
    output.active = theFallDownState.state == FallDownState::falling && specialActionFall() && theMotionInfo.motion != MotionRequest::getUp && theMotionRequest.motion != MotionRequest::getUp;

    if(output.active)
    {
      calcDirection(output);
      lastJointsBeforeFall = theJointAngles;
      output.angles = theJointAngles.angles;
      output.startTime = theFrameInfo.time;
    }
  }
  // check if we want to leave

  // leave if fallen
  else if(theFallDownState.state == FallDownState::fallen)
    waitingForGetup = true;

  // leave when upright again
  else if(theFallDownState.state == FallDownState::squatting ||
          theFallDownState.state == FallDownState::upright)
    output.active = false;

  // if too long active consider leaving
  else if(theFrameInfo.getTimeSince(output.startTime) > waitAfterFalling)
  {
    if(theInertialData.angle.y() < forwardThreshold && theInertialData.angle.y() > backwardsThreshold)
    {
      JointRequest jq;
      MotionUtilities::stand(jq);
      MotionUtilities::interpolate(output, 0.02f, 1., theJointRequest, theJointAngles, jq);
    }
    waitingForGetup = true;
  }

  if(output.active && waitingForGetup && theMotionRequest.motion == MotionRequest::getUp)
    output.active = false;

  if(output.active && !waitingForGetup)
  {
    safeFall(output);
    safeArms(output);
    centerHead(output);
    ASSERT(output.isValid());
  }
  else
  {
    waitingForGetup = false;
    headYawInSafePosition = false;
    headPitchInSafePosition = false;
    shoulderInSafePosition = false;
  }
}

void FallEngine::calcDirection(FallEngineOutput& output)
{
  output.fallingForward = theInertialData.angle.y() > forwardThreshold;
  output.fallingBackwards = theInertialData.angle.y() < backwardsThreshold;
}

void FallEngine::safeFall(FallEngineOutput& output) const
{
  for(int i = 0; i < Joints::numOfJoints; i++)
    output.stiffnessData.stiffnesses[i] = 30;

  if(output.fallingBackwards)
  {
    // Sit down to reduce the impact-force
    output.angles[Joints::lKneePitch] = 180_deg;
    output.angles[Joints::rKneePitch] = 180_deg;
    output.angles[Joints::lHipPitch] = -90_deg;
    output.angles[Joints::rHipPitch] = -90_deg;
    output.angles[Joints::lHipRoll] = 0_deg;
    output.angles[Joints::rHipRoll] = 0_deg;
    output.angles[Joints::lAnklePitch] = -45_deg;
    output.angles[Joints::rAnklePitch] = -45_deg;
    output.angles[Joints::lAnkleRoll] = 0_deg;
    output.angles[Joints::rAnkleRoll] = 0_deg;
  }

  if(output.fallingForward)
  {
    MotionUtilities::stand(output);
  }
}

void FallEngine::safeArms(FallEngineOutput& output) const
{
  if(output.fallingBackwards)
  {
    // Set angles
    if(theJointAngles.angles[Joints::lShoulderRoll] > 3_deg)
      output.angles[Joints::lShoulderPitch] = 90_deg;
    output.angles[Joints::lShoulderRoll] = 11_deg;
    output.angles[Joints::lElbowYaw] = -180_deg;
    output.angles[Joints::lElbowRoll] = 0_deg;
    output.angles[Joints::lWristYaw] = -45_deg;
    if(theJointAngles.angles[Joints::rShoulderRoll] < -3_deg)
      output.angles[Joints::rShoulderPitch] = 90_deg;
    output.angles[Joints::rShoulderRoll] = -11_deg;
    output.angles[Joints::rElbowYaw] = 180_deg;
    output.angles[Joints::rElbowRoll] = 0_deg;
    output.angles[Joints::rWristYaw] = 45_deg;
  }
}

void FallEngine::centerHead(FallEngineOutput& output)
{
  if(output.angles[Joints::headYaw] == JointAngles::off &&
     output.angles[Joints::headPitch] == JointAngles::off)
    return;

  output.angles[Joints::headYaw] = 0;
  output.angles[Joints::headPitch] = 0;
  if(output.fallingForward)
    output.angles[Joints::headPitch] = -0.4f;
  else if(output.fallingBackwards)
    output.angles[Joints::headPitch] = 0.3f;
  if(std::abs(theJointAngles.angles[Joints::headYaw]) > 0.1f && !headYawInSafePosition)
    output.stiffnessData.stiffnesses[Joints::headYaw] = 100;
  else
  {
    headYawInSafePosition = true;
    output.stiffnessData.stiffnesses[Joints::headYaw] = 25;
  }
  if(std::abs(theJointAngles.angles[Joints::headPitch] - output.angles[Joints::headPitch]) > 0.1f && !headPitchInSafePosition)
    output.stiffnessData.stiffnesses[Joints::headPitch] = 100;
  else
  {
    headPitchInSafePosition = true;
    output.stiffnessData.stiffnesses[Joints::headPitch] = 50;
  }
}

/**  in general the fallEngine shouldn't be triggered when a special action is executed. but there
 * are some cases when it should trigger. theese are handled here
 * @return true, if the MotionInfo is about a special action where no jump is intended
 */
bool FallEngine::specialActionFall() const
{
  return theMotionRequest.motion != MotionRequest::specialAction ||
         (theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) < noGameControllerThreshold &&
          theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh);
}

MAKE_MODULE(FallEngine, motionControl)
