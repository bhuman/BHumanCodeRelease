/**
 * @file ArmContactModelProvider.cpp
 *
 * Implementation of class ArmContactModelProvider.
 * @author <a href="mailto:lrust@uni-bremen.de">Lukas Rust</a>
 */

#include "ArmContactModelProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Constants.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(ArmContactModelProvider, sensing);

void ArmContactModelProvider::decideArmMovementDone(ArmContactModel& model)
{
  for(int arm = 0; arm < 2; arm++)
  {
    bool change = model.status[arm].armOnBack != armOnBackLastFrame[arm];
    if(change || model.status[arm].armOnBack)
      adjustingHandCounter[arm] = 1;
    if(adjustingHandCounter[arm] < 150 && adjustingHandCounter[arm] > 0)
    {
      adjustingHand[arm] = true;
      adjustingHandCounter[arm]++;
    }
    if(adjustingHandCounter[arm] >= 150)
    {
      adjustingHand[arm] = false;
      adjustingHandCounter[arm] = 0;
    }
    armOnBackLastFrame[arm] = model.status[arm].armOnBack;
  }
}
Vector3f ArmContactModelProvider::calculateActualHandPosition(bool left)
{
  RobotModel actual(theJointAngles, theRobotDimensions, theMassCalibration);
  if(left)
    return actual.limbs[Limbs::wristLeft].translation;
  else
    return actual.limbs[Limbs::wristRight].translation;
}

Vector2f ArmContactModelProvider::calcXOffsets(bool armOnBack)
{
  Vector2f result;
  if(armOnBack)
  {
    result[0] = -3.f;
    result[1] = -3.f;
  }
  else
  {
    result[0] = 0.f;
    result[1] = 0.f;
  }
  return result;
}

Vector2f ArmContactModelProvider::calcYOffsets(bool armOnBack)
{
  Vector2f result;
  bool moving = std::abs(theMotionInfo.speed.translation.x()) > 0.03 ||
                std::abs(theMotionInfo.speed.translation.y()) > 0.03 ||
                std::abs(theMotionInfo.speed.rotation) >= 1_deg;
  if(!armOnBack && !moving)
  {
    result[0] = 3.f;
    result[1] = -3.f;
  }
  if(!armOnBack && moving)
  {
    result[0] = 1.5f;
    result[1] = -1.5f;
  }
  if(armOnBack)
  {
    result[0] = -3.f;
    result[1] = 3.f;
  }
  return result;
}

void ArmContactModelProvider::determinePushDirection(ArmContactModel& model)
{
  for(int arm = 0; arm < 2; arm++)
  {
    float xErrorThreshold;
    float yErrorThreshold;
    calcWalkDirectionFactor();
    if(model.status[arm].armOnBack)
    {
      xErrorThreshold = xErrorThresholdBase;
      yErrorThreshold = yErrorThresholdBase;
    }
    else
    {
      xErrorThreshold = xErrorThresholdBase + xErrorThresholdExtension * directionFactor.x();
      yErrorThreshold = yErrorThresholdBase + yErrorThresholdExtension * directionFactor.y();
    }
    model.status[arm].directionMap[ArmContactModel::backward] = dimmedError[arm].x() < -xErrorThreshold;
    model.status[arm].directionMap[ArmContactModel::forward] = dimmedError[arm].x() > xErrorThreshold;
    model.status[arm].directionMap[ArmContactModel::left] = dimmedError[arm].y() > yErrorThreshold;
    model.status[arm].directionMap[ArmContactModel::right] = dimmedError[arm].y() < -yErrorThreshold;

    if(model.status[arm].directionMap[ArmContactModel::backward] || model.status[arm].directionMap[ArmContactModel::forward] || model.status[arm].directionMap[ArmContactModel::left] || model.status[arm].directionMap[ArmContactModel::right])
    {
      model.status[arm].duration++;
      if(model.status[arm].duration >= minimalContactDuration)
      {
        if(!model.status[arm].contact && SystemCall::getMode() == SystemCall::physicalRobot && theFrameInfo.getTimeSince(lastArmSoundTimestamp) > soundWaitTime)
        {
          //SystemCall::playSound("arm.wav");
          ANNOTATION("ArmContactModelProvider", "Arm Contact");
          lastArmSoundTimestamp = theFrameInfo.time;
        }
        model.status[arm].contact = true;
      }
      if(std::abs(dimmedError[arm].x()) > std::abs(dimmedError[arm].y()))
        model.status[arm].pushDirection = dimmedError[arm].x() < 0.f ? ArmContactModel::backward : ArmContactModel::forward;
      else
        model.status[arm].pushDirection = dimmedError[arm].y() < 0.f ? ArmContactModel::left : ArmContactModel::right;
    }
    else
    {
      model.status[arm].duration = 0;
      model.status[arm].contact = false;
      model.status[arm].pushDirection = ArmContactModel::none;
    }
  }
}

void ArmContactModelProvider::calculateForce()
{
  Vector2f actualLeftHand = ArmContactModelProvider::calculateActualHandPosition(true).topRows(2);
  Vector2f requestedLeftHand = angleBuffer[0][frameDelay].topRows(2);
  Vector2f actualRightHand = ArmContactModelProvider::calculateActualHandPosition(false).topRows(2);
  Vector2f requestedRightHand = angleBuffer[1][frameDelay].topRows(2);

  Vector2f leftError;
  Vector2f rightError;
  leftError = actualLeftHand - requestedLeftHand;
  rightError = actualRightHand - requestedRightHand;
  errorBuffer[0].push_front(leftError);
  errorBuffer[1].push_front(rightError);
}

Vector3f ArmContactModelProvider::calculateRequestedHandPosition(bool left)
{
  RobotModel requested(theJointRequest, theRobotDimensions, theMassCalibration);
  if(left)
    return requested.limbs[Limbs::wristLeft].translation;
  else
    return requested.limbs[Limbs::wristRight].translation;
}

void ArmContactModelProvider::update(ArmContactModel& model)
{
  DECLARE_PLOT("module:ArmContactModelProvider:armErrorXLeft");
  DECLARE_PLOT("module:ArmContactModelProvider:armErrorXRight");
  DECLARE_PLOT("module:ArmContactModelProvider:armErrorYLeft");
  DECLARE_PLOT("module:ArmContactModelProvider:armErrorYRight");
  DECLARE_PLOT("module:ArmContactModelProvider:requestedSpeedX");
  DECLARE_PLOT("module:ArmContactModelProvider:requestedSpeedY");
  DECLARE_PLOT("module:ArmContactModelProvider:speedFactorX");
  DECLARE_PLOT("module:ArmContactModelProvider:speedFactorY");
  DECLARE_DEBUG_DRAWING("module:ArmContactModelProvider:leanedArmContact", "drawingOnField");

  if(theGameInfo.state == STATE_INITIAL ||
     theGameInfo.state == STATE_FINISHED ||
     theFallDownState.state == FallDownState::falling ||
     theFallDownState.state == FallDownState::fallen ||
     (theMotionInfo.executedPhase != MotionPhase::stand && theMotionInfo.executedPhase != MotionPhase::walk) ||
     !theGroundContactState.contact ||
     theRobotInfo.penalty != PENALTY_NONE)
  {
    reset(model);
    return;
  }
  model.status[0].armOnBack = theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion == ArmKeyFrameRequest::ArmKeyFrameId::back && theArmMotionRequest.armMotion[Arms::left] == ArmMotionRequest::ArmRequest::keyFrame;
  model.status[1].armOnBack = theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion == ArmKeyFrameRequest::ArmKeyFrameId::back && theArmMotionRequest.armMotion[Arms::right] == ArmMotionRequest::ArmRequest::keyFrame;

  angleBuffer[0].push_front(calculateRequestedHandPosition(true));
  angleBuffer[1].push_front(calculateRequestedHandPosition(false));

  // Make sure the buffer is filled
  if(theFrameInfo.getTimeSince(lastInActiveTimestamp) > frameDelay * Constants::motionCycleTime * 1000.f)
  {
    calculateForce();
    decideArmMovementDone(model);
    updateError(model);
    determinePushDirection(model);
  }

  CROSS("module:ArmContactModelProvider:leanedArmContact", calculateActualHandPosition(true).x() + theRobotPose.translation.x(), calculateActualHandPosition(true).y() + theRobotPose.translation.y(), 20, 20, Drawings::solidPen, ColorRGBA::green);
  CROSS("module:ArmContactModelProvider:leanedArmContact", calculateRequestedHandPosition(true).x() + theRobotPose.translation.x(), calculateRequestedHandPosition(true).y() + theRobotPose.translation.y(), 20, 20, Drawings::solidPen, ColorRGBA::red);
  PLOT("module:ArmContactModelProvider:armErrorXLeft", dimmedError[0].x());
  PLOT("module:ArmContactModelProvider:armErrorXRight", dimmedError[1].x());
  PLOT("module:ArmContactModelProvider:armErrorYLeft", dimmedError[0].y());
  PLOT("module:ArmContactModelProvider:armErrorYRight", dimmedError[1].y());
  PLOT("module:ArmContactModelProvider:requestedSpeedX", handSpeed[1].x());
  PLOT("module:ArmContactModelProvider:requestedSpeedY", handSpeed[1].y());
  PLOT("module:ArmContactModelProvider:speedFactorX", speedFactor.x());
  PLOT("module:ArmContactModelProvider:speedFactorY", speedFactor.y());
}

void ArmContactModelProvider::updateError(ArmContactModel& model)
{
  for(int arm = 0; arm < 2; arm++)
  {
    xOffsets = calcXOffsets(model.status[arm].armOnBack);
    yOffsets = calcYOffsets(model.status[arm].armOnBack);
    calcCorrectionFactor(arm == 0);
    dimmedError[arm].x() = (errorBuffer[arm].average().x() + xOffsets[arm]) * speedFactor.x();
    dimmedError[arm].y() = (errorBuffer[arm].average().y() + yOffsets[arm]) * speedFactor.y();
    if(adjustingHand[arm])
    {
      dimmedError[arm].x() = 0.f;
      dimmedError[arm].y() = 0.f;
    }
  }
}

void ArmContactModelProvider::reset(ArmContactModel& model)
{
  for(size_t i = 0; i < errorBufferSize; i++)
  {
    errorBuffer[0].push_front(Vector2f(0.f, 0.f));
    errorBuffer[1].push_front(Vector2f(0.f, 0.f));
    angleBuffer[0].push_front(Vector3f(0.f, 0.f, 0.f));
    angleBuffer[1].push_front(Vector3f(0.f, 0.f, 0.f));
  }
  lastHandPosRequested[0] = Vector2f(0.f, 0.f);
  lastHandPosRequested[1] = Vector2f(0.f, 0.f);
  model.status[0].contact = false;
  model.status[0].duration = 0;
  model.status[0].pushDirection = ArmContactModel::none;
  model.status[0].directionMap[ArmContactModel::forward] = false;
  model.status[0].directionMap[ArmContactModel::backward] = false;
  model.status[0].directionMap[ArmContactModel::left] = false;
  model.status[0].directionMap[ArmContactModel::right] = false;
  model.status[0].timeOfLastContact = 0;
  model.status[1].contact = false;
  model.status[1].duration = 0;
  model.status[1].pushDirection = ArmContactModel::none;
  model.status[1].directionMap[ArmContactModel::forward] = false;
  model.status[1].directionMap[ArmContactModel::backward] = false;
  model.status[1].directionMap[ArmContactModel::left] = false;
  model.status[1].directionMap[ArmContactModel::right] = false;
  model.status[1].timeOfLastContact = 0;
  lastInActiveTimestamp = theFrameInfo.time;
}

void ArmContactModelProvider::calcCorrectionFactor(bool left)
{
  int arm;
  if(left)
  {
    arm = 0;
    handPosRequested[arm] = flattenSpeed(angleBuffer[0]);
  }
  else
  {
    arm = 1;
    handPosRequested[arm] = flattenSpeed(angleBuffer[1]);
  }

  Vector2f handSpeedRequested = (handPosRequested[arm] - lastHandPosRequested[arm]) / Constants::motionCycleTime;
  if(handSpeedRequested.x() <= 500.f)
    speedFactor.x() = std::pow(0.85f, std::abs(handSpeedRequested.x()) / 40.f);
  if(handSpeedRequested.y() <= 45.f)
    speedFactor.y() = std::pow(0.94f, std::abs(handSpeedRequested.y()) / 19.f); //0.58^ (speed/10)

  if(std::abs(handSpeedRequested.x()) > 500.f)
    speedFactor.x() = 0.f;
  if(std::abs(handSpeedRequested.y()) > 200.f)
    speedFactor.y() = 0.f;

  lastHandPosRequested[arm] = handPosRequested[arm];
  handSpeed[arm] = handSpeedRequested;
}

Vector2f ArmContactModelProvider::flattenSpeed(RingBuffer<Vector3f, frameBufferSize>& angleBuffer)
{
  return (angleBuffer[frameDelay - 2].topRows(2) + angleBuffer[frameDelay - 1].topRows(2) + angleBuffer[frameDelay].topRows(2) + angleBuffer[frameDelay + 1].topRows(2) + angleBuffer[frameDelay + 2].topRows(2)) / 5.f;
}

void ArmContactModelProvider::calcWalkDirectionFactor()
{
  float total = std::abs(theMotionInfo.speed.translation.x()) + std::abs(theMotionInfo.speed.translation.y());
  if(total == 0.f)
  {
    directionFactor.x() = 0.f;
    directionFactor.y() = 0.f;
  }
  else
  {
    directionFactor.x() = sinf(abs(theMotionInfo.speed.translation.x()) / total * pi_2);
    directionFactor.y() = sinf(abs(theMotionInfo.speed.translation.y()) / total * pi_2);
  }
}
