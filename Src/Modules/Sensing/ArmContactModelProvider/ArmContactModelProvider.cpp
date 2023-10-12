/**
 * @file ArmContactModelProvider.cpp
 *
 * Implementation of class ArmContactModelProvider.
 * @author <a href="mailto:lrust@uni-bremen.de">Lukas Rust</a>
 */

#include "ArmContactModelProvider.h"
#include "Platform/SystemCall.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Plot.h"
#include "Math/Constants.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(ArmContactModelProvider);

void ArmContactModelProvider::decideArmMovementDone(ArmContactModel& model)
{
  FOREACH_ENUM(Arms::Arm, arm)
  {
    const bool change = model.status[arm].armOnBack != armOnBackLastFrame[arm];
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

Vector3f ArmContactModelProvider::calculateActualHandPosition(Arms::Arm arm) const
{
  return theRobotModel.limbs[arm == Arms::left ? Limbs::wristLeft : Limbs::wristRight].translation;
}

float ArmContactModelProvider::calcXOffset(bool armOnBack) const
{
  return armOnBack ? -3.f : 0.f;
}

float ArmContactModelProvider::calcYOffset(Arms::Arm arm, bool armOnBack) const
{
  if(armOnBack)
    return arm == Arms::left ? -3.f : 3.f;

  const bool moving = std::abs(theMotionInfo.speed.translation.x()) > 0.03f ||
                      std::abs(theMotionInfo.speed.translation.y()) > 0.03f ||
                      std::abs(theMotionInfo.speed.rotation) >= 1_deg;
  if(moving)
    return arm == Arms::left ? 1.5f : -1.5f;
  else
    return arm == Arms::left ? 3.f : -3.f;
}

void ArmContactModelProvider::determinePushDirection(ArmContactModel& model)
{
  const Vector2f directionFactor = calcWalkDirectionFactor();
  FOREACH_ENUM(Arms::Arm, arm)
  {
    float xErrorThreshold;
    float yErrorThreshold;
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
  const Vector2f actualLeftHand = calculateActualHandPosition(Arms::left).head<2>();
  const Vector2f requestedLeftHand = angleBuffer[Arms::left][frameDelay].head<2>();
  const Vector2f actualRightHand = calculateActualHandPosition(Arms::right).head<2>();
  const Vector2f requestedRightHand = angleBuffer[Arms::right][frameDelay].head<2>();

  const Vector2f leftError = actualLeftHand - requestedLeftHand;
  const Vector2f rightError = actualRightHand - requestedRightHand;
  errorBuffer[Arms::left].push_front(leftError);
  errorBuffer[Arms::right].push_front(rightError);
}

Vector3f ArmContactModelProvider::calculateRequestedHandPosition(Arms::Arm arm) const
{
  return requestedRobotModel.limbs[arm == Arms::left ? Limbs::wristLeft : Limbs::wristRight].translation;
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

  if(theGameState.isInitial() ||
     theGameState.isFinished() ||
     theGameState.isPenalized() ||
     theFallDownState.state == FallDownState::falling ||
     theFallDownState.state == FallDownState::fallen ||
     (theMotionInfo.executedPhase != MotionPhase::stand && theMotionInfo.executedPhase != MotionPhase::walk) ||
     !theGroundContactState.contact)
  {
    reset(model);
    return;
  }
  model.status[Arms::left].armOnBack = theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion == ArmKeyFrameRequest::ArmKeyFrameId::back && theArmMotionRequest.armMotion[Arms::left] == ArmMotionRequest::ArmRequest::keyFrame;
  model.status[Arms::right].armOnBack = theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion == ArmKeyFrameRequest::ArmKeyFrameId::back && theArmMotionRequest.armMotion[Arms::right] == ArmMotionRequest::ArmRequest::keyFrame;

  requestedRobotModel.setJointData(theJointRequest, theRobotDimensions, theMassCalibration);

  angleBuffer[Arms::left].push_front(calculateRequestedHandPosition(Arms::left));
  angleBuffer[Arms::right].push_front(calculateRequestedHandPosition(Arms::right));

  // Make sure the buffer is filled
  if(angleBuffer[Arms::left].full() && angleBuffer[Arms::right].full())
  {
    calculateForce();
    decideArmMovementDone(model);
    updateError(model);
    determinePushDirection(model);
  }

  CROSS("module:ArmContactModelProvider:leanedArmContact", calculateActualHandPosition(Arms::left).x(), calculateActualHandPosition(Arms::left).y(), 20, 20, Drawings::solidPen, ColorRGBA::green);
  CROSS("module:ArmContactModelProvider:leanedArmContact", calculateRequestedHandPosition(Arms::left).x(), calculateRequestedHandPosition(Arms::left).y(), 20, 20, Drawings::solidPen, ColorRGBA::red);
  PLOT("module:ArmContactModelProvider:armErrorXLeft", dimmedError[Arms::left].x());
  PLOT("module:ArmContactModelProvider:armErrorXRight", dimmedError[Arms::right].x());
  PLOT("module:ArmContactModelProvider:armErrorYLeft", dimmedError[Arms::left].y());
  PLOT("module:ArmContactModelProvider:armErrorYRight", dimmedError[Arms::right].y());
  PLOT("module:ArmContactModelProvider:requestedSpeedX", handSpeed[Arms::right].x());
  PLOT("module:ArmContactModelProvider:requestedSpeedY", handSpeed[Arms::right].y());
  PLOT("module:ArmContactModelProvider:speedFactorX", speedFactor.x());
  PLOT("module:ArmContactModelProvider:speedFactorY", speedFactor.y());
}

void ArmContactModelProvider::updateError(ArmContactModel& model)
{
  FOREACH_ENUM(Arms::Arm, arm)
  {
    calcCorrectionFactor(arm);
    if(adjustingHand[arm])
    {
      dimmedError[arm].x() = 0.f;
      dimmedError[arm].y() = 0.f;
    }
    else
    {
      const float xOffset = calcXOffset(model.status[arm].armOnBack);
      const float yOffset = calcYOffset(arm, model.status[arm].armOnBack);
      dimmedError[arm].x() = (errorBuffer[arm].average().x() + xOffset) * speedFactor.x();
      dimmedError[arm].y() = (errorBuffer[arm].average().y() + yOffset) * speedFactor.y();
    }
  }
}

void ArmContactModelProvider::reset(ArmContactModel& model)
{
  for(std::size_t i = 0; i < errorBufferSize; i++)
  {
    errorBuffer[Arms::left].push_front(Vector2f::Zero());
    errorBuffer[Arms::right].push_front(Vector2f::Zero());
  }
  angleBuffer[Arms::left].clear();
  angleBuffer[Arms::right].clear();
  lastHandPosRequested[Arms::left] = Vector2f::Zero();
  lastHandPosRequested[Arms::right] = Vector2f::Zero();
  for(ArmContactModel::ArmContact& status : model.status)
  {
    status.contact = false;
    status.duration = 0;
    status.pushDirection = ArmContactModel::none;
    status.directionMap[ArmContactModel::forward] = false;
    status.directionMap[ArmContactModel::backward] = false;
    status.directionMap[ArmContactModel::left] = false;
    status.directionMap[ArmContactModel::right] = false;
    status.timeOfLastContact = 0;
  }
}

void ArmContactModelProvider::calcCorrectionFactor(Arms::Arm arm)
{
  handPosRequested[arm] = flattenSpeed(angleBuffer[arm]);

  const Vector2f handSpeedRequested = (handPosRequested[arm] - lastHandPosRequested[arm]) / Constants::motionCycleTime;
  if(std::abs(handSpeedRequested.x()) <= 500.f)
    speedFactor.x() = std::pow(0.85f, std::abs(handSpeedRequested.x()) / 40.f);
  else
    speedFactor.x() = 0.f;

  if(std::abs(handSpeedRequested.y()) <= 200.f)
    speedFactor.y() = std::pow(0.94f, std::abs(handSpeedRequested.y()) / 19.f);
  else
    speedFactor.y() = 0.f;

  lastHandPosRequested[arm] = handPosRequested[arm];
  handSpeed[arm] = handSpeedRequested;
}

Vector2f ArmContactModelProvider::flattenSpeed(const RingBuffer<Vector3f, frameBufferSize>& angleBuffer) const
{
  return (angleBuffer[frameDelay - 2].head<2>() + angleBuffer[frameDelay - 1].head<2>() + angleBuffer[frameDelay].head<2>() + angleBuffer[frameDelay + 1].head<2>() + angleBuffer[frameDelay + 2].head<2>()) / 5.f;
}

Vector2f ArmContactModelProvider::calcWalkDirectionFactor() const
{
  const float total = std::abs(theMotionInfo.speed.translation.x()) + std::abs(theMotionInfo.speed.translation.y());
  if(total == 0.f)
    return Vector2f::Zero();
  return Vector2f(std::sin(std::abs(theMotionInfo.speed.translation.x()) / total * pi_2),
                  std::sin(std::abs(theMotionInfo.speed.translation.y()) / total * pi_2));
}
