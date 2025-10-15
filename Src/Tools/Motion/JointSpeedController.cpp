/**
 * @file JointSpeedController.cpp
 * @author Philip Reichenberg
 */

#include "JointSpeedController.h"
#include "Framework/Settings.h"
#include "MathBase/BHMath.h"
#include "Streaming/Global.h"

JointSpeedController::JointSpeedController(const ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const Angle turn)
{
  reset(currentJoints, turn);
}

void JointSpeedController::update(const SpeedControlParams& scp, const float currentPitchPosition,
                                  const float currentRollPosition, const bool isLeftPhase, const Angle gyroY)
{
  auto getMaxSpeed = [&](const Joints::Joint joint)
  {
    switch(joint)
    {
      case Joints::lAnklePitch:
      case Joints::rAnklePitch:
        return scp.rotationSpeedAnklePitch;
      case Joints::lHipPitch:
      case Joints::rHipPitch:
        return scp.rotationSpeedHip;
      case Joints::lKneePitch:
      case Joints::rKneePitch:
        return scp.rotationSpeedKnee;
      case Joints::lHipYawPitch:
      case Joints::rHipYawPitch:
        return scp.rotationSpeedHYP;
      case Joints::lHipRoll:
      case Joints::rHipRoll:
      case Joints::lAnkleRoll:
      case Joints::rAnkleRoll:
        return scp.rotationSpeedRoll;
      default:
        return Rangea(0, 0);
    }
  };

  const float ratioRoll = mapToRange(currentRollPosition, scp.rollRatio.min, scp.rollRatio.max, 0.f, 1.f);
  float ratioForward = mapToRange(currentPitchPosition, scp.pitchRatioForward.min, scp.pitchRatioForward.max, 0.f, 1.f);
  float ratioBackward = mapToRange(currentPitchPosition, scp.pitchRatioBackward.min, scp.pitchRatioBackward.max, 0.f, 1.f);
  ratioBackward = std::min(rotationErrorRatioBackward, ratioBackward);
  ratioForward = std::max(ratioForward, 1.f - rotationErrorRatioForward);

  FOREACH_ENUM(Joints::Joint, joint)
  {
    // Roll Cases
    if(joint > Joints::firstLegJoint)
    {
      if(joint == Joints::rHipRoll || joint == Joints::rAnkleRoll || joint == Joints::lHipRoll || joint == Joints::lAnkleRoll)    // Roll Case
      {
        const Rangea& useMaxSpeed = getMaxSpeed(joint);
        // Left is support -> moving leg to the left is always allowed (robot walks right)
        // Right is support -> moving leg to the right is always allowed (robot walks to the left)
        this->at(joint).max += (joint == Joints::lHipRoll || joint == Joints::rAnkleRoll) ? useMaxSpeed.max : Angle(useMaxSpeed.max * (1.f - ratioRoll) + useMaxSpeed.min * ratioRoll);
        this->at(joint).min -= (joint == Joints::lAnkleRoll || joint == Joints::rHipRoll) ? useMaxSpeed.max : Angle(useMaxSpeed.max * (1.f - ratioRoll) + useMaxSpeed.min * ratioRoll);
      }

      // Pitch Cases
      else if((!isLeftPhase && joint < Joints::firstRightLegJoint) || (isLeftPhase && joint > Joints::firstRightLegJoint))
      {
        const Rangea& useMaxSpeed = getMaxSpeed(joint);
        const Angle adjustedForMax = useMaxSpeed.limit(std::max(useMaxSpeed.min, Angle(gyroY * Global::getSettings().motionCycleTime)));
        const Angle adjustedForMin = useMaxSpeed.limit(std::max(useMaxSpeed.min, Angle(-gyroY * Global::getSettings().motionCycleTime)));
        this->at(joint).max += useMaxSpeed.max * ratioBackward + adjustedForMax * (1.f - ratioBackward);
        this->at(joint).min -= adjustedForMin * ratioForward + useMaxSpeed.max * (1.f - ratioForward);
      }
    }
  }
}

void JointSpeedController::apply(ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const bool isLeftPhase)
{
  for(std::size_t joint = (!isLeftPhase ? Joints::firstLegJoint : Joints::firstRightLegJoint); joint < (!isLeftPhase ? Joints::firstRightLegJoint : Joints::numOfJoints); joint++) // Skip all joints != legs
  {
    if(joint == Joints::lHipYawPitch ||
       joint == Joints::rHipYawPitch)
      continue;
    currentJoints[joint] = this->at(joint).limit(currentJoints[joint]);
  }
  if(isLeftPhase)
  {
    currentJoints[Joints::lHipRoll] = this->at(Joints::lHipRoll).limit(currentJoints[Joints::lHipRoll]);
    currentJoints[Joints::lAnkleRoll] = this->at(Joints::lAnkleRoll).limit(currentJoints[Joints::lAnkleRoll]);
  }
  else
  {
    currentJoints[Joints::rHipRoll] = this->at(Joints::rHipRoll).limit(currentJoints[Joints::rHipRoll]);
    currentJoints[Joints::rAnkleRoll] = this->at(Joints::rAnkleRoll).limit(currentJoints[Joints::rAnkleRoll]);
  }
}

void JointSpeedController::applySoleError(const Angle rotationErrorMeasured, const SpeedControlParams& scp, const float hardwareValue)
{
  const Rangea useRotationErrorRatioForward(scp.rotationErrorRatioForwardGood.min * hardwareValue + scp.rotationErrorRatioForwardWornOut.min * (1.f - hardwareValue), scp.rotationErrorRatioForwardGood.max * hardwareValue + scp.rotationErrorRatioForwardWornOut.max * (1.f - hardwareValue));

  rotationErrorRatioForward = mapToRange(rotationErrorMeasured, useRotationErrorRatioForward.min, useRotationErrorRatioForward.max, 0_deg, Angle(1.f));
  rotationErrorRatioBackward = 1.f - mapToRange(rotationErrorMeasured, scp.rotationErrorRatioBackward.min, scp.rotationErrorRatioBackward.max, 0_deg, Angle(1.f));
}

void JointSpeedController::reset(const ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const Angle turn)
{
  FOREACH_ENUM(Joints::Joint, joint)
  {
    this->at(joint).min = currentJoints[joint];
    this->at(joint).max = currentJoints[joint];
  }

  turnRange.min = turn;
  turnRange.max = turn;

  rotationErrorRatioForward = 1.f;
  rotationErrorRatioBackward = 1.f;
}

void JointSpeedController::updateZRotation(const SpeedControlParams& scp, const Angle rotationError, const float currentPitchPosition, const float currentPitchPositionTorso, const float hardwareValue, const Angle gyroY)
{
  const Rangea& useMaxSpeed = scp.rotationSpeedHYP;
  float ratioForward = std::max(mapToRange(currentPitchPositionTorso, scp.pitchRatioForward.min, scp.pitchRatioForward.max, 0.f, 1.f),
                                mapToRange(currentPitchPosition, scp.pitchRatioForward.min, scp.pitchRatioForward.max, 0.f, 1.f));
  float ratioBackward = std::min(mapToRange(currentPitchPositionTorso, scp.pitchRatioBackward.min, scp.pitchRatioBackward.max, 0.f, 1.f),
                                 mapToRange(currentPitchPosition, scp.pitchRatioBackward.min, scp.pitchRatioBackward.max, 0.f, 1.f));

  const Rangea useRotationErrorRatioForward(scp.rotationErrorRatioForwardGood.min * hardwareValue + scp.rotationErrorRatioForwardWornOut.min * (1.f - hardwareValue), scp.rotationErrorRatioForwardGood.max * hardwareValue + scp.rotationErrorRatioForwardWornOut.max * (1.f - hardwareValue));
  ratioForward = std::max(ratioForward, 1.f - mapToRange(rotationError, useRotationErrorRatioForward.min, useRotationErrorRatioForward.max, 0_deg, Angle(1.f)));
  ratioBackward = std::min(ratioBackward, 1.f - mapToRange(rotationError, scp.rotationErrorRatioBackward.min, scp.rotationErrorRatioBackward.max, 0_deg, Angle(1.f)));

  // Use half of the frame time. Otherwise 25 deg/s rotation for the gyro is already enough to reduce the min turn speed down to 0.
  const Angle negativeMin = gyroY >= 0.f ? useMaxSpeed.min : Angle(std::max(0.f, useMaxSpeed.min + gyroY * Global::getSettings().motionCycleTime * 0.5f));
  const Angle positiveMin = gyroY <= 0.f ? useMaxSpeed.min : Angle(std::max(0.f, useMaxSpeed.min - gyroY * Global::getSettings().motionCycleTime * 0.5f));
  turnRange.min -= useMaxSpeed.max * ratioBackward + negativeMin * (1.f - ratioBackward);
  turnRange.max += positiveMin * ratioForward + useMaxSpeed.max * (1.f - ratioForward);
}

void JointSpeedController::clipZRotation(Angle& turn)
{
  turn = turnRange.limit(turn);
}
