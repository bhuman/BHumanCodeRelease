/**
 * @file JointSpeedRegulator.cpp
 * @author Philip Reichenberg
 */

#include "JointSpeedRegulator.h"
#include "MathBase/BHMath.h"

JointSpeedRegulator::JointSpeedRegulator(const ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const Angle turn)
{
  reset(currentJoints, turn);
}

void JointSpeedRegulator::update(const SpeedRegulatorParams& srp, const float currentPitchPosition,
                                 const float currentRollPosition, const bool isLeftPhase)
{
  auto getMaxSpeed = [&](const Joints::Joint joint)
  {
    switch(joint)
    {
      case Joints::lAnklePitch:
      case Joints::rAnklePitch:
        return srp.rotationSpeedAnklePitch;
      case Joints::lHipPitch:
      case Joints::rHipPitch:
        return srp.rotationSpeedHip;
      case Joints::lKneePitch:
      case Joints::rKneePitch:
        return srp.rotationSpeedKnee;
      case Joints::lHipYawPitch:
      case Joints::rHipYawPitch:
        return srp.rotationSpeedHYP;
      case Joints::lHipRoll:
      case Joints::rHipRoll:
      case Joints::lAnkleRoll:
      case Joints::rAnkleRoll:
        return srp.rotationSpeedRoll;
      default:
        return Rangea(0, 0);
    }
  };

  const float ratioRoll = mapToRange(currentRollPosition, srp.rollRatio.min, srp.rollRatio.max, 0.f, 1.f);
  float ratioForward = mapToRange(currentPitchPosition, srp.pitchRatioForward.min, srp.pitchRatioForward.max, 0.f, 1.f);
  float ratioBackward = mapToRange(currentPitchPosition, srp.pitchRatioBackward.min, srp.pitchRatioBackward.max, 0.f, 1.f);
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
        this->at(joint).max += useMaxSpeed.max * ratioBackward + useMaxSpeed.min * (1.f - ratioBackward);
        this->at(joint).min -= useMaxSpeed.min * ratioForward + useMaxSpeed.max * (1.f - ratioForward);
      }
    }
  }
}

void JointSpeedRegulator::apply(ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const bool isLeftPhase)
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

void JointSpeedRegulator::applySoleError(const Angle rotationErrorMeasured, const SpeedRegulatorParams& speedRegulatorParams, const float hardwareValue)
{
  const SpeedRegulatorParams& srp = speedRegulatorParams;

  const Rangea useRotationErrorRatioForward(srp.rotationErrorRatioForwardGood.min * hardwareValue + srp.rotationErrorRatioForwardWornOut.min * (1.f - hardwareValue), srp.rotationErrorRatioForwardGood.max * hardwareValue + srp.rotationErrorRatioForwardWornOut.max * (1.f - hardwareValue));

  rotationErrorRatioForward = mapToRange(rotationErrorMeasured, useRotationErrorRatioForward.min, useRotationErrorRatioForward.max, 0_deg, Angle(1.f));
  rotationErrorRatioBackward = 1.f - mapToRange(rotationErrorMeasured, srp.rotationErrorRatioBackward.min, srp.rotationErrorRatioBackward.max, 0_deg, Angle(1.f));
}

void JointSpeedRegulator::reset(const ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const Angle turn)
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

void JointSpeedRegulator::updateZRotation(const SpeedRegulatorParams& srp, const Angle rotationError, const float currentPitchPosition, const float currentPitchPositionTorso, const float hardwareValue)
{
  const Rangea& useMaxSpeed = srp.rotationSpeedHYP;
  float ratioForward = std::max(mapToRange(currentPitchPositionTorso, srp.pitchRatioForward.min, srp.pitchRatioForward.max, 0.f, 1.f),
                                mapToRange(currentPitchPosition, srp.pitchRatioForward.min, srp.pitchRatioForward.max, 0.f, 1.f));
  float ratioBackward = std::min(mapToRange(currentPitchPositionTorso, srp.pitchRatioBackward.min, srp.pitchRatioBackward.max, 0.f, 1.f),
                                 mapToRange(currentPitchPosition, srp.pitchRatioBackward.min, srp.pitchRatioBackward.max, 0.f, 1.f));

  const Rangea useRotationErrorRatioForward(srp.rotationErrorRatioForwardGood.min * hardwareValue + srp.rotationErrorRatioForwardWornOut.min * (1.f - hardwareValue), srp.rotationErrorRatioForwardGood.max * hardwareValue + srp.rotationErrorRatioForwardWornOut.max * (1.f - hardwareValue));
  ratioForward = std::max(ratioForward, 1.f - mapToRange(rotationError, useRotationErrorRatioForward.min, useRotationErrorRatioForward.max, 0_deg, Angle(1.f)));
  ratioBackward = std::min(ratioBackward, 1.f - mapToRange(rotationError, srp.rotationErrorRatioBackward.min, srp.rotationErrorRatioBackward.max, 0_deg, Angle(1.f)));

  turnRange.min -= useMaxSpeed.max * ratioBackward + useMaxSpeed.min * (1.f - ratioBackward);
  turnRange.max += useMaxSpeed.min * ratioForward + useMaxSpeed.max * (1.f - ratioForward);
}

void JointSpeedRegulator::regulateZRotation(Angle& turn)
{
  turn = turnRange.limit(turn);
}
