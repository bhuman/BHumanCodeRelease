/**
 * @file JointPlayTranslationProvider.h
 *
 * @author Philip Reichenberg
 */

#include "JointPlayTranslationProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(JointPlayTranslationProvider);

JointPlayTranslationProvider::JointPlayTranslationProvider()
{
  FOREACH_ENUM(Joints::Joint, joint)
  {
    this->offsetFilter[joint].lowPassFactor = factor;
    this->offsetFilter[joint].fastFactor = fastFactor;
  }
}

void JointPlayTranslationProvider::update(JointPlayTranslation& theJointPlayTranslation)
{
  FOREACH_ENUM(Joints::Joint, joint)
  {
    // 1. Calculate the offset
    JointAnglePred::Joint predJoint;
    const bool usePred = convertToPredictJoint(joint, predJoint) && SystemCall::getMode() != SystemCall::simulatedRobot;
    Angle currentOffset =
      theJointAngles.angles[joint] - theJointPlay.jointState[joint].lastExecutedRequest;
    if(usePred)
    {
      Angle useRequest = theJointRequest.angles[joint];
      const bool isHipPitch = joint == Joints::lHipPitch || joint == Joints::rHipPitch;
      //if(isHipPitch)
      //  useRequest = theJointPlay.jointState[joint].lastExecutedRequest;
      Angle predictedOffset =
        theJointAnglePred.angles[predJoint] - useRequest;

      if(isHipPitch || std::abs(currentOffset) > std::abs(predictedOffset))
        currentOffset = predictedOffset;
    }

    // 2. limit the offset based on the measured joint velocity
    const Rangea movementOffset(std::min(-theJointPlay.jointState[joint].velocity, 0_deg), std::max(-theJointPlay.jointState[joint].velocity, 0_deg));
    currentOffset -= movementOffset.limit(currentOffset);
    offsetFilter[joint].update(currentOffset);

    // 3. write the offset in a range, to make usage easier
    theJointPlayTranslation.jointOffset[joint].min = std::min(0.f, offsetFilter[joint].currentValue);
    theJointPlayTranslation.jointOffset[joint].max = std::max(0.f, offsetFilter[joint].currentValue);
    theJointPlayTranslation.jointPlayState[joint].offset = offsetFilter[joint].currentValue;

    // 4. calculate the actual joint play
    currentOffset =
      theJointAngles.angles[joint] - theJointPlay.jointState[joint].requestBoundary.limit(theJointAngles.angles[joint]);

    if(usePred)
    {
      Angle predictedOffset =
        theJointAnglePred.angles[predJoint] - theJointPlay.jointState[joint].requestBoundary.limit(theJointAnglePred.angles[predJoint]);

      if(std::abs(currentOffset) > std::abs(predictedOffset))
        currentOffset = predictedOffset;
    }

    currentOffset -= movementOffset.limit(currentOffset);
    theJointPlayTranslation.jointPlayState[joint].play = currentOffset;
  }
}

bool JointPlayTranslationProvider::convertToPredictJoint(const Joints::Joint joint, JointAnglePred::Joint& predJoint)
{
  if(joint < Joints::firstLeftLegJoint)
    return false;

  switch(joint)
  {
    case Joints::lHipYawPitch:
      predJoint = JointAnglePred::hipYawPitch;
      break;
    case Joints::lHipPitch:
      predJoint = JointAnglePred::lHipPitch;
      break;
    case Joints::lKneePitch:
      predJoint = JointAnglePred::lKneePitch;
      break;
    case Joints::lAnklePitch:
      predJoint = JointAnglePred::lAnklePitch;
      break;
    case Joints::lHipRoll:
      predJoint = JointAnglePred::lHipRoll;
      break;
    case Joints::lAnkleRoll:
      predJoint = JointAnglePred::lAnkleRoll;
      break;
    case Joints::rHipYawPitch:
      predJoint = JointAnglePred::hipYawPitch;
      break;
    case Joints::rHipPitch:
      predJoint = JointAnglePred::rHipPitch;
      break;
    case Joints::rKneePitch:
      predJoint = JointAnglePred::rKneePitch;
      break;
    case Joints::rAnklePitch:
      predJoint = JointAnglePred::rAnklePitch;
      break;
    case Joints::rHipRoll:
      predJoint = JointAnglePred::rHipRoll;
      break;
    case Joints::rAnkleRoll:
      predJoint = JointAnglePred::rAnkleRoll;
      break;
    default:
      return false;
  }

  return true;
}
