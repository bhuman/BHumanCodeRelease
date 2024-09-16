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
    const bool usePred = theJointAnglePred.isValid && theJointAnglePred.angles[joint] != SensorData::ignore && SystemCall::getMode() != SystemCall::simulatedRobot;
    Angle currentOffset = theJointAngles.angles[joint] - theJointPlay.jointState[joint].lastExecutedRequest;
    if(usePred)
    {
      Angle useRequest = theJointRequest.angles[joint];
      const bool isHipPitch = joint == Joints::lHipPitch || joint == Joints::rHipPitch;
      //if(isHipPitch)
      //  useRequest = theJointPlay.jointState[joint].lastExecutedRequest;
      Angle predictedOffset = theJointAnglePred.angles[joint] - useRequest;

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
        theJointAnglePred.angles[joint] - theJointPlay.jointState[joint].requestBoundary.limit(theJointAnglePred.angles[joint]);

      if(std::abs(currentOffset) > std::abs(predictedOffset))
        currentOffset = predictedOffset;
    }

    currentOffset -= movementOffset.limit(currentOffset);
    theJointPlayTranslation.jointPlayState[joint].play = currentOffset;
  }
}
