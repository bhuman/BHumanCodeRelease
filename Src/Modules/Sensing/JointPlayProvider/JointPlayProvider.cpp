/**
 * @file JointPlayProvider.h
 *  This modules analyzes the joint play and returns information about how bad the hardware state is.
 * @author Philip Reichenberg
 */

#include "JointPlayProvider.h"
#include "Debugging/Annotation.h"
#include "Debugging/Plot.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(JointPlayProvider);

JointPlayProvider::JointPlayProvider()
{
  for(std::size_t i = 0; i < Joints::numOfJoints; i++)
  {
    jointPlayValue[i].push_front(0.f);
    jointPlayValue[i].push_front(0.f);
    bufferValue[i] = 0.f;
    // TODO is this even needed?
    bufferRequest[i].clear();
  }
}

void JointPlayProvider::update(JointPlay& theJointPlay)
{
  // 0. skip very first motion frame
  if(!skippedFirstFrame)
  {
    skippedFirstFrame = true;
    return;
  }

  // 1. Check the walk state
  const bool isWalkingNow = theMotionRequest.isWalking() && theMotionInfo.isMotion(MotionPhase::walk) && theGroundContactState.contact;
  if((isWalkingNow && !isWalking) || startWalkingTimestamp > theFrameInfo.time)
    startWalkingTimestamp = theFrameInfo.time;
  isWalking = isWalkingNow;
  const bool skipBuffer = theFrameInfo.getTimeSince(startWalkingTimestamp) < minWalkTime || !isWalking;
  if(!skipBuffer)
    timeSpendWalking += Constants::motionCycleTime;
  const float ratio = Rangef::ZeroOneRange().limit(timeSpendWalking / interpolateLowpassFilterTime);
  const float useLowPassFilterFactor = lowpassFilterFactor.min * ratio + lowpassFilterFactor.max * (1.f - ratio);
  Angle jointPlaySum = 0.f;
  const float minSpeedRatio = minForwardSpeed / maxForwardSpeed;
  const float speedRatio = theWalkStepData.stepDuration == 0.f ? 0.f : (theWalkStepData.stepTarget.translation.x() / theWalkStepData.stepDuration) / maxForwardSpeed;
  const Angle jointPlayOffset = (1.f - Rangef::ZeroOneRange().limit((speedRatio - minSpeedRatio) / (1.f - minSpeedRatio))) * jointPlayScalingWalkingSpeed;
  bool jointAbovePlaySoundValue = false;

  // 2. Filter
  FOREACH_ENUM(Joints::Joint, joint)
  {
    // Buffer joint request, because of motion delay
    bufferRequest[joint].push_front(theJointRequest.angles[joint]);
    if(!bufferRequest[joint].full())
    {
      theJointPlay.jointState[joint].requestBoundary.min = theJointPlay.jointState[joint].requestBoundary.max = theJointRequest.angles[joint];
      continue;
    }
    // Filter differences
    if(!skipBuffer)
    {
      const auto it = std::find_if(jointPlayList.begin(), jointPlayList.end(), [&joint](const JointPlayPair& other)
      {
        return other.joint == joint;
      });
      const Angle useJointPlayOffset = joint == Joints::lAnklePitch || joint == Joints::rAnklePitch ? jointPlayOffset : 0_deg;
      const Angle ref = it != jointPlayList.end() ? it->ref : 0_deg;
      const float ratio = it != jointPlayList.end() ? it->ratio : 0.f;
      bufferValue[joint] = bufferValue[joint] * (1.f - useLowPassFilterFactor) +
                           (std::abs(theJointAngles.angles[joint] - bufferRequest[joint].back()) - (ref - useJointPlayOffset)) * useLowPassFilterFactor;

      jointPlaySum += std::max(bufferValue[joint], 0_deg) * ratio;
    }
    const Angle clippedJoint = theJointPlay.jointState[joint].requestBoundary.limit(theJointAngles.angles[joint]);

    theJointPlay.jointState[joint].requestBoundary.min = std::min(bufferRequest[joint].back(), clippedJoint);
    theJointPlay.jointState[joint].requestBoundary.max = std::max(bufferRequest[joint].back(), clippedJoint);

    theJointPlay.jointState[joint].lastExecutedRequest = bufferRequest[joint].back();
    theJointPlay.jointState[joint].velocity = theJointAngles.angles[joint] - lastJointAngles.angles[joint];

    jointPlayValue[joint].push_front(std::abs(theJointPlay.jointState[joint].requestBoundary.limit(theJointAngles.angles[joint]) - theJointAngles.angles[joint]));
    if(joint > Joints::firstLegJoint)
    {
      const bool hasHighPlay = theMotionInfo.isMotion(MotionPhase::walk) && (std::abs(jointPlayValue[joint].front() - jointPlayValue[joint].back()) > minPlayForSound || jointPlayValue[joint].front() > minPlaySlowForSound);
      if(hasHighPlay)
        theJointPlay.jointState[joint].status =
          static_cast<JointPlay::JointStatus>(std::min(static_cast<unsigned>(JointPlay::numOfJointStatuss) - 1, static_cast<unsigned>(theJointPlay.jointState[joint].status) + 1));
      else
        theJointPlay.jointState[joint].status = JointPlay::allFine;
      if(hasHighPlay)
      {
        jointAbovePlaySoundValue |= hasHighPlay;
      }
    }
  }

  bool calcQuality = SystemCall::getMode() != SystemCall::logFileReplay;
  MODIFY("module:JointPlayProvider:calcQuality", calcQuality);
  if(!skipBuffer && calcQuality)
  {
    theJointPlay.qualityOfRobotHardware = 1.f - Rangef::ZeroOneRange().limit((jointPlaySum - jointPlayScaling.min) / jointPlayScaling.max);
    // If the robot did not walk much, assume lower values
    theJointPlay.qualityOfRobotHardware *= Rangef::ZeroOneRange().limit(timeSpendWalking / interpolateJointPlayValueWalkTime);
    theJointPlay.isCalibrated |= timeSpendWalking >= interpolateJointPlayValueWalkTime;
  }
  PLOT("module:JointPlayProvider:lhp", bufferValue[Joints::lHipPitch].toDegrees());
  PLOT("module:JointPlayProvider:lkp", bufferValue[Joints::lKneePitch].toDegrees());
  PLOT("module:JointPlayProvider:lap", bufferValue[Joints::lAnklePitch].toDegrees());
  PLOT("module:JointPlayProvider:rhp", bufferValue[Joints::rHipPitch].toDegrees());
  PLOT("module:JointPlayProvider:rkp", bufferValue[Joints::rKneePitch].toDegrees());
  PLOT("module:JointPlayProvider:rap", bufferValue[Joints::rAnklePitch].toDegrees());

  PLOT("module:JointPlayProvider:sum", jointPlaySum.toDegrees());
  PLOT("module:JointPlayProvider:qualityOfRobotHardware", theJointPlay.qualityOfRobotHardware);

  PLOT("module:JointPlayProvider:play:value:lHipYawPitch", jointPlayValue[Joints::lHipYawPitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:lHipRoll", jointPlayValue[Joints::lHipRoll].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:lHipPitch", jointPlayValue[Joints::lHipPitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:lKneePitch", jointPlayValue[Joints::lKneePitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:lAnklePitch", jointPlayValue[Joints::lAnklePitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:lAnkleRoll", jointPlayValue[Joints::lAnkleRoll].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:rHipRoll", jointPlayValue[Joints::rHipRoll].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:rHipPitch", jointPlayValue[Joints::rHipPitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:rKneePitch", jointPlayValue[Joints::rKneePitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:rAnklePitch", jointPlayValue[Joints::rAnklePitch].front().toDegrees());
  PLOT("module:JointPlayProvider:play:value:rAnkleRoll", jointPlayValue[Joints::rAnkleRoll].front().toDegrees());

  bool playSound = SystemCall::getMode() == SystemCall::physicalRobot;

  MODIFY("module:JointPlayProvider:sound", playSound);

  if(lastWarningTimestamp > theFrameInfo.time || // Time jumped backwards. Happens when replaying logs
     (theFrameInfo.time - lastFrameInfoTime) > (Constants::motionCycleTime * 1000.f * 1.5f)) // Time jump is a lot larger than it should be
    lastWarningTimestamp = theFrameInfo.time;

  if(playSound && theMotionInfo.isMotion(MotionPhase::walk) &&
     theFrameInfo.getTimeSince(lastWarningTimestamp) > warningSoundWaitTime &&
     jointAbovePlaySoundValue)
  {
    std::vector<Joints::Joint> damagedJoints = theJointPlay.getJointsWithSensorJump(JointPlay::damaged);
    std::vector<Joints::Joint> brokenJoints = theJointPlay.getJointsWithSensorJump(JointPlay::broken);
    std::string joints;
    for(Joints::Joint joint : damagedJoints)
    {
      joints += TypeRegistry::getEnumName(Joints::Joint(joint));
      joints += " ";
    }
    OUTPUT_ERROR("Joints with sensor jump " << joints);
    ANNOTATION("JointPlayProvider", "Joint with sensor jump " << joints);
    if(!brokenJoints.empty())
    {
      lastWarningTimestamp = theFrameInfo.time;
      joints = "";
      bool playSound = false;
      for(Joints::Joint joint : brokenJoints)
      {
        joints += TypeRegistry::getEnumName(Joints::Joint(joint));
        joints += " ";
        if(std::find(theDamageConfigurationBody.jointsToMuteSirenes.cbegin(), theDamageConfigurationBody.jointsToMuteSirenes.cend(), joint) == theDamageConfigurationBody.jointsToMuteSirenes.cend())
          playSound = true;
      }
      ANNOTATION("JointPlayProvider", "Joint with high play " << joints);

      if(playSound)
      {
        SystemCall::playSound("sirene.wav", true);
        SystemCall::say((std::string("Joints may be broken ") + joints).c_str(), true);
      }
    }
  }

  lastFrameInfoTime = theFrameInfo.time;
  lastJointAngles = theJointAngles;
}
