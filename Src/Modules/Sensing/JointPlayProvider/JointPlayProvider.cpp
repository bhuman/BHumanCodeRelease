/**
 * @file JointPlayProvider.h
 *  This modules analyzes the joint play and returns information about how bad the hardware state is.
 * @author Philip Reichenberg
 */

#include "JointPlayProvider.h"

MAKE_MODULE(JointPlayProvider, sensing);

JointPlayProvider::JointPlayProvider()
{
  for(std::size_t i = 0; i < JointPlayTrack::numOfJointPlayTracks; i++)
  {
    bufferValue[i] = 0.f;
    // TODO is this even needed?
    bufferRequest[i].clear();
  }
}

void JointPlayProvider::update(JointPlay& theJointPlay)
{
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
  // 2. Filter
  FOREACH_ENUM(JointPlayTrack, joint)
  {
    // Buffer joint request, because of motion delay
    bufferRequest[joint].push_front(theJointRequest.angles[getJoint(joint)]);
    if(!bufferRequest[joint].full())
      continue;
    if(skipBuffer)
      continue;
    // Filter differences
    // TODO Check if jointPlayList can be deleted. Seems to be just a useless copy
    const Angle useJointPlayOffset = joint == lap || joint == rap ? jointPlayOffset : 0_deg;
    bufferValue[joint] = bufferValue[joint] * (1.f - useLowPassFilterFactor) +
                         (std::abs(theJointAngles.angles[getJoint(joint)] - bufferRequest[joint].back()) - (maxJointPlay[joint] - useJointPlayOffset)) * useLowPassFilterFactor;
    bufferValueShortTerm[joint] = bufferValueShortTerm[joint] * (1.f - lowpassFilterFactor.max) +
                                  (std::abs(theJointAngles.angles[getJoint(joint)] - bufferRequest[joint].back()) - (maxJointPlay[joint] - useJointPlayOffset)) * lowpassFilterFactor.max;

    jointPlayList[joint] = bufferValue[joint];
    jointPlaySum += std::max(jointPlayList[joint], 0_deg) * maxJointPlayRatio[joint];
  }

  if(!skipBuffer)
    theJointPlay.qualityOfRobotHardware = 1.f - Rangef::ZeroOneRange().limit((jointPlaySum - jointPlayScaling.min) / jointPlayScaling.max);

  PLOT("module:JointPlayProvider:lhp", jointPlayList[JointPlayTrack::lhp].toDegrees());
  PLOT("module:JointPlayProvider:lkp", jointPlayList[JointPlayTrack::lkp].toDegrees());
  PLOT("module:JointPlayProvider:lap", jointPlayList[JointPlayTrack::lap].toDegrees());
  PLOT("module:JointPlayProvider:rhp", jointPlayList[JointPlayTrack::rhp].toDegrees());
  PLOT("module:JointPlayProvider:rkp", jointPlayList[JointPlayTrack::rkp].toDegrees());
  PLOT("module:JointPlayProvider:rap", jointPlayList[JointPlayTrack::rap].toDegrees());

  PLOT("module:JointPlayProvider:lhps", bufferValueShortTerm[JointPlayTrack::lhp].toDegrees());
  PLOT("module:JointPlayProvider:lkps", bufferValueShortTerm[JointPlayTrack::lkp].toDegrees());
  PLOT("module:JointPlayProvider:laps", bufferValueShortTerm[JointPlayTrack::lap].toDegrees());
  PLOT("module:JointPlayProvider:rhps", bufferValueShortTerm[JointPlayTrack::rhp].toDegrees());
  PLOT("module:JointPlayProvider:rkps", bufferValueShortTerm[JointPlayTrack::rkp].toDegrees());
  PLOT("module:JointPlayProvider:raps", bufferValueShortTerm[JointPlayTrack::rap].toDegrees());

  PLOT("module:JointPlayProvider:sum", jointPlaySum.toDegrees());
  PLOT("module:JointPlayProvider:qualityOfRobotHardware", theJointPlay.qualityOfRobotHardware);
}

Joints::Joint JointPlayProvider::getJoint(JointPlayTrack joint)
{
  switch(joint)
  {
    case lhp:
      return Joints::lHipPitch;
    case lkp:
      return Joints::lKneePitch;
    case lap:
      return Joints::lAnklePitch;
    case rhp:
      return Joints::rHipPitch;
    case rkp:
      return Joints::rKneePitch;
    case rap:
      return Joints::rAnklePitch;
    default:
      ASSERT(false);
      return Joints::headYaw;
  }
}
