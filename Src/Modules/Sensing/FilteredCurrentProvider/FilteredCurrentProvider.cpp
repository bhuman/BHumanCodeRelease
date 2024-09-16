/*
 * @file FilteredCurrentProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "FilteredCurrentProvider.h"
#include "Debugging/Plot.h"
#include <cmath>

MAKE_MODULE(FilteredCurrentProvider);

FilteredCurrentProvider::FilteredCurrentProvider()
{
  currents.resize(Joints::numOfJoints);
  flags.resize(Joints::numOfJoints);
  fill(flags.begin(), flags.end(), 0);
}

void FilteredCurrentProvider::update(FilteredCurrent& theFilteredCurrent)
{
  DECLARE_PLOT("module:FilteredCurrentProvider:lHYP");
  DECLARE_PLOT("module:FilteredCurrentProvider:lHP");
  DECLARE_PLOT("module:FilteredCurrentProvider:lKP");
  DECLARE_PLOT("module:FilteredCurrentProvider:lAP");
  DECLARE_PLOT("module:FilteredCurrentProvider:rHP");
  DECLARE_PLOT("module:FilteredCurrentProvider:rKP");
  DECLARE_PLOT("module:FilteredCurrentProvider:rAP");

  // Filter the currents
  Joints::Joint legJointWithHighestCurrent = Joints::lHipPitch;
  for(std::size_t i = 0; i < Joints::numOfJoints; i++)
  {
    short value = theJointSensorData.currents[i];
    currents[i].push_front(static_cast<int>(value == SensorData::off ? 1.f : value));
    theFilteredCurrent.currents[i] = currents[i].average();
    if(i > Joints::firstLeftLegJoint &&
       theFilteredCurrent.currents[i] > theFilteredCurrent.currents[legJointWithHighestCurrent])
    {
      legJointWithHighestCurrent = static_cast<Joints::Joint>(i);
    }
  }
  theFilteredCurrent.isValid = true;
  theFilteredCurrent.legJointWithHighestCurrent = legJointWithHighestCurrent;
  PLOT("module:FilteredCurrentProvider:lHYP", theFilteredCurrent.currents[Joints::lHipYawPitch]);
  PLOT("module:FilteredCurrentProvider:lHP", theFilteredCurrent.currents[Joints::lHipPitch]);
  PLOT("module:FilteredCurrentProvider:lKP", theFilteredCurrent.currents[Joints::lKneePitch]);
  PLOT("module:FilteredCurrentProvider:lAP", theFilteredCurrent.currents[Joints::lAnklePitch]);
  PLOT("module:FilteredCurrentProvider:rHP", theFilteredCurrent.currents[Joints::rHipPitch]);
  PLOT("module:FilteredCurrentProvider:rKP", theFilteredCurrent.currents[Joints::rKneePitch]);
  PLOT("module:FilteredCurrentProvider:rAP", theFilteredCurrent.currents[Joints::rAnklePitch]);

  // Check for a motor malfunction
  checkMotorMalfunction(theFilteredCurrent);

  // If the gyro is stuck, we assume the whole connection to the robot disconnected. In such a case, we can not detect a motor malfunction.
  if(theGyroOffset.bodyDisconnect)
    theFilteredCurrent.legMotorMalfunction = false;
}

void FilteredCurrentProvider::checkMotorMalfunction(FilteredCurrent& theFilteredCurrent)
{
  if(theFrameInfo.getTimeSince(checkTimestamp) > checkWaitTime)
  {
    //The whole idea with managing the flags is not great,
    //but using ringbuffers for the average joint difference (of request and measured) can (maybe) lead to more false-positives.
    //But also for a faster detection -> TODO for future
    //Right now it takes like 5 secs before a motormalfunction is detected. Also based on the logs it has no false-positives (yet), but some false-negatives.
    checkTimestamp = theFrameInfo.time;
    for(size_t i = 0; i < currents.size(); i++)
    {
      // Decide which threshold to use
      Angle jointDiff = theGroundContactState.contact ? minJointDiffNormalJoints : minJointDiffNormalJointsNoGroundConntact;
      int stiffness = stiffnessThreshold;
      if(i == Joints::lAnkleRoll || i == Joints::rAnkleRoll)
        jointDiff = minJointDiffAnkleRoll;
      else if(i >= Joints::firstArmJoint && i < Joints::firstLegJoint)
      {
        stiffness = stiffnessThresholdArms;
        jointDiff = minJointDiffArms;
      }
      // If current is 0, the stiffness high enough and the jointRequest and jointAngle difference is high enough, increase the counter
      if(currents[i].average() == 0 && theJointRequest.stiffnessData.stiffnesses[i] >= stiffness && std::abs(theJointRequest.angles[i] - theJointSensorData.angles[i]) >= jointDiff)
        flags[i] += 1;
      else
        flags[i] = std::max(flags[i] - 1, 0); // subtract by 1 is better than setting to 0, in case we did a check at a bad time, where the deactivated joint had a correct position
    }

    // In a very rare case it can happen that the upper body has no stiffness and no sensor data.
    // This part checks for this situations if it happens at the very beginning of the robot usage
    if(theFrameInfo.getTimeSince(soundTimestamp) >= motorMalfunctionTime)
    {
      int sum = 0;
      const Rangea zeroCheck(-zeroCheckRange, zeroCheckRange);
      for(Joints::Joint joint : specialMotorCheck)
        sum += flags[0] > flagsThreshold && zeroCheck.isInside(theJointSensorData.angles[joint]) ? 1 : 0;
      if(sum > 1)
      {
        soundTimestamp = theFrameInfo.time;
        SystemCall::playSound("sirene.wav", true);
        SystemCall::say("Motor malfunction! Upper body!", true);
        SystemCall::say((std::string(TypeRegistry::getEnumName(theGameState.color())) + " " + std::to_string(theGameState.playerNumber)).c_str(), true);
        OUTPUT_TEXT("hee" << " " << theFrameInfo.time);
      }
    }

    if(theFrameInfo.getTimeSince(soundTimestamp) >= motorMalfunctionTime)
    {
      bool playedSound = false;
      for(size_t i = flags.size() - 1; i > 0; i--) // reverse, first check legs, then arms, then head
      {
        // Are enough possible motor malfunction detected?
        if(flags[i] >= flagsThreshold)
        {
          flags[i] = 0;
          bool isUninterestingJoint = false;
          // Ignore specific joints
          for(size_t j = 0; j < ignoreJoints.size(); j++)
            isUninterestingJoint = isUninterestingJoint || i == ignoreJoints[j];
          if(isUninterestingJoint)
            continue;

          soundTimestamp = theFrameInfo.time;
          if(!playedSound && i >= Joints::firstArmJoint && i < Joints::firstLegJoint && (theMotionInfo.isMotion(MotionPhase::walk) || theMotionInfo.isMotion(MotionPhase::stand)))
          {
            SystemCall::say(std::string("Arms motor malfunction").c_str());
            playedSound = true;
          }
          else
          {
            if(!playedSound)
            {
              SystemCall::playSound("sirene.wav", true);
              SystemCall::say("Motor malfunction!", true);
              SystemCall::say((std::string(TypeRegistry::getEnumName(theGameState.color())) + " " + std::to_string(theGameState.playerNumber) + " " + TypeRegistry::getEnumName(Joints::Joint(i))).c_str(), true);
            }
            playedSound = true;
            if(!theGyroOffset.bodyDisconnect)
              theFilteredCurrent.legMotorMalfunction = true;
          }
          if(theFrameInfo.getTimeSince(annotationTimestamp) > annotationWaitTime)
          {
            annotationTimestamp = theFrameInfo.time;
            ANNOTATION("FilteredCurrentProvider", TypeRegistry::getEnumName((static_cast<Joints::Joint>(i))));
          }
        }
        continue;
      }
    }
  }
}
