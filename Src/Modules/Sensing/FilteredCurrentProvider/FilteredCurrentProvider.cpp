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
  for(auto& filter : currents)
    filter.reserve(filterSize);
}

void FilteredCurrentProvider::update(FilteredCurrent& theFilteredCurrent)
{
  DEBUG_RESPONSE("module:FilteredCurrentProvider:resizeFilter")
  {
    for(auto& filter : currents)
      filter.reserve(filterSize);
  }

  theFilteredCurrent.legPitchCurrentSums[Legs::left] = 0.f;
  theFilteredCurrent.legPitchCurrentSums[Legs::right] = 0.f;

  // Filter the currents
  Joints::Joint legJointWithHighestCurrent = Joints::lHipPitch;
  for(std::size_t i = 0; i < Joints::numOfJoints; i++)
  {
    int value = static_cast<int>(theJointSensorData.currents[i]);
    currents[i].push_front(static_cast<int>(value == SensorData::off ? 1 : value));
    if(!currents[i].full())
      continue;
    theFilteredCurrent.currents[i] = currents[i].average();
    if(i > Joints::firstLeftLegJoint)
    {
      if(i < Joints::firstRightLegJoint)//if(i == Joints::lHipPitch || i == Joints::lKneePitch || i == Joints::lAnklePitch)
        theFilteredCurrent.legPitchCurrentSums[Legs::left] += std::abs(currents[i].average());
      else//if(i == Joints::rHipPitch || i == Joints::rKneePitch || i == Joints::rAnklePitch)
        theFilteredCurrent.legPitchCurrentSums[Legs::right] += std::abs(currents[i].average());
    }
    if(i > Joints::firstLeftLegJoint &&
       std::abs(theFilteredCurrent.currents[i]) > std::abs(theFilteredCurrent.currents[legJointWithHighestCurrent]))
    {
      legJointWithHighestCurrent = static_cast<Joints::Joint>(i);
    }
  }
  theFilteredCurrent.isValid = true;
  theFilteredCurrent.legJointWithHighestCurrent = legJointWithHighestCurrent;

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
    //but using ring buffers for the average joint difference (of request and measured) can (maybe) lead to more false-positives.
    //But also for a faster detection -> TODO for future
    //Right now it takes like 5 secs before a motor malfunction is detected. Also based on the logs it has no false-positives (yet), but some false-negatives.
    checkTimestamp = theFrameInfo.time;
    for(size_t i = 0; i < currents.size(); i++)
    {
      // Decide which threshold to use
      Angle jointDiff = theGroundContactState.contact ? minJointDiffNormalJoints : minJointDiffNormalJointsNoGroundContact;
      int stiffness = stiffnessThreshold;
      if(i == Joints::lAnkleRoll || i == Joints::rAnkleRoll)
        jointDiff = minJointDiffAnkleRoll;
      else if(i >= Joints::firstArmJoint && i < Joints::firstLegJoint)
      {
        stiffness = stiffnessThresholdArms;
        jointDiff = minJointDiffArms;
      }
      // If current is 0, the stiffness high enough and the jointRequest and jointAngle difference is high enough, increase the counter
      if(currents[i].average() == 0 && theJointRequest.stiffnessData.stiffnesses[i] >= stiffness && std::abs(theJointRequest.angles[i] - theJointAngles.angles[i]) >= jointDiff)
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
        sum += flags[0] > flagsThreshold && zeroCheck.isInside(theJointAngles.angles[joint]) ? 1 : 0;
      if(sum > 1)
      {
        soundTimestamp = theFrameInfo.time;
        SystemCall::playSound("siren", true);
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
              SystemCall::playSound("siren", true);
              SystemCall::say("Motor malfunction!", true);
              SystemCall::say((std::string(TypeRegistry::getEnumName(theGameState.color())) + " " + std::to_string(theGameState.playerNumber) + " " + TypeRegistry::getEnumName(Joints::Joint(i))).c_str(), true);
            }
            playedSound = true;
            if(!theGyroOffset.bodyDisconnect && SystemCall::getMode() != SystemCall::logFileReplay)
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
