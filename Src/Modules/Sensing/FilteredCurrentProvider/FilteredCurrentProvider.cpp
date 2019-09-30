/*
 * @file FilteredCurrentProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "FilteredCurrentProvider.h"
#include "Tools/Settings.h"

MAKE_MODULE(FilteredCurrentProvider, sensing)

FilteredCurrentProvider::FilteredCurrentProvider()
{
  currents.resize(Joints::numOfJoints);
  flags.resize(Joints::numOfJoints);
  fill(flags.begin(), flags.end(), 0);
  checkTimestamp = 0;
  soundTimestamp = 0;
  annotationTimestamp = 0;
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
  for(int i = 0; i < Joints::numOfJoints; i++)
  {
    short value = theJointSensorData.currents[i];
    currents[i].push_front(static_cast<int>(value == SensorData::off ? 1.f : value));
    theFilteredCurrent.currents[i] = currents[i].average();
  }
  theFilteredCurrent.isValid = true;
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
  if(theGyroOffset.gyroIsStuck)
    theFilteredCurrent.legMotorMalfunction = false;
}

void FilteredCurrentProvider::checkMotorMalfunction(FilteredCurrent& theFilteredCurrent)
{
  if(theFrameInfo.getTimeSince(checkTimestamp) > checkWaitTime)
  {
    //The whole idea with managing the flags is not great,
    //but using ringbuffers for the average joint difference (of request and meassured) can (maybe) lead to more false-positives.
    //But also for a faster detection -> TODO for future
    //Right now it takes like 5 secs before a motormalfunction is detected. Also based on the logs it has no false-positives (yet), but some false-negatives.
    checkTimestamp = theFrameInfo.time;
    for(size_t i = 0; i < currents.size(); i++)
    {
      // Decide which threshold to use
      Angle jointDif = theGroundContactState.contact ? minJointDifNormalJoints : minJointDifNormalJointsNoGroundConntact;
      int stiffness = stiffnessThreshold;
      if(i == Joints::lAnkleRoll || i == Joints::rAnkleRoll)
        jointDif = minJointDifAnkleRoll;
      else if(i >= Joints::firstArmJoint && i < Joints::firstLegJoint)
      {
        stiffness = stiffnessThresholdArms;
        jointDif = minJointDifArms;
      }
      // If current is 0, the stiffness high enough and the jointRequest and jointAngle difference is high enough, increase the counter
      if(currents[i].average() == 0 && theJointRequest.stiffnessData.stiffnesses[i] >= stiffness && std::fabs(theJointRequest.angles[i] - theJointSensorData.angles[i]) >= jointDif)
        flags[i] += 1;
      else
        flags[i] = std::max(flags[i] - 1, 0); // subtract by 1 is better than setting to 0, in case we did a check at a bad time, where the deactivated joint had a correct position
    }
    for(size_t i = 0; i < flags.size(); i++)
    {
      // Are enough possible motor malfunction detected?
      if(flags[i] >= flagsThreshold)
      {
        if(theFrameInfo.getTimeSince(soundTimestamp) >= motorMalfunctionTime)
        {
          flags[i] = 0;
          soundTimestamp = theFrameInfo.time;
          bool isUninteresstingJoint = false;
          //Ignore specific joints
          for(size_t j = 0; j < ignoreJoints.size(); j++)
            isUninteresstingJoint = isUninteresstingJoint || i == ignoreJoints[j];
          if(isUninteresstingJoint)
            continue;
          //arms shall not play motorMalFunction
          if(i >= Joints::firstArmJoint && i < Joints::firstLegJoint)
          {
            SystemCall::say(std::string("Arm damaged").c_str()); //probably only playing while doing get ups
          }
          else
          {
            SystemCall::playSound("sirene.wav");
            SystemCall::say((std::string("Motor malfunction ") + TypeRegistry::getEnumName(Global::getSettings().teamColor) + " " + std::to_string(theRobotInfo.number)).c_str());
            if(!theGyroOffset.gyroIsStuck)
              theFilteredCurrent.legMotorMalfunction = true;
          }
          if(theFrameInfo.getTimeSince(annotationTimestamp) > annotationWaitTime)
          {
            annotationTimestamp = theFrameInfo.time;
            ANNOTATION("FilteredCurrentProvider", TypeRegistry::getEnumName((static_cast<Joints::Joint>(i))));
          }
        }
        return;
      }
    }
  }
}
