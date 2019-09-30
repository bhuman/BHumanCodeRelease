/*
 * @file FilteredCurrentProvider.cpp
 * @author Philip Reichenberg
 */

#include "EnergySavingProvider.h"

MAKE_MODULE(EnergySavingProvider, motionControl)

EnergySavingProvider::EnergySavingProvider()
{
  motionChangeTimestamp = 0;
  adjustTimestamp = 0;
  lastInAdjustmentTimestamp = 0;
  standHeatInSpecialAction = false;
  hasGroundContact = false;
  for(size_t i = 0; i < lastOffsets.size(); i++)
    lastOffsets[i] = 0_deg;
  state = off;
  std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
}

void EnergySavingProvider::update(EnergySaving& energySaving)
{
  switch(state)
  {
    case off:
    {
      if(shouldBeActive())
        state = waiting;
      else
      {
        //Not activ -> set timestamps and set offsets to 0
        motionChangeTimestamp = theFrameInfo.time;
        adjustTimestamp = theFrameInfo.time;
        energySaving.offsets.fill(0_deg);
      }
      break;
    }
    case waiting:
    {
      //Should not be activ -> off
      if(!shouldBeActive())
        state = off;
      //wait a moment to make sure the robot reached it new position
      else if(theFrameInfo.getTimeSince(motionChangeTimestamp) > motionChangeWaitTime)
      {
        hasGroundContact = theGroundContactState.contact;
        std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
        state = working;
        standHeatInSpecialAction = theMotionRequest.motion == MotionRequest::specialAction;
      }
      break;
    }
    case working:
    {
      lastInAdjustmentTimestamp = theFrameInfo.time;
      //The offsets shall reset, if we switch from highstand to stand. (Or from stand to other specialActions)
      if(theGroundContactState.contact != hasGroundContact || (standHeatInSpecialAction && theMotionRequest.motion == MotionRequest::stand) || (!standHeatInSpecialAction && theMotionRequest.motion == MotionRequest::specialAction) || !shouldBeActive())
      {
        motionChangeTimestamp = theFrameInfo.time;
        standHeatInSpecialAction = !standHeatInSpecialAction;
        state = reset;
        break;
      }
      //Adjust once in a while
      if(theFrameInfo.getTimeSince(adjustTimestamp) > adjustWaitTime)
      {
        adjustTimestamp = theFrameInfo.time;
        std::vector<Angle> jointDifs(Joints::numOfJoints);
        for(size_t i = 0; i < energySaving.offsets.size(); i++)
        {
          //skip joint that can not be effectivly adjusted
          bool skip = false;
          for(Joints::Joint joint : skipJoints)
            if(joint == i)
              skip = true;
          if(skip)
            continue;
          jointDifs[i] = theJointAngles.angles[i] - theJointRequest.angles[i];
          if((i >= Joints::firstLegJoint && theFilteredCurrent.currents[i] > currentThresholdLegs)
             || (i < Joints::firstLegJoint && theFilteredCurrent.currents[i] > currentThresholdArms && allowedToAdjustArms())) //if current is above the threshold, change the offset
          {
            //It can happen, that the offset changes arround the current meassured joint value.
            //In such a case, we adjust by maxGearStep, because if the joint moved a bit more,
            //the current often jumps down to a low number.
            emergencyChangeCounter[i] -= 1;
            if(emergencyChangeCounter[i] <= 0)
            {
              int sign = theMotionRequest.motion == MotionRequest::stand ? standSigns[i] : highStandSigns[i];
              energySaving.offsets[i] += i < Joints::firstLegJoint ? (maxGearStepArms * sign) : (maxGearStepLegs * sign);
              emergencyChangeCounter[i] = stepsBeforeEmergencyStep;
            }
            else
            {
              //adjust step is smaller than the minimal step the gears can do
              //this help to reach a better optimal offset
              //check if we can do a big step to adjust the arm as fast as possible
              Angle dif = std::fabs(jointDifs[i]);
              Angle step = i < Joints::firstLegJoint && dif > maxGearStepArms ? maxGearStepArms :
                           (dif > maxGearStepLegs && i >= Joints::firstLegJoint ? maxGearStepLegs : minGearStep);
              energySaving.offsets[i] += (jointDifs[i] > 0 ? 1 : -1) * step;
            }
          }
        }
        //cap the offsets and count how many offsets are capped. If too many are capped, reset the offsets
        int offsetCapCounter = 0;
        int offsetSingleCap = 0;
        for(size_t i = Joints::firstLegJoint; i < energySaving.offsets.size(); i++) //Only reset and cap based on legs
        {
          if(i == Joints::rHipYawPitch) //skip rHipYawPitch
            continue;
          Angle capOffset = energySaving.offsets[i] > 0 ? std::min(energySaving.offsets[i], maxAngleMultipleJoints) : std::max(energySaving.offsets[i], -maxAngleMultipleJoints);
          if(capOffset != energySaving.offsets[i])
            offsetCapCounter++;
          capOffset = energySaving.offsets[i] > 0 ? std::min(energySaving.offsets[i], maxAngleOneJoint) : std::max(energySaving.offsets[i], -maxAngleOneJoint);
          if(capOffset != energySaving.offsets[i])
            offsetSingleCap++;
          energySaving.offsets[i] = capOffset;
        }
        lastOffsets = energySaving.offsets;
        if(offsetCapCounter >= minNumberForHeatAdjustmentReset || offsetSingleCap != 0)
          state = reset;
      }
      break;
    }
    case reset:
    {
      //reduce the offsets to 0 over the time of <resetTime>
      if(theFrameInfo.getTimeSince(lastInAdjustmentTimestamp) < resetTime)
      {
        float ratio = static_cast<float>(theFrameInfo.getTimeSince(lastInAdjustmentTimestamp) / resetTime);
        for(size_t i = 0; i < energySaving.offsets.size(); i++)
          energySaving.offsets[i] = lastOffsets[i] * (1 - ratio);
      }
      //after reduced to 0, check if new offsets need to be calculated, the module needs to wait or needs to be off
      else if(shouldBeActive())
      {
        std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
        energySaving.offsets.fill(0_deg);
        if(theFrameInfo.getTimeSince(motionChangeTimestamp) > motionChangeWaitTime)
        {
          hasGroundContact = theGroundContactState.contact;
          state = working;
        }
        else
          state = waiting;
      }
      else
        state = off;
      break;
    }
  }
}

bool EnergySavingProvider::shouldBeActive()
{
  //All motions where the module shall be active
  if(theFilteredCurrent.isValid
     && ((theMotionRequest.motion == MotionRequest::stand && theLegMotionSelection.ratios[MotionRequest::stand] == 1.f)
         || (theMotionRequest.motion == MotionRequest::specialAction
             && (theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh
                 || theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHighLookUp)
             && theLegMotionSelection.ratios[MotionRequest::specialAction] == 1.f)))
    return true;
  return false;
}

bool EnergySavingProvider::allowedToAdjustArms()
{
  //All motions where the module shall be active
  if(theFilteredCurrent.isValid
     && ((theMotionRequest.motion == MotionRequest::specialAction
          && (theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh
              || theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHighLookUp)
          && theLegMotionSelection.ratios[MotionRequest::specialAction] == 1.f)))
    return true;
  return false;
}
