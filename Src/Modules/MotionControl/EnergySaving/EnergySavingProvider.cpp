/*
 * @file FilteredCurrentProvider.cpp
 * @author Philip Reichenberg
 */

#include "EnergySavingProvider.h"
#include <cmath>

MAKE_MODULE(EnergySavingProvider, motionControl);

EnergySavingProvider::EnergySavingProvider()
{
  std::fill(lastOffsets.begin(), lastOffsets.end(), 0_deg);
  std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
}

void EnergySavingProvider::update(EnergySaving& energySaving)
{
  // reset if no engine is using the energy saving mode
  if(state == State::deactive)
    energySaving.shutDown();
  else
  {
    lastState = state;
    state = State::deactive;
  }

  energySaving.shutDown = [this, &energySaving]()
  {
    motionChangeTimestamp = theFrameInfo.time;
    adjustTimestamp = theFrameInfo.time;
    energySaving.offsets.fill(0_deg);
    energySaving.state = EnergyState::waiting;
  };

  energySaving.reset = [&energySaving]()
  {
    energySaving.state = EnergyState::resetState;
  };

  energySaving.applyHeatAdjustment = [this, &energySaving](JointRequest& request, const bool adjustLegs, const bool adjustArms, const bool standHigh, const bool accuratePositions)
  {
    auto waitingFunc = [this, &energySaving]()
    {
      if(theFrameInfo.getTimeSince(motionChangeTimestamp) > motionChangeWaitTime)
      {
        hasGroundContact = theGroundContactState.contact;
        std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
        energySaving.state = EnergyState::working;
      }
    };

    auto resetFunc = [this, &energySaving]()
    {
      //reduce the offsets to 0 over the time of <resetTime>
      if(theFrameInfo.getTimeSince(lastInAdjustmentTimestamp) < usedResetInterpolation)
      {
        float ratio = static_cast<float>(theFrameInfo.getTimeSince(lastInAdjustmentTimestamp) / usedResetInterpolation);
        for(size_t i = 0; i < energySaving.offsets.size(); i++)
          energySaving.offsets[i] = lastOffsets[i] * (1 - ratio);
      }
      //after reduced to 0, check if new offsets need to be calculated, the module needs to wait or needs to be off
      else
      {
        std::fill(emergencyChangeCounter.begin(), emergencyChangeCounter.end(), stepsBeforeEmergencyStep);
        energySaving.offsets.fill(0_deg);
        if(theFrameInfo.getTimeSince(motionChangeTimestamp) > motionChangeWaitTime)
        {
          hasGroundContact = theGroundContactState.contact;
          energySaving.state = EnergyState::working;
        }
        else
          energySaving.state = EnergyState::waiting;
      }
    };

    auto workingPreFunc = [this, &energySaving]()
    {
      lastOffsets = energySaving.offsets;
      lastInAdjustmentTimestamp = theFrameInfo.time;
      //The offsets shall reset, if we switch from high stand to stand. (Or from stand to other keyframeMotion)
      if(theGroundContactState.contact != hasGroundContact)
      {
        motionChangeTimestamp = theFrameInfo.time;
        energySaving.state = EnergyState::resetState;
        return true;
      }
      return false;
    };

    auto workingPostFunc = [this, &energySaving]()
    {
      int offsetCapCounter = 0;
      int offsetSingleCap = 0;
      for(size_t i = Joints::firstLegJoint; i < energySaving.offsets.size(); i++)   //Only reset and cap based on legs
      {
        if(i == Joints::rHipYawPitch)   //skip rHipYawPitch
          continue;
        Angle capOffset = energySaving.offsets[i] > 0 ? std::min(energySaving.offsets[i], maxAngleMultipleJoints) : std::max(energySaving.offsets[i], -maxAngleMultipleJoints);
        if(capOffset != energySaving.offsets[i])
          offsetCapCounter++;
        capOffset = energySaving.offsets[i] > 0 ? std::min(energySaving.offsets[i], maxAngleOneJoint) : std::max(energySaving.offsets[i], -maxAngleOneJoint);
        if(capOffset != energySaving.offsets[i])
          offsetSingleCap++;
        energySaving.offsets[i] = capOffset;
      }
      if(offsetCapCounter >= minNumberForHeatAdjustmentReset || offsetSingleCap != 0)
      {
        lastOffsets = energySaving.offsets;
        energySaving.state = EnergyState::resetState;
      }
    };

    // Energy Saving Move is active
    if(!accuratePositions)
    {
      state = State::energySaving;
      if(lastState == State::accuratePosition)
      {
        lastOffsets = energySaving.offsets;
        energySaving.state = EnergyState::resetState;
      }

      switch(energySaving.state)
      {
        case EnergyState::waiting:
        {
          //wait a moment to make sure the robot reached it new position
          waitingFunc();
          break;
        }
        case EnergyState::working:
        {
          if(workingPreFunc())
            break;
          //Adjust once in a while
          if(theFrameInfo.getTimeSince(adjustTimestamp) > adjustWaitTime)
          {
            adjustTimestamp = theFrameInfo.time;
            std::vector<Angle> jointDifs(Joints::numOfJoints);
            for(size_t i = 0; i < energySaving.offsets.size(); i++)
            {
              //skip joint that can not be effectively adjusted
              bool skip = false;
              for(Joints::Joint joint : skipJoints)
                if(joint == i)
                  skip = true;
              if(skip)
                continue;
              jointDifs[i] = theJointAngles.angles[i] - request.angles[i];
              if((i >= Joints::firstLegJoint && theFilteredCurrent.currents[i] > currentThresholdLegs && adjustLegs)
                 || (i < Joints::firstLegJoint && theFilteredCurrent.currents[i] > currentThresholdArms && adjustArms)) //if current is above the threshold, change the offset
              {
                //It can happen, that the offset changes around the current measured joint value.
                //In such a case, we adjust by maxGearStep, because if the joint moved a bit more,
                //the current often jumps down to a low value.
                emergencyChangeCounter[i] -= 1;
                if(emergencyChangeCounter[i] <= 0)
                {
                  // TODO this stupid stuff needs to be replaced ...
                  int sign = !standHigh ? standSigns[i] : highStandSigns[i];
                  energySaving.offsets[i] += i < Joints::firstLegJoint ? (maxGearStepArms * sign) : (maxGearStepLegs * sign);
                  emergencyChangeCounter[i] = stepsBeforeEmergencyStep;
                }
                else
                {
                  //adjust step is smaller than the minimal step the gears can do
                  //this help to reach a better optimal offset
                  //check if we can do a big step to adjust the arm as fast as possible
                  Angle dif = std::abs(jointDifs[i]);
                  Angle step = i < Joints::firstLegJoint && dif > maxGearStepArms ? maxGearStepArms :
                               (dif > maxGearStepLegs && i >= Joints::firstLegJoint ? maxGearStepLegs : minGearStep);
                  energySaving.offsets[i] += (jointDifs[i] > 0 ? 1 : -1) * step;
                }
              }
            }
            //cap the offsets and count how many offsets are capped. If too many are capped, reset the offsets
            workingPostFunc();
          }
          break;
        }
        case EnergyState::resetState:
        {
          resetFunc();
          break;
        }
      }
    }
    // Calibration Mode is active
    else
    {
      state = State::accuratePosition;
      if(lastState == State::energySaving)
      {
        lastOffsets = energySaving.offsets;
        energySaving.state = EnergyState::resetState;
      }

      switch(energySaving.state)
      {
        case EnergyState::waiting:
        {
          //wait a moment to make sure the robot reached it new position
          waitingFunc();
          break;
        }
        case EnergyState::working:
        {
          if(workingPreFunc())
            break;
          //Adjust once in a while
          if(theFrameInfo.getTimeSince(adjustTimestamp) > adjustWaitTime)
          {
            adjustTimestamp = theFrameInfo.time;
            std::vector<Angle> jointDifs(Joints::numOfJoints);
            for(size_t i = 0; i < energySaving.offsets.size(); i++)
            {
              if(i < Joints::firstLegJoint)
                continue;

              jointDifs[i] = request.angles[i] - theJointAngles.angles[i];
              if(minGearStep < std::abs(jointDifs[i]))
              {
                //It can happen, that the offset changes around the current measured joint value.
                //In such a case, we adjust by maxGearStep, because if the joint moved a bit more,
                //the current often jumps down to a low value.
                emergencyChangeCounter[i] -= 1;
                if(emergencyChangeCounter[i] <= 0)
                {
                  energySaving.offsets[i] += maxGearStepLegs;
                  emergencyChangeCounter[i] = stepsBeforeEmergencyStep;
                }
                else
                {
                  //adjust step is smaller than the minimal step the gears can do
                  //this help to reach a better optimal offset
                  //check if we can do a big step to adjust the arm as fast as possible
                  Angle dif = std::abs(jointDifs[i]);
                  Angle step = dif > maxGearStepLegs && i >= Joints::firstLegJoint ? maxGearStepLegs : Angle(minGearStep / 2.f);
                  energySaving.offsets[i] += (jointDifs[i] > 0 ? 1 : -1) * step;
                }
              }
            }
            //cap the offsets and count how many offsets are capped. If too many are capped, reset the offsets
            workingPostFunc();
          }
          break;
        }
        case EnergyState::resetState:
        {
          resetFunc();
          break;
        }
      }
    }
    FOREACH_ENUM(Joints::Joint, joint)
      request.angles[joint] += energySaving.offsets[joint];
  };
}
