/**
 * @file StandBodyRotationProvider.cpp
 * Implementation of the StandBodyRotationProvider
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "StandBodyRotationProvider.h"

#include <algorithm>

MAKE_MODULE(StandBodyRotationProvider, motionControl)

void StandBodyRotationProvider::update(StandBodyRotation& standBodyRotation)
{
  ASSERT(i < amountToAverage);

  if(theLegMotionSelection.ratios[MotionRequest::stand] != 1.f)
  {
    if(standBodyRotation.bodyRotation.x() < 0.f)
      standBodyRotation.bodyRotation = Vector2f(std::max(standBodyRotation.bodyRotation.x(), 0.5f * -maxXRotation * theLegMotionSelection.ratios[MotionRequest::stand]), 0.f);
    else
      standBodyRotation.bodyRotation = Vector2f(std::min(standBodyRotation.bodyRotation.x(), 0.5f * maxXRotation * theLegMotionSelection.ratios[MotionRequest::stand]), 0.f);

    i = leftSum = rightSum = 0;
    changeValue = 0.f;
    return;
  }

  if(std::any_of(theJointSensorData.currents.begin(), theJointSensorData.currents.end(), [](float c) { return c == SensorData::off; }))
  {
    leftSum = rightSum = 0;
    changeValue = 0.f;
  }
  else
  {
    for(int joint = Joints::lHipRoll; joint <= Joints::lAnkleRoll; ++joint)
      leftSum += theJointSensorData.currents[joint];
    for(int joint = Joints::rHipRoll; joint <= Joints::rAnkleRoll; ++joint)
      rightSum += theJointSensorData.currents[joint];
  }

  if(++i == amountToAverage)
  {
    const unsigned leftAvg = leftSum / amountToAverage;
    const unsigned rightAvg = rightSum / amountToAverage;
    const unsigned sumAvg = leftAvg + rightAvg;
    if(sumAvg != 0.f)
      changeValue = (leftAvg / sumAvg - 0.5f) * maxXRotation / minStepsToFullRoation / amountToAverage;

    i = leftSum = rightSum = 0;
  }

  standBodyRotation.bodyRotation = Vector2f(std::max(-maxXRotation / 2.f, std::min(maxXRotation / 2.f, (standBodyRotation.bodyRotation.x() + changeValue))), 0.f);
}
