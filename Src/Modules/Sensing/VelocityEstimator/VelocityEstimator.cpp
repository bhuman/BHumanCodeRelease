/**
 * @file VelocityEstimator.cpp
 * Implementation of module VelocityEstimator.
 * @author Colin Graf
 */

#include "VelocityEstimator.h"
#include "Framework/Settings.h"

MAKE_MODULE(VelocityEstimator);

void VelocityEstimator::update(VelocityEstimation& theVelocityEstimation)
{
  theVelocityEstimation.rotation = theVelocityEstimation.rotation * 0.9f + theOdometryDataPreview.odometryChange.rotation * 0.1f / Global::getSettings().motionCycleTime;
  theVelocityEstimation.translation = theVelocityEstimation.translation * 0.9f + theOdometryDataPreview.odometryChange.translation * 0.1f / Global::getSettings().motionCycleTime;

  lastEstimations.push_front(theVelocityEstimation);

  theVelocityEstimation.highestLastQuarterSecond = Pose2f();
  if(lastEstimations.full())
  {
    for(std::size_t index = 0; index < lastEstimations.capacity(); index++)
    {
      theVelocityEstimation.highestLastQuarterSecond.rotation = std::abs(lastEstimations[index].rotation) > std::abs(theVelocityEstimation.highestLastQuarterSecond.rotation) ? lastEstimations[index].rotation : theVelocityEstimation.highestLastQuarterSecond.rotation;
      theVelocityEstimation.highestLastQuarterSecond.translation.x() = std::abs(lastEstimations[index].translation.x()) > std::abs(theVelocityEstimation.highestLastQuarterSecond.translation.x()) ? lastEstimations[index].translation.x() : theVelocityEstimation.highestLastQuarterSecond.translation.x();
      theVelocityEstimation.highestLastQuarterSecond.translation.y() = std::abs(lastEstimations[index].translation.y()) > std::abs(theVelocityEstimation.highestLastQuarterSecond.translation.y()) ? lastEstimations[index].translation.y() : theVelocityEstimation.highestLastQuarterSecond.translation.y();
    }
  }
}
