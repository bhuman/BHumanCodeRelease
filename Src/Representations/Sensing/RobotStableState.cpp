/**
 * @file RobotStableState.cpp
 * This representation holds information of the CoM position in the foot area.
 * @author Philip Reichenberg
 */

#include "RobotStableState.h"
#include "Debugging/Plot.h"

void RobotStableState::draw()
{
  PLOT("representation:RobotStableState:prediction:x", predictedTorsoRotation.x().toDegrees());
  PLOT("representation:RobotStableState:prediction:y", predictedTorsoRotation.y().toDegrees());
}
