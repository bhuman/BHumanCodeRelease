/**
 * @file BoosterRobotStableStateProvider.h
 * Provides information about the predicted center of mass in the soles
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotStableState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"

MODULE(BoosterRobotStableStateProvider,
{,
  REQUIRES(FallDownState),
  REQUIRES(InertialData),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  PROVIDES(RobotStableState),
});

class BoosterRobotStableStateProvider : public BoosterRobotStableStateProviderBase
{
private:

  void update(RobotStableState& theRobotStableState) override;

  /**
   * Calculate the % position of the CoM, given the 0%, 50%, 75% and 100% points.
   */
  float calcPercentInFeet(const float refPoint, const float p0, const float p05, const float p075, const float p1);
};
